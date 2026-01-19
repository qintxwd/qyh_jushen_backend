"""
QYH Jushen Control Plane - 机器人信息 API

提供机器人基本信息查询:
- 机器人信息（机型、配置）
- URDF 模型获取
- 状态概览（子系统连接状态）

注意: 这里只提供低频查询，高频状态推送走 data_plane WebSocket
"""
import os
import logging
import time
import glob
from datetime import datetime
from pathlib import Path
from typing import Optional, List, Tuple

from fastapi import APIRouter, Depends
from fastapi.responses import Response

from app.dependencies import get_current_user, get_current_admin
from app.models.user import User
from app.schemas.response import (
    ApiResponse,
    ErrorCodes,
    success_response,
    error_response,
)
from app.schemas.robot import (
    RobotInfo,
    RobotOverview,
    SubsystemStatus,
    SystemState,
    BaseState,
    GripperState,
    ShutdownState,
    ShutdownRequest,
)
from app.services.health_checker import get_health_checker, ServiceStatus
from app.services.ros2_client import get_ros2_client, ROS2ServiceClient
from app.core.mode_manager import mode_manager

logger = logging.getLogger(__name__)
router = APIRouter()

# 配置
ROBOT_NAME = os.environ.get('GLOBAL_ROBOT_NAME', 'general')
ROBOT_VERSION = os.environ.get('GLOBAL_ROBOT_VERSION', '1.0')
URDF_PATH = os.path.expanduser("~/qyh-robot-system/config/robot.urdf")


# ============================================================================
# ROS2 集成
# ============================================================================


class ROS2Bridge:
    """ROS2 桥接（使用 ROS2ServiceClient）"""
    
    def __init__(self):
        self._client: Optional[ROS2ServiceClient] = None
    
    def _get_client(self) -> ROS2ServiceClient:
        """获取 ROS2 客户端"""
        if self._client is None:
            self._client = get_ros2_client()
        return self._client
    
    def is_connected(self) -> bool:
        """检查 ROS2 是否连接"""
        try:
            client = self._get_client()
        except RuntimeError:
            return False
        return client._node is not None
    
    def get_robot_state(self) -> Optional[dict]:
        """获取机器人状态"""
        client = self._get_client()
        return client.get_robot_state()
    
    def get_shutdown_state(self) -> dict:
        """获取关机状态"""
        client = self._get_client()
        return client.get_shutdown_state()

    async def apply_chassis_config_if_needed(
        self,
        chassis_connected: bool,
    ) -> None:
        """底盘连接后应用持久化配置"""
        client = self._get_client()
        if not client._initialized:
            await client.initialize()
        await client.apply_chassis_config_if_needed(chassis_connected)
    
    async def call_shutdown(self, reason: str = "", delay: int = 0) -> dict:
        """调用关机服务"""
        client = self._get_client()
        if not client._initialized:
            await client.initialize()
        
        result = await client.request_shutdown()
        return {
            "success": result.success,
            "message": result.message
        }
    
    async def call_reboot(self) -> dict:
        """调用重启服务"""
        client = self._get_client()
        if not client._initialized:
            await client.initialize()
        
        result = await client.request_reboot()
        return {
            "success": result.success,
            "message": result.message
        }


# 全局实例
ros2_bridge = ROS2Bridge()


# ============================================================================
# 辅助函数
# ============================================================================

async def _get_subsystem_status() -> list:
    """获取子系统状态列表"""
    now = datetime.now().isoformat()
    health_checker = get_health_checker()
    results = await health_checker.check_all()

    def _map_status(result_name: str, display_name: str) -> SubsystemStatus:
        result = results.get(result_name)
        if not result:
            return SubsystemStatus(
                name=display_name,
                connected=False,
                status="unknown",
                message="状态未获取",
            )

        if result.status == ServiceStatus.HEALTHY:
            status = "ok"
            connected = True
        elif result.status == ServiceStatus.DEGRADED:
            status = "warning"
            connected = True
        elif result.status == ServiceStatus.UNHEALTHY:
            status = "error"
            connected = False
        else:
            status = "unknown"
            connected = False

        return SubsystemStatus(
            name=display_name,
            connected=connected,
            status=status,
            message=result.message or "",
            last_seen=now if connected else None,
        )

    subsystems = [
        _map_status("ros2", "ROS2 Core"),
        _map_status("data_plane", "Data Plane"),
        _map_status("media_plane", "Media Plane"),
        SubsystemStatus(
            name="Left Arm",
            connected=False,
            status="unknown",
            message="状态源未接入",
        ),
        SubsystemStatus(
            name="Right Arm",
            connected=False,
            status="unknown",
            message="状态源未接入",
        ),
        SubsystemStatus(
            name="Head",
            connected=False,
            status="unknown",
            message="状态源未接入",
        ),
        SubsystemStatus(
            name="Chassis",
            connected=False,
            status="unknown",
            message="状态源未接入",
        ),
        SubsystemStatus(
            name="PLC",
            connected=False,
            status="unknown",
            message="状态源未接入",
        ),
    ]

    return [s.model_dump() for s in subsystems]


def _read_temperature_zones() -> List[Tuple[str, float]]:
    zones = []
    for zone_path in glob.glob("/sys/class/thermal/thermal_zone*/"):
        type_path = os.path.join(zone_path, "type")
        temp_path = os.path.join(zone_path, "temp")
        try:
            with open(type_path, "r", encoding="utf-8") as f:
                zone_type = f.read().strip().lower()
            with open(temp_path, "r", encoding="utf-8") as f:
                raw = f.read().strip()
            if not raw:
                continue
            temp_c = float(raw) / 1000.0
            zones.append((zone_type, temp_c))
        except Exception:
            continue
    return zones


def _get_system_state() -> SystemState:
    cpu_temp = 0.0
    gpu_temp = 0.0
    battery = 100.0
    uptime_seconds = 0

    zones = _read_temperature_zones()
    cpu_candidates = [
        t for name, t in zones if "cpu" in name or "x86_pkg_temp" in name
    ]
    gpu_candidates = [t for name, t in zones if "gpu" in name]

    if cpu_candidates:
        cpu_temp = max(cpu_candidates)
    elif zones:
        cpu_temp = max(t for _, t in zones)

    if gpu_candidates:
        gpu_temp = max(gpu_candidates)

    try:
        with open("/proc/uptime", "r", encoding="utf-8") as f:
            uptime_seconds = int(float(f.read().split()[0]))
    except Exception:
        uptime_seconds = int(time.monotonic())

    try:
        for path in glob.glob("/sys/class/power_supply/*/capacity"):
            with open(path, "r", encoding="utf-8") as f:
                battery = float(f.read().strip())
                break
    except Exception:
        battery = 100.0

    return SystemState(
        cpu_temp=cpu_temp,
        gpu_temp=gpu_temp,
        battery=battery,
        mode=mode_manager.current_mode.value,
        uptime_seconds=uptime_seconds,
    )


def _load_urdf() -> Optional[str]:
    """加载 URDF 文件"""
    urdf_file = Path(URDF_PATH)
    
    if urdf_file.exists():
        try:
            return urdf_file.read_text(encoding='utf-8')
        except Exception as e:
            logger.error(f"Failed to read URDF: {e}")
            return None
    
    # 返回占位 URDF
    return """<?xml version="1.0"?>
<robot name="qyh_jushen">
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.5 0.5 0.1"/>
      </geometry>
    </visual>
  </link>
  <!-- TODO: 完整 URDF 模型 -->
</robot>"""


# ============================================================================
# API 端点
# ============================================================================

@router.get("/info", response_model=ApiResponse, summary="获取机器人信息")
async def get_robot_info(
    current_user: User = Depends(get_current_user),
):
    """获取机器人基本信息（机型、配置等）"""
    info = RobotInfo(
        name="QYH Jushen",
        model=ROBOT_NAME,
        version=ROBOT_VERSION,
        serial_number=os.environ.get('ROBOT_SERIAL', 'QYH-001'),
        has_left_arm=True,
        has_right_arm=True,
        has_head=True,
        has_lift=True,
        has_waist=True,
        has_chassis=True,
        left_arm_joints=7,
        right_arm_joints=7,
        head_joints=2,
        urdf_path=URDF_PATH,
    )
    
    return success_response(
        data=info.model_dump(),
        message="获取机器人信息成功"
    )


@router.get("/overview", response_model=ApiResponse, summary="获取状态概览")
async def get_robot_overview(
    current_user: User = Depends(get_current_user),
):
    """
    获取机器人状态概览
    
    包含各子系统连接状态，用于仪表盘展示
    注意：这是低频查询，高频状态请使用 Data Plane WebSocket
    """
    overview = RobotOverview(
        timestamp=datetime.now().isoformat(),
        name="QYH Jushen",
        model=ROBOT_NAME,
        version=ROBOT_VERSION,
        subsystems=await _get_subsystem_status(),
        system=_get_system_state().model_dump(),
    )
    
    # 尝试从 ROS2 获取实际状态
    state = ros2_bridge.get_robot_state()
    if state:
        await ros2_bridge.apply_chassis_config_if_needed(
            chassis_connected=bool(state.get("chassis"))
        )
        if state.get("left_arm"):
            overview.left_arm = state["left_arm"]
        if state.get("right_arm"):
            overview.right_arm = state["right_arm"]
        if state.get("head"):
            overview.head = state["head"]
        if state.get("lift"):
            overview.lift = state["lift"]
        if state.get("waist"):
            overview.waist = state["waist"]
        if state.get("chassis"):
            overview.chassis = BaseState(**state["chassis"])
        if state.get("left_gripper"):
            overview.left_gripper = GripperState(**state["left_gripper"])
        if state.get("right_gripper"):
            overview.right_gripper = GripperState(**state["right_gripper"])
        if state.get("battery") is not None and overview.system:
            overview.system["battery"] = float(state["battery"])
    
    return success_response(
        data=overview.model_dump(),
        message="获取状态概览成功"
    )


@router.get("/urdf", summary="获取 URDF 模型")
async def get_robot_urdf(
    current_user: User = Depends(get_current_user),
):
    """
    获取机器人 URDF 模型
    
    返回 XML 格式的 URDF 文件内容
    """
    urdf_content = _load_urdf()
    
    if urdf_content:
        return Response(
            content=urdf_content,
            media_type="application/xml",
            headers={"Content-Disposition": "inline; filename=robot.urdf"}
        )
    else:
        return Response(
            content="URDF not found",
            status_code=404,
            media_type="text/plain"
        )


# ============================================================================
# 系统关机 API
# ============================================================================

@router.get("/shutdown/state", response_model=ApiResponse, summary="获取关机状态")
async def get_shutdown_state(
    current_user: User = Depends(get_current_user),
):
    """
    获取关机状态
    
    用于前端轮询显示关机进度
    """
    state = ros2_bridge.get_shutdown_state()
    
    source_text = {
        0: "",
        1: "硬件按钮触发",
        2: "软件命令触发"
    }.get(state.get("trigger_source", 0), "")
    
    shutdown_state = ShutdownState(
        shutdown_in_progress=state.get("shutdown_in_progress", False),
        trigger_source=state.get("trigger_source", 0),
        trigger_source_text=source_text,
        countdown_seconds=state.get("countdown_seconds", -1),
        plc_connected=state.get("plc_connected", False),
    )
    
    return success_response(
        data=shutdown_state.model_dump(),
        message="获取关机状态成功"
    )


@router.post("/shutdown", response_model=ApiResponse, summary="系统关机")
async def system_shutdown(
    request: ShutdownRequest = None,
    current_user: User = Depends(get_current_admin),
):
    """
    系统关机（需要管理员权限）
    
    软件关机流程:
    1. Web 前端调用此接口
    2. 通过 ROS2 调用 qyh_shutdown 服务
    3. 关机节点写 PLC 关机线圈
    4. 系统执行 shutdown 命令
    5. PLC 检测到系统关机后断电
    """
    if request is None:
        request = ShutdownRequest()
    
    try:
        result = await ros2_bridge.call_shutdown(
            reason=request.reason,
            delay=request.delay_seconds
        )
        
        if result.get("success"):
            return success_response(
                data={
                    "shutdown_initiated": True,
                    "reason": request.reason,
                    "delay_seconds": request.delay_seconds,
                },
                message=result.get("message", "系统关机命令已发送")
            )
        else:
            return error_response(
                code=ErrorCodes.INTERNAL_ERROR,
                message=result.get("message", "关机命令发送失败")
            )
            
    except Exception as e:
        logger.error(f"Shutdown failed: {e}")
        return error_response(
            code=ErrorCodes.INTERNAL_ERROR,
            message=f"系统关机失败: {str(e)}"
        )


@router.post("/reboot", response_model=ApiResponse, summary="系统重启")
async def system_reboot(
    current_user: User = Depends(get_current_admin),
):
    """
    系统重启（需要管理员权限）
    """
    try:
        result = await ros2_bridge.call_reboot()
        
        if result.get("success"):
            return success_response(
                data={"reboot_initiated": True},
                message=result.get("message", "系统重启命令已发送")
            )
        else:
            return error_response(
                code=ErrorCodes.INTERNAL_ERROR,
                message=result.get("message", "重启命令发送失败")
            )
    except Exception as e:
        logger.error(f"Reboot failed: {e}")
        return error_response(
            code=ErrorCodes.INTERNAL_ERROR,
            message=f"系统重启失败: {str(e)}"
        )
