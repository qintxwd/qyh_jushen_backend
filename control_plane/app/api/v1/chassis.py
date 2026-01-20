"""
QYH Jushen Control Plane - 底盘配置 API

提供底盘配置的持久化管理（速度级别、音量、避障策略等）

注意：实时底盘控制（速度命令、急停等）必须通过 Data Plane WebSocket 进行，
    本模块仅提供低频配置与状态查询。
"""
import json
from pathlib import Path
from typing import Optional

from fastapi import APIRouter, Depends
from pydantic import BaseModel, Field

from app.dependencies import get_current_operator, get_current_user
from app.models.user import User
from app.schemas.response import (
    ApiResponse,
    success_response,
    error_response,
    ErrorCodes,
)
from app.services.ros2_client import get_ros2_client

router = APIRouter()


# ==================== 配置文件管理 ====================

def _get_chassis_config_file() -> Path:
    """获取底盘配置文件路径"""
    # 配置文件位于 workspace_root/persistent/web/chassis_config.json
    import os
    workspace_root = Path(
        os.environ.get(
            'QYH_WORKSPACE_ROOT',
            Path.home() / 'qyh-robot-system',
        )
    )
    config_dir = workspace_root / "persistent" / "web"
    config_dir.mkdir(parents=True, exist_ok=True)
    return config_dir / "chassis_config.json"


def _load_chassis_config() -> dict:
    """加载底盘配置"""
    config_file = _get_chassis_config_file()
    if config_file.exists():
        try:
            with open(config_file, 'r', encoding='utf-8') as f:
                return json.load(f)
        except Exception:
            pass
    
    # 默认配置
    return {
        "speed_level": 50,
        "volume": 50,
        "obstacle_strategy": 1,
    }


def _save_chassis_config(config: dict):
    """保存底盘配置"""
    config_file = _get_chassis_config_file()
    with open(config_file, 'w', encoding='utf-8') as f:
        json.dump(config, f, indent=2, ensure_ascii=False)


# ==================== 数据模型 ====================

class ChassisConfig(BaseModel):
    """底盘配置"""
    speed_level: int = Field(
        default=50,
        ge=1,
        le=100,
        description="速度级别 (1-100)"
    )
    volume: int = Field(
        default=50,
        ge=0,
        le=100,
        description="音量 (0-100)"
    )
    obstacle_strategy: int = Field(
        default=1,
        ge=0,
        le=2,
        description="避障策略: 0=禁用, 1=正常, 2=激进"
    )


class ChassisConfigUpdate(BaseModel):
    """底盘配置更新（允许部分更新）"""
    speed_level: Optional[int] = Field(
        default=None,
        ge=1,
        le=100,
        description="速度级别 (1-100)"
    )
    volume: Optional[int] = Field(
        default=None,
        ge=0,
        le=100,
        description="音量 (0-100)"
    )
    obstacle_strategy: Optional[int] = Field(
        default=None,
        ge=0,
        le=2,
        description="避障策略: 0=禁用, 1=正常, 2=激进"
    )


# ==================== API 端点 ====================

@router.get("/config", response_model=ApiResponse)
async def get_chassis_config(
    current_user: User = Depends(get_current_user),
):
    """
    获取底盘配置
    
    返回当前的速度级别、音量、避障策略等配置
    """
    config_dict = _load_chassis_config()
    config = ChassisConfig(**config_dict)
    
    return success_response(
        data=config.model_dump(),
        message="获取底盘配置成功"
    )


@router.put("/config", response_model=ApiResponse)
async def update_chassis_config(
    update: ChassisConfigUpdate,
    current_user: User = Depends(get_current_operator),
):
    """
    更新底盘配置
    
    支持部分更新，只需提供要修改的字段。
    需要操作员权限。
    """
    # 加载现有配置
    config_dict = _load_chassis_config()
    
    # 更新字段
    if update.speed_level is not None:
        config_dict["speed_level"] = update.speed_level
    if update.volume is not None:
        config_dict["volume"] = update.volume
    if update.obstacle_strategy is not None:
        config_dict["obstacle_strategy"] = update.obstacle_strategy
    
    # 保存配置
    try:
        _save_chassis_config(config_dict)
    except Exception as e:
        return error_response(
            code=ErrorCodes.INTERNAL_ERROR,
            message=f"保存配置失败: {str(e)}"
        )
    
    # TODO: 通知 Data Plane 配置已更新
    # 可以通过 ROS2 参数服务器或专用话题通知
    
    config = ChassisConfig(**config_dict)
    return success_response(
        data=config.model_dump(),
        message="底盘配置已更新"
    )


@router.post("/config/reset", response_model=ApiResponse)
async def reset_chassis_config(
    current_user: User = Depends(get_current_operator),
):
    """
    重置底盘配置为默认值
    
    需要操作员权限。
    """
    default_config = {
        "speed_level": 50,
        "volume": 50,
        "obstacle_strategy": 1,
    }
    
    try:
        _save_chassis_config(default_config)
    except Exception as e:
        return error_response(
            code=ErrorCodes.INTERNAL_ERROR,
            message=f"重置配置失败: {str(e)}"
        )
    
    config = ChassisConfig(**default_config)
    return success_response(
        data=config.model_dump(),
        message="底盘配置已重置为默认值"
    )


# ==================== 底盘状态 API ====================
#
# 重要设计说明：
# - 状态获取：从 ROS2 订阅缓存读取真实数据
# - 实时控制：必须通过 Data Plane WebSocket
# - 配置管理：持久化存储 + ROS2 服务同步


@router.get("/status", response_model=ApiResponse)
async def get_chassis_status(
    current_user: User = Depends(get_current_user),
):
    """
    获取底盘状态快照 (HTTP 接口)
    
    ⚠️ 使用场景：
    - 页面初始化时获取一次完整状态
    - WebSocket 未连接时的后备方案
    
    ❌ 不要用于高频轮询！请使用 Data Plane WebSocket 订阅 chassis_state
    
    数据来源：ROS2 话题订阅缓存
    """
    ros2_client = get_ros2_client()
    
    # 从 ROS2 缓存获取状态
    status_data = ros2_client.get_standard_robot_status()
    odom_data = ros2_client.get_odom()
    
    if not status_data and not odom_data:
        return success_response(
            data={
                "connected": False,
                "system_status": 0,
                "system_status_text": "未连接",
                "message": "ROS2 底盘状态未接收到数据，请确认底盘节点已启动"
            },
            message="底盘未连接"
        )
    
    # 构建响应数据
    response_data = {
        "connected": True,
        "system_status": (
            status_data.get("system_status") if status_data else None
        ),
        "system_status_text": _get_system_status_text(
            status_data.get("system_status") if status_data else None
        ),
        "location_status": (
            status_data.get("location_status") if status_data else None
        ),
        "location_status_text": _get_location_status_text(
            status_data.get("location_status") if status_data else None
        ),
        "operation_status": (
            status_data.get("operation_status") if status_data else None
        ),
        "operation_status_text": _get_operation_status_text(
            status_data.get("operation_status") if status_data else None
        ),
    }
    
    # 位姿信息
    if odom_data and odom_data.get("pose"):
        pose = odom_data["pose"]
        pos = (
            pose.position if hasattr(pose, 'position')
            else pose.get("position", {})
        )
        ori = (
            pose.orientation if hasattr(pose, 'orientation')
            else pose.get("orientation", {})
        )
        
        # 提取位置
        x = getattr(pos, 'x', pos.get('x', 0)) if pos else 0
        y = getattr(pos, 'y', pos.get('y', 0)) if pos else 0
        
        # 四元数转航向角
        qz = getattr(ori, 'z', ori.get('z', 0)) if ori else 0
        qw = getattr(ori, 'w', ori.get('w', 1)) if ori else 1
        import math
        yaw = math.atan2(2.0 * qw * qz, 1.0 - 2.0 * qz * qz)
        
        response_data["pose"] = {
            "x": x,
            "y": y,
            "yaw": yaw,
            "confidence": (
                status_data.get("location_confidence")
                if status_data else None
            ),
        }
    
    # 速度信息
    if odom_data and odom_data.get("twist"):
        twist = odom_data["twist"]
        linear = (
            twist.linear if hasattr(twist, 'linear')
            else twist.get("linear", {})
        )
        angular = (
            twist.angular if hasattr(twist, 'angular')
            else twist.get("angular", {})
        )
        
        response_data["velocity"] = {
            "linear_x": (
                getattr(linear, 'x', linear.get('x', 0))
                if linear else 0
            ),
            "linear_y": (
                getattr(linear, 'y', linear.get('y', 0))
                if linear else 0
            ),
            "angular_z": (
                getattr(angular, 'z', angular.get('z', 0))
                if angular else 0
            ),
        }
    
    # 电池信息
    if status_data:
        is_charging = status_data.get("is_charging")
        response_data["battery"] = {
            "percentage": status_data.get("battery_remaining_percentage"),
            "voltage": status_data.get("battery_voltage"),
            "current": status_data.get("battery_current"),
            "status_text": (
                "充电中" if is_charging is True
                else "放电中" if is_charging is False
                else "未知"
            ),
        }
        
        # 标志位
        is_emergency_stopped = status_data.get("is_emergency_stopped")
        operation_status = status_data.get("operation_status")
        response_data["flags"] = {
            "is_emergency_stopped": is_emergency_stopped,
            "is_emergency_recoverable": (
                status_data.get("is_emergency_recoverable")
            ),
            "is_brake_released": (
                (not is_emergency_stopped)
                if is_emergency_stopped is not None else None
            ),
            "is_charging": is_charging,
            "is_low_power_mode": status_data.get("is_low_power_mode"),
            "obstacle_slowdown": status_data.get("obstacle_slowdown"),
            "obstacle_paused": status_data.get("obstacle_paused"),
            "can_run_motion_task": status_data.get("can_run_motion_task"),
            "is_auto_mode": (
                (operation_status == 1)
                if operation_status is not None else None
            ),
            "is_loaded": status_data.get("is_loaded"),
            "has_wifi": status_data.get("has_wifi"),
        }
    
    return success_response(
        data=response_data,
        message="获取状态成功"
    )


def _get_system_status_text(status: int | None) -> str:
    """系统状态码转文本"""
    status_map = {
        0: "未知",
        1: "正常",
        2: "警告",
        3: "错误",
    }
    return status_map.get(status, "未知")


def _get_location_status_text(status: int | None) -> str:
    """定位状态码转文本"""
    status_map = {
        0: "未定位",
        1: "定位中",
        2: "已定位",
    }
    return status_map.get(status, "未知")


def _get_operation_status_text(status: int | None) -> str:
    """运行状态码转文本"""
    status_map = {
        0: "空闲",
        1: "自动",
        2: "手动",
        3: "急停",
    }
    return status_map.get(status, "未知")


@router.get("/stations", response_model=ApiResponse)
async def get_stations(
    current_user: User = Depends(get_current_user),
):
    """
    获取站点列表
    
    从地图配置文件读取站点数据
    """
    import os
    
    workspace_root = Path(
        os.environ.get(
            'QYH_WORKSPACE_ROOT',
            Path.home() / 'qyh-robot-system',
        )
    )
    current_map_file = workspace_root / "maps" / "current_map.txt"
    
    current_map = "standard"
    if current_map_file.exists():
        try:
            current_map = current_map_file.read_text().strip()
        except Exception:
            pass
    
    map_json = workspace_root / "maps" / current_map / f"{current_map}.json"
    stations = []
    
    if map_json.exists():
        try:
            with open(map_json, 'r', encoding='utf-8') as f:
                map_data = json.load(f)
                raw_stations = map_data.get("stations", [])
                for s in raw_stations:
                    stations.append({
                        "id": s.get("id", 0),
                        "name": s.get("name", ""),
                        "x": s.get("pos.x", s.get("x", 0)),
                        "y": s.get("pos.y", s.get("y", 0)),
                        "yaw": s.get("pos.yaw", s.get("yaw", 0)),
                    })
        except Exception as e:
            return error_response(
                code=ErrorCodes.INTERNAL_ERROR,
                message=f"读取站点数据失败: {str(e)}"
            )
    
    return success_response(
        data={"stations": stations},
        message="获取站点成功"
    )


# ==================== 注意 ====================
#
# 底盘实时控制接口（速度命令、急停、导航）已移至 Data Plane WebSocket
# 参见: data_plane/README.md
#
# WebSocket 消息类型:
#   - CHASSIS_VELOCITY: 发送速度命令
#   - EMERGENCY_STOP: 紧急停止
#   - NAVIGATE_TO_POSE: 导航到坐标
#   - NAVIGATE_TO_STATION: 导航到站点
#
# 此设计是为了保证实时控制的低延迟和高可靠性

