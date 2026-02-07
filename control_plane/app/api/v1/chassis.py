"""
QYH Jushen Control Plane - 底盘配置 API

提供底盘配置的持久化管理（速度级别、音量、避障策略等）

注意：实时底盘控制（速度命令、急停等）应通过 Data Plane WebSocket 进行，
      以获得最低延迟。本模块的控制接口仅作为 HTTP 后备方案。
"""
import json
from pathlib import Path
from typing import Optional

from fastapi import APIRouter, Depends, HTTPException
from pydantic import BaseModel, Field

from app.dependencies import get_current_operator, get_current_user
from app.models.user import User
from app.schemas.response import ApiResponse, success_response, error_response, ErrorCodes
from app.services.ros2_client import get_ros2_client

router = APIRouter()


# ==================== 配置文件管理 ====================

def _get_chassis_config_file() -> Path:
    """获取底盘配置文件路径"""
    # 配置文件位于 workspace_root/persistent/web/chassis_config.json
    import os
    workspace_root = Path(os.environ.get('QYH_WORKSPACE_ROOT', Path.home() / 'qyh-robot-system'))
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


class ManualVelocityRequest(BaseModel):
    """手动速度命令请求"""
    linear: float = Field(default=0.0, description="线速度 (m/s)")
    angular: float = Field(default=0.0, description="角速度 (rad/s)")


class ControlModeRequest(BaseModel):
    """控制模式切换请求"""
    mode: int = Field(..., description="0=手动, 1=自动")


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


# ==================== 底盘状态与控制 API ====================
# 
# 重要设计说明：
# - 状态获取：从 ROS2 订阅缓存读取真实数据
# - 实时控制：应优先通过 Data Plane WebSocket，HTTP 仅作后备
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
        "system_status": status_data.get("system_status", 0) if status_data else 0,
        "system_status_text": _get_system_status_text(
            status_data.get("system_status", 0) if status_data else 0
        ),
        "location_status": status_data.get("location_status", 0) if status_data else 0,
        "location_status_text": _get_location_status_text(
            status_data.get("location_status", 0) if status_data else 0
        ),
        "operation_status": status_data.get("operation_status", 0) if status_data else 0,
        "operation_status_text": _get_operation_status_text(
            status_data.get("operation_status", 0) if status_data else 0
        ),
    }
    
    # 位姿信息
    if odom_data and odom_data.get("pose"):
        pose = odom_data["pose"]
        pos = pose.position if hasattr(pose, 'position') else pose.get("position", {})
        ori = pose.orientation if hasattr(pose, 'orientation') else pose.get("orientation", {})
        
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
            "confidence": 1.0  # TODO: 从定位状态获取置信度
        }
    
    # 速度信息
    if odom_data and odom_data.get("twist"):
        twist = odom_data["twist"]
        linear = twist.linear if hasattr(twist, 'linear') else twist.get("linear", {})
        angular = twist.angular if hasattr(twist, 'angular') else twist.get("angular", {})
        
        response_data["velocity"] = {
            "linear_x": getattr(linear, 'x', linear.get('x', 0)) if linear else 0,
            "linear_y": getattr(linear, 'y', linear.get('y', 0)) if linear else 0,
            "angular_z": getattr(angular, 'z', angular.get('z', 0)) if angular else 0,
        }
    
    # 电池信息
    if status_data:
        response_data["battery"] = {
            "percentage": status_data.get("battery_remaining_percentage", 0),
            "voltage": 0,  # TODO: 需要从其他话题获取
            "current": 0,
            "status_text": "充电中" if status_data.get("is_charging") else "放电中",
        }
        
        # 标志位
        response_data["flags"] = {
            "is_emergency_stopped": status_data.get("is_emergency_stopped", False),
            "is_emergency_recoverable": True,  # TODO: 从硬件状态获取
            "is_brake_released": not status_data.get("is_emergency_stopped", False),
            "is_charging": status_data.get("is_charging", False),
            "is_low_power_mode": False,
            "obstacle_slowdown": False,
            "obstacle_paused": False,
            "can_run_motion_task": True,
            "is_auto_mode": status_data.get("operation_status", 0) == 1,
            "is_loaded": False,
            "has_wifi": True,
        }
    
    return success_response(
        data=response_data,
        message="获取状态成功"
    )


def _get_system_status_text(status: int) -> str:
    """系统状态码转文本"""
    status_map = {
        0: "未知",
        1: "正常",
        2: "警告",
        3: "错误",
    }
    return status_map.get(status, "未知")


def _get_location_status_text(status: int) -> str:
    """定位状态码转文本"""
    status_map = {
        0: "未定位",
        1: "定位中",
        2: "已定位",
    }
    return status_map.get(status, "未知")


def _get_operation_status_text(status: int) -> str:
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
    
    workspace_root = Path(os.environ.get('QYH_WORKSPACE_ROOT', Path.home() / 'qyh-robot-system'))
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
                raw_stations = map_data.get("data", {}).get("station", [])
                if not raw_stations:
                    raw_stations = map_data.get("stations", [])
                for s in raw_stations:
                    x_mm = s.get("pos.x", s.get("x", 0))
                    y_mm = s.get("pos.y", s.get("y", 0))
                    yaw_mrad = s.get("pos.yaw", s.get("yaw", 0))
                    stations.append({
                        "id": s.get("id", 0),
                        "name": s.get("name", ""),
                        "x": x_mm / 1000.0,
                        "y": y_mm / 1000.0,
                        "yaw": yaw_mrad / 1000.0,
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


# ==================== 底盘控制 API (HTTP 后备) ====================

@router.post("/manual_velocity", response_model=ApiResponse)
async def manual_velocity(
    request: ManualVelocityRequest,
    current_user: User = Depends(get_current_operator),
):
    """手动速度控制 (HTTP 后备，推荐使用 WebSocket)"""
    ros2_client = get_ros2_client()
    if not ros2_client._initialized:
        await ros2_client.initialize()

    try:
        await ros2_client.publish_manual_velocity(request.linear, request.angular)
        return success_response(
            data={"sent": True},
            message="速度指令已发送"
        )
    except Exception as e:
        return error_response(
            code=ErrorCodes.ROS2_ERROR,
            message=f"发送速度指令失败: {str(e)}"
        )


@router.post("/control_mode", response_model=ApiResponse)
async def set_control_mode(
    request: ControlModeRequest,
    current_user: User = Depends(get_current_operator),
):
    """切换手动/自动控制模式"""
    ros2_client = get_ros2_client()
    if not ros2_client._initialized:
        await ros2_client.initialize()

    if request.mode not in (0, 1):
        return error_response(
            code=ErrorCodes.INVALID_PARAMS,
            message="mode 仅支持 0(手动) 或 1(自动)"
        )

    result = await ros2_client.set_manual_control(request.mode == 0)
    if result.success:
        return success_response(
            data={"mode": request.mode},
            message=result.message or "模式切换成功"
        )
    return error_response(
        code=ErrorCodes.OPERATION_FAILED,
        message=result.message or "模式切换失败"
    )


@router.post("/system_reset", response_model=ApiResponse)
async def system_reset(
    current_user: User = Depends(get_current_operator),
):
    """系统复位"""
    ros2_client = get_ros2_client()
    if not ros2_client._initialized:
        await ros2_client.initialize()

    result = await ros2_client.system_reset()
    if result.success:
        return success_response(
            data={"reset": True},
            message=result.message or "系统复位成功"
        )
    return error_response(
        code=ErrorCodes.OPERATION_FAILED,
        message=result.message or "系统复位失败"
    )


@router.post("/stop_localization", response_model=ApiResponse)
async def stop_localization(
    current_user: User = Depends(get_current_operator),
):
    """停止定位"""
    ros2_client = get_ros2_client()
    if not ros2_client._initialized:
        await ros2_client.initialize()

    result = await ros2_client.stop_localization()
    if result.success:
        return success_response(
            data={"stopped": True},
            message=result.message or "定位已停止"
        )
    return error_response(
        code=ErrorCodes.OPERATION_FAILED,
        message=result.message or "停止定位失败"
    )


@router.post("/start_charging", response_model=ApiResponse)
async def start_charging(
    current_user: User = Depends(get_current_operator),
):
    """开始充电"""
    ros2_client = get_ros2_client()
    if not ros2_client._initialized:
        await ros2_client.initialize()

    result = await ros2_client.start_charging()
    if result.success:
        return success_response(
            data={"charging": True},
            message=result.message or "开始充电"
        )
    return error_response(
        code=ErrorCodes.OPERATION_FAILED,
        message=result.message or "开始充电失败"
    )


@router.post("/stop_charging", response_model=ApiResponse)
async def stop_charging(
    current_user: User = Depends(get_current_operator),
):
    """停止充电"""
    ros2_client = get_ros2_client()
    if not ros2_client._initialized:
        await ros2_client.initialize()

    result = await ros2_client.stop_charging()
    if result.success:
        return success_response(
            data={"charging": False},
            message=result.message or "停止充电"
        )
    return error_response(
        code=ErrorCodes.OPERATION_FAILED,
        message=result.message or "停止充电失败"
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


# ==================== 导航控制 API ====================
# 
# HTTP 导航接口用于发起导航任务，适合非实时场景。
# 取消/暂停等频繁操作应通过 WebSocket 进行。


class NavigateToPoseRequest(BaseModel):
    """导航到坐标点请求"""
    x: float = Field(..., description="目标 X 坐标 (米)")
    y: float = Field(..., description="目标 Y 坐标 (米)")
    yaw: float = Field(default=0.0, description="目标航向角 (弧度)")
    speed_factor: float = Field(
        default=1.0,
        ge=0.1,
        le=1.0,
        description="速度因子 (0.1-1.0)"
    )


class NavigateToStationRequest(BaseModel):
    """导航到站点请求"""
    station_id: Optional[int] = Field(default=None, description="站点 ID")
    station_name: Optional[str] = Field(default=None, description="站点名称")
    speed_factor: float = Field(
        default=1.0,
        ge=0.1,
        le=1.0,
        description="速度因子 (0.1-1.0)"
    )





@router.post("/navigate/pose", response_model=ApiResponse)
async def navigate_to_pose(
    request: NavigateToPoseRequest,
    current_user: User = Depends(get_current_operator),
):
    """
    发起导航到坐标点
    
    需要操作员权限。
    
    ⚠️ 推荐场景：
    - 从任务系统或上层调度发起的导航
    - 页面上点击地图发起导航
    
    💡 如需取消或暂停导航，请使用 WebSocket 通道发送 MSG_NAVIGATION_CANCEL
    """
    ros2_client = get_ros2_client()
    
    try:
        result = await ros2_client.navigate_to_pose(
            x=request.x,
            y=request.y,
            yaw=request.yaw,
            speed_factor=request.speed_factor
        )
        
        if result.success:
            return success_response(
                data={
                    "task_id": result.task_id,
                    "target": {
                        "x": request.x,
                        "y": request.y,
                        "yaw": request.yaw,
                    },
                },
                message="导航任务已发起"
            )
        else:
            return error_response(
                code=ErrorCodes.OPERATION_FAILED,
                message=f"导航发起失败: {result.message}"
            )
    except Exception as e:
        return error_response(
            code=ErrorCodes.ROS2_ERROR,
            message=f"ROS2 服务调用失败: {str(e)}"
        )


@router.post("/navigate/station", response_model=ApiResponse)
async def navigate_to_station(
    request: NavigateToStationRequest,
    current_user: User = Depends(get_current_operator),
):
    """
    发起导航到站点
    
    需要操作员权限。可通过站点 ID 或名称指定目标。
    
    ⚠️ 推荐场景：
    - 任务系统调度
    - 前端站点列表点击导航
    """
    # 验证参数
    if request.station_id is None and request.station_name is None:
        return error_response(
            code=ErrorCodes.INVALID_PARAMS,
            message="必须提供 station_id 或 station_name"
        )
    
    # 查找站点坐标
    import os
    workspace_root = Path(os.environ.get('QYH_WORKSPACE_ROOT', Path.home() / 'qyh-robot-system'))
    current_map_file = workspace_root / "maps" / "current_map.txt"
    
    current_map = "standard"
    if current_map_file.exists():
        try:
            current_map = current_map_file.read_text().strip()
        except Exception:
            pass
    
    map_json = workspace_root / "maps" / current_map / f"{current_map}.json"
    
    target_station = None
    if map_json.exists():
        try:
            with open(map_json, 'r', encoding='utf-8') as f:
                map_data = json.load(f)
                stations = map_data.get("data", {}).get("station", [])
                if not stations:
                    stations = map_data.get("stations", [])
                for s in stations:
                    if request.station_id is not None and s.get("id") == request.station_id:
                        target_station = s
                        break
                    if request.station_name is not None and s.get("name") == request.station_name:
                        target_station = s
                        break
        except Exception as e:
            return error_response(
                code=ErrorCodes.INTERNAL_ERROR,
                message=f"读取站点数据失败: {str(e)}"
            )
    
    if not target_station:
        return error_response(
            code=ErrorCodes.RESOURCE_NOT_FOUND,
            message="未找到指定站点"
        )
    
    # 提取坐标
    x_mm = target_station.get("pos.x", target_station.get("x", 0))
    y_mm = target_station.get("pos.y", target_station.get("y", 0))
    yaw_mrad = target_station.get("pos.yaw", target_station.get("yaw", 0))
    x = x_mm / 1000.0
    y = y_mm / 1000.0
    yaw = yaw_mrad / 1000.0
    
    ros2_client = get_ros2_client()
    
    try:
        result = await ros2_client.navigate_to_pose(
            x=x,
            y=y,
            yaw=yaw,
            speed_factor=request.speed_factor
        )
        
        if result.success:
            return success_response(
                data={
                    "task_id": result.task_id,
                    "station": {
                        "id": target_station.get("id"),
                        "name": target_station.get("name"),
                    },
                    "target": {"x": x, "y": y, "yaw": yaw},
                },
                message=f"导航到站点 {target_station.get('name', '未命名')} 已发起"
            )
        else:
            return error_response(
                code=ErrorCodes.OPERATION_FAILED,
                message=f"导航发起失败: {result.message}"
            )
    except Exception as e:
        return error_response(
            code=ErrorCodes.ROS2_ERROR,
            message=f"ROS2 服务调用失败: {str(e)}"
        )


@router.post("/navigate/cancel", response_model=ApiResponse)
async def cancel_navigation(
    current_user: User = Depends(get_current_operator),
):
    """
    取消当前导航任务 (HTTP 后备接口)
    
    ⚠️ 推荐使用 WebSocket 通道发送 MSG_NAVIGATION_CANCEL 以获得更低延迟。
    此 HTTP 接口作为后备方案。
    """
    ros2_client = get_ros2_client()
    
    try:
        result = await ros2_client.cancel_navigation()
        
        if result.success:
            return success_response(
                data={"cancelled": True},
                message="导航任务已取消"
            )
        else:
            return error_response(
                code=ErrorCodes.OPERATION_FAILED,
                message=f"取消导航失败: {result.message}"
            )
    except Exception as e:
        return error_response(
            code=ErrorCodes.ROS2_ERROR,
            message=f"ROS2 服务调用失败: {str(e)}"
        )



# ==================== 地图数据 API ====================

@router.get("/map_data", response_model=ApiResponse)
async def get_map_data(
    current_user: User = Depends(get_current_user),
):
    """
    获取当前地图的完整数据，用于3D场景渲染
    返回: meta, nodes, edges, stations, image_url
    """
    import os
    
    workspace_root = Path(os.environ.get('QYH_WORKSPACE_ROOT', Path.home() / 'qyh-robot-system'))
    maps_dir = workspace_root / "maps"
    
    # 读取当前地图名
    current_map_file = maps_dir / "current_map.txt"
    if not current_map_file.exists():
        return error_response(
            code=ErrorCodes.RESOURCE_NOT_FOUND,
            message="未找到当前地图信息，请先同步地图"
        )
    
    current_map = current_map_file.read_text(encoding='utf-8').strip()
    if not current_map:
        return error_response(
            code=ErrorCodes.RESOURCE_NOT_FOUND,
            message="当前地图名称为空"
        )
    
    # 读取地图JSON数据
    map_json_file = maps_dir / current_map / f"{current_map}.json"
    if not map_json_file.exists():
        return error_response(
            code=ErrorCodes.RESOURCE_NOT_FOUND,
            message=f"地图数据文件不存在: {current_map}"
        )
    
    try:
        with open(map_json_file, 'r', encoding='utf-8') as f:
            map_data = json.load(f)
        
        meta = map_data.get('meta', {})
        data = map_data.get('data', {})
        
        # 检查地图图片
        map_image_file = maps_dir / current_map / f"{current_map}.png"
        has_image = map_image_file.exists()
        
        return success_response(
            data={
                "success": True,
                "map_name": current_map,
                "meta": meta,
                "nodes": data.get('node', []),
                "edges": data.get('edge', []),
                "stations": data.get('station', []),
                "has_image": has_image,
                "image_url": f"/api/v1/chassis/map_image/{current_map}" if has_image else None
            },
            message="获取地图数据成功"
        )
    except Exception as e:
        return error_response(
            code=ErrorCodes.INTERNAL_ERROR,
            message=f"读取地图数据失败: {str(e)}"
        )


@router.get("/map_image/{map_name}")
async def get_map_image(map_name: str):
    """获取地图图片 (公开访问，用于前端 Image 元素加载)"""
    from fastapi.responses import FileResponse
    from fastapi import Response
    import os
    
    workspace_root = Path(os.environ.get('QYH_WORKSPACE_ROOT', Path.home() / 'qyh-robot-system'))
    maps_dir = workspace_root / "maps"
    
    # 安全检查：防止路径遍历
    if '..' in map_name or '/' in map_name or '\\' in map_name:
        raise HTTPException(status_code=400, detail="无效的地图名称")
    
    map_image_file = maps_dir / map_name / f"{map_name}.png"
    if not map_image_file.exists():
        # 尝试 jpg
        map_image_file = maps_dir / map_name / f"{map_name}.jpg"
        if not map_image_file.exists():
            raise HTTPException(status_code=404, detail="地图图片不存在")
    
    # 返回图片，添加 CORS 头
    return FileResponse(
        path=str(map_image_file),
        media_type="image/png" if map_image_file.suffix == '.png' else "image/jpeg",
        headers={
            "Access-Control-Allow-Origin": "*",
            "Access-Control-Allow-Methods": "GET",
            "Cache-Control": "public, max-age=3600"
        }
    )


@router.get("/maps", response_model=ApiResponse)
async def get_maps_list(
    current_user: User = Depends(get_current_user),
):
    """获取所有地图列表"""
    import os
    
    workspace_root = Path(os.environ.get('QYH_WORKSPACE_ROOT', Path.home() / 'qyh-robot-system'))
    maps_dir = workspace_root / "maps"
    
    if not maps_dir.exists():
        return success_response(
            data={"maps": [], "current_map": None},
            message="地图目录不存在"
        )
    
    # 读取当前地图
    current_map_file = maps_dir / "current_map.txt"
    current_map = ""
    if current_map_file.exists():
        current_map = current_map_file.read_text(encoding='utf-8').strip()
    
    # 获取地图列表
    map_list = []
    for d in maps_dir.iterdir():
        if d.is_dir():
            map_list.append(d.name)
    
    return success_response(
        data={
            "maps": map_list,
            "current_map": current_map
        },
        message="获取地图列表成功"
    )
