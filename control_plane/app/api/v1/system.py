"""
QYH Jushen Control Plane - 系统配置 API

提供服务发现、健康检查、系统信息等功能
这是重构补充.md 中必须实现的核心接口
"""
import time
from fastapi import APIRouter

from app.config import settings
from app.core.mode_manager import mode_manager
from app.services.health_checker import get_health_checker, ServiceStatus
from app.schemas.response import ApiResponse, success_response
from app.schemas.system import (
    SystemConfig,
    EndpointsConfig,
    WebRTCConfig,
    ICEServer,
    FeaturesConfig,
    SystemHealth,
    ServiceHealth,
    SystemInfo,
)

router = APIRouter()

# 应用启动时间（用于计算 uptime）
_start_time = time.time()


@router.get("/config", response_model=ApiResponse)
async def get_system_config():
    """
    获取系统配置（服务发现）
    
    这是前端获取其他服务地址的唯一入口。
    前端只硬编码 FastAPI 地址，其他所有服务地址都从此接口动态获取。
    """
    config = SystemConfig(
        robot_id=settings.ROBOT_ID,
        robot_name=settings.ROBOT_NAME,
        endpoints=EndpointsConfig(
            websocket=settings.WEBSOCKET_SERVER_URL,
            webrtc_signaling=settings.WEBRTC_SIGNALING_URL,
        ),
        webrtc=WebRTCConfig(
            ice_servers=[
                ICEServer(urls="stun:stun.l.google.com:19302"),
            ]
        ),
        features=FeaturesConfig(
            vr_teleop=True,
            multi_camera=True,
            recording=True,
            navigation=True,
        ),
    )
    
    return success_response(
        data=config.model_dump(),
        message="获取系统配置成功"
    )


@router.get("/health", response_model=ApiResponse)
async def get_system_health():
    """
    获取系统健康状态
    
    检查各个子服务的健康状态：
    - control_plane: 自身（总是 healthy）
    - data_plane: WebSocket 数据服务 (ws://host:8765)
    - media_plane: WebRTC 媒体服务 (http://host:8888)
    - ros2: ROS2 连接状态
    """
    health_checker = get_health_checker()
    
    # 并行检查所有服务
    check_results = await health_checker.check_all()
    
    services = []
    
    # FastAPI 服务（自身）- 总是 healthy
    services.append(ServiceHealth(
        name="control_plane",
        status="healthy",
        latency_ms=0.1,
        message="Control Plane 运行正常",
    ))
    
    # 添加各服务检查结果
    for name in ["data_plane", "media_plane", "ros2"]:
        result = check_results.get(name)
        if result:
            services.append(ServiceHealth(
                name=result.name,
                status=result.status.value,
                latency_ms=result.latency_ms,
                message=result.message,
            ))
    
    # 判断整体状态
    overall_status = "healthy"
    unhealthy_count = sum(1 for s in services if s.status == "unhealthy")
    unknown_count = sum(1 for s in services if s.status == "unknown")
    degraded_count = sum(1 for s in services if s.status == "degraded")
    
    if unhealthy_count > 0:
        overall_status = "unhealthy"
    elif unknown_count > 0 or degraded_count > 0:
        overall_status = "degraded"
    
    # ROS2 连接状态
    ros2_result = check_results.get("ros2")
    ros2_connected = (
        ros2_result is not None and 
        ros2_result.status == ServiceStatus.HEALTHY
    )
    
    health = SystemHealth(
        status=overall_status,
        services=services,
        ros2_connected=ros2_connected,
        robot_mode=mode_manager.current_mode.value,
    )
    
    return success_response(
        data=health.model_dump(),
        message="获取健康状态成功"
    )


@router.get("/info", response_model=ApiResponse)
async def get_system_info():
    """
    获取系统信息
    """
    uptime = time.time() - _start_time
    
    info = SystemInfo(
        app_name=settings.APP_NAME,
        app_version=settings.APP_VERSION,
        robot_id=settings.ROBOT_ID,
        robot_name=settings.ROBOT_NAME,
        uptime_seconds=uptime,
        ros2_domain_id=settings.ROS_DOMAIN_ID,
    )
    
    return success_response(
        data=info.model_dump(),
        message="获取系统信息成功"
    )
