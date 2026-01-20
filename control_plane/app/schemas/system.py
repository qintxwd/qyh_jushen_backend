"""
QYH Jushen Control Plane - 系统 Schema
"""
from typing import Optional, List, Dict, Any
from pydantic import BaseModel, Field


class EndpointsConfig(BaseModel):
    """服务端点配置"""
    websocket: str = Field(..., description="WebSocket 服务地址")
    webrtc_signaling: str = Field(..., description="WebRTC 信令服务地址")


class ICEServer(BaseModel):
    """ICE 服务器配置"""
    urls: str
    username: Optional[str] = None
    credential: Optional[str] = None


class WebRTCConfig(BaseModel):
    """WebRTC 配置"""
    ice_servers: List[ICEServer] = Field(default_factory=list)


class FeaturesConfig(BaseModel):
    """功能特性配置"""
    vr_teleop: bool = True
    multi_camera: bool = True
    recording: bool = True
    navigation: bool = True


class SystemConfig(BaseModel):
    """系统配置（服务发现响应）"""
    robot_id: str = Field(..., description="机器人 ID")
    robot_name: str = Field(..., description="机器人名称")
    endpoints: EndpointsConfig = Field(..., description="服务端点")
    webrtc: WebRTCConfig = Field(..., description="WebRTC 配置")
    features: FeaturesConfig = Field(default_factory=FeaturesConfig)


class ServiceHealth(BaseModel):
    """服务健康状态"""
    name: str
    status: str  # healthy, unhealthy, unknown
    latency_ms: Optional[float] = None
    message: Optional[str] = None


class SystemHealth(BaseModel):
    """系统健康状态"""
    overall_status: str = Field(..., description="healthy, degraded, unhealthy")
    services: List[ServiceHealth] = Field(default_factory=list)
    ros2_connected: bool = False
    robot_mode: str = "unknown"


class SystemInfo(BaseModel):
    """系统信息"""
    app_name: str
    app_version: str
    version: Optional[str] = Field(None, description="Frontend alias for app_version")
    environment: str = Field(default="production", description="Frontend field")
    robot_id: str
    robot_name: str
    uptime_seconds: float
    ros2_domain_id: int
