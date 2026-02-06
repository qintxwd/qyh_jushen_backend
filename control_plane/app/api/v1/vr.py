"""VR 遥操作状态 API (简化版)

VR 采用专用通道设计:
- VR 通过 Data Plane 的 /vr 端点连接 (单连接限制)
- Control Plane 只负责查询 VR 连接状态
- Web/Mobile 通过订阅 Data Plane 的 VRSystemState 获取实时状态

本模块提供:
- GET /status: 查询 VR 连接状态
- POST /internal/connected: Data Plane 上报 VR 连接
- POST /internal/disconnected: Data Plane 上报 VR 断开
"""
import ipaddress
import logging
from datetime import datetime
from typing import Optional
from fastapi import APIRouter, Depends, Request
from pydantic import BaseModel, Field

from app.dependencies import get_current_user
from app.models.user import User
from app.schemas.response import ApiResponse, success_response, error_response, ErrorCodes
from app.config import settings

router = APIRouter()
logger = logging.getLogger(__name__)


def _is_loopback_address(host: str) -> bool:
    try:
        return ipaddress.ip_address(host).is_loopback
    except ValueError:
        return False


# ==================== 数据模型 ====================

class VRClientInfo(BaseModel):
    """VR 客户端信息"""
    device: str = Field(default="", description="设备型号")
    version: str = Field(default="", description="客户端版本")
    session_id: str = Field(default="", description="会话ID")
    connected_at: Optional[datetime] = Field(
        default=None, description="连接时间"
    )


class VRConnectionStatus(BaseModel):
    """VR 连接状态"""
    connected: bool = Field(default=False, description="是否已连接")
    client_info: Optional[VRClientInfo] = Field(
        default=None, description="客户端信息"
    )


class VRConnectedRequest(BaseModel):
    """VR 连接上报请求 (Data Plane → Control Plane)"""
    device: str = Field(default="PICO 4", description="设备型号")
    version: str = Field(default="1.0.0", description="客户端版本")
    session_id: str = Field(..., description="WebSocket 会话ID")


class VRDisconnectedRequest(BaseModel):
    """VR 断开上报请求 (Data Plane → Control Plane)"""
    session_id: str = Field(..., description="WebSocket 会话ID")
    reason: str = Field(
        default="client_close",
        description="断开原因: client_close, heartbeat_timeout, error"
    )


# ==================== VR 状态存储 (内存) ====================

class VRStateStore:
    """VR 状态存储 (单例)"""

    _instance: Optional['VRStateStore'] = None

    def __new__(cls):
        if cls._instance is None:
            cls._instance = super().__new__(cls)
            cls._instance._connected = False
            cls._instance._client_info = None
        return cls._instance

    @property
    def connected(self) -> bool:
        return self._connected

    @property
    def client_info(self) -> Optional[VRClientInfo]:
        return self._client_info

    def set_connected(self, info: VRClientInfo):
        self._connected = True
        self._client_info = info
        logger.info(f"VR connected: {info.device} v{info.version}")

    def set_disconnected(self, reason: str):
        old_info = self._client_info
        self._connected = False
        self._client_info = None
        if old_info:
            logger.info(
                f"VR disconnected: {old_info.device}, reason={reason}"
            )


# 全局单例
vr_state = VRStateStore()


# ==================== API 端点 ====================

@router.get("/status", response_model=ApiResponse)
async def get_vr_status(
    current_user: User = Depends(get_current_user),
) -> ApiResponse:
    """查询 VR 连接状态

    返回:
    - connected: VR 是否已连接到 Data Plane
    - client_info: VR 客户端信息 (设备型号、版本、连接时间)

    注意: 实时 VR 位姿/按钮状态请订阅 Data Plane 的 VRSystemState
    """
    status = VRConnectionStatus(
        connected=vr_state.connected,
        client_info=vr_state.client_info
    )
    return success_response(data=status.model_dump())


# ==================== 内部接口 (Data Plane 调用) ====================

@router.post("/internal/connected", response_model=ApiResponse)
async def vr_connected(
    request: VRConnectedRequest,
    http_request: Request,
) -> ApiResponse:
    """VR 连接上报 (内部接口)

    由 Data Plane 在 VR 客户端连接时调用

    注意: 此接口应仅允许内部调用，生产环境需添加鉴权
    """
    client_ip = http_request.client.host if http_request.client else "unknown"
    if not _is_loopback_address(client_ip):
        return error_response(
            code=ErrorCodes.PERMISSION_DENIED,
            message="Loopback access required",
        )

    if settings.VR_INTERNAL_TOKEN:
        auth_header = http_request.headers.get("Authorization", "")
        token = auth_header.removeprefix("Bearer ").strip()
        if token != settings.VR_INTERNAL_TOKEN:
            return error_response(
                code=ErrorCodes.PERMISSION_DENIED,
                message="Unauthorized",
            )
    logger.info(f"VR connected notification from {client_ip}")

    info = VRClientInfo(
        device=request.device,
        version=request.version,
        session_id=request.session_id,
        connected_at=datetime.now()
    )
    vr_state.set_connected(info)

    return success_response(message="VR connection recorded")


@router.post("/internal/disconnected", response_model=ApiResponse)
async def vr_disconnected(
    request: VRDisconnectedRequest,
    http_request: Request,
) -> ApiResponse:
    """VR 断开上报 (内部接口)

    由 Data Plane 在 VR 客户端断开时调用

    断开原因:
    - client_close: 客户端主动断开
    - heartbeat_timeout: 心跳超时 (Watchdog 触发)
    - error: 连接错误
    """
    client_ip = http_request.client.host if http_request.client else "unknown"
    if not _is_loopback_address(client_ip):
        return error_response(
            code=ErrorCodes.PERMISSION_DENIED,
            message="Loopback access required",
        )

    if settings.VR_INTERNAL_TOKEN:
        auth_header = http_request.headers.get("Authorization", "")
        token = auth_header.removeprefix("Bearer ").strip()
        if token != settings.VR_INTERNAL_TOKEN:
            return error_response(
                code=ErrorCodes.PERMISSION_DENIED,
                message="Unauthorized",
            )
    logger.info(f"VR disconnected notification from {client_ip}")

    vr_state.set_disconnected(request.reason)

    return success_response(message="VR disconnection recorded")

