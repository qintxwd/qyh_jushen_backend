"""
QYH Jushen Control Plane - Schemas 模块
"""
from app.schemas.response import (
    ApiResponse,
    PagedData,
    ErrorCodes,
    success_response,
    error_response,
    paged_response,
)
from app.schemas.auth import (
    LoginRequest,
    TokenResponse,
    UserInfo,
    LoginResponse,
    UserCreate,
    UserUpdate,
    PasswordChange,
)
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
from app.schemas.control import (
    AcquireControlRequest,
    ControlHolder,
    ControlStatus,
    ForceReleaseRequest,
)
from app.schemas.mode import (
    ModeSwitchRequest,
    ModeInfo,
    ModeStatus,
)
from app.schemas.task import (
    TaskAction,
    CreateTaskRequest,
    TaskInfo,
    TaskDetail,
)

__all__ = [
    # Response
    "ApiResponse",
    "PagedData",
    "ErrorCodes",
    "success_response",
    "error_response",
    "paged_response",
    # Auth
    "LoginRequest",
    "TokenResponse",
    "UserInfo",
    "LoginResponse",
    "UserCreate",
    "UserUpdate",
    "PasswordChange",
    # System
    "SystemConfig",
    "EndpointsConfig",
    "WebRTCConfig",
    "ICEServer",
    "FeaturesConfig",
    "SystemHealth",
    "ServiceHealth",
    "SystemInfo",
    # Control
    "AcquireControlRequest",
    "ControlHolder",
    "ControlStatus",
    "ForceReleaseRequest",
    # Mode
    "ModeSwitchRequest",
    "ModeInfo",
    "ModeStatus",
    # Task
    "TaskAction",
    "CreateTaskRequest",
    "TaskInfo",
    "TaskDetail",
]
