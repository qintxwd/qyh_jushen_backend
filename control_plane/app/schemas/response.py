"""
QYH Jushen Control Plane - 统一响应模型

提供标准化的响应格式，确保所有 API 返回一致的数据结构
"""
from typing import Any, Optional, Generic, TypeVar
from datetime import datetime

from pydantic import BaseModel, Field

T = TypeVar("T")


class ApiResponse(BaseModel, Generic[T]):
    """
    统一 API 响应模型
    
    所有 API 应使用此模型返回数据，确保响应格式一致。
    """
    success: bool = Field(..., description="操作是否成功")
    code: int = Field(default=0, description="业务状态码，0=成功")
    message: str = Field(default="", description="响应消息")
    data: Optional[T] = Field(default=None, description="响应数据")
    timestamp: datetime = Field(
        default_factory=datetime.utcnow,
        description="响应时间戳"
    )

    class Config:
        json_encoders = {
            datetime: lambda v: v.isoformat() + "Z"
        }


class PagedData(BaseModel, Generic[T]):
    """分页数据"""
    items: list[T] = Field(default_factory=list, description="数据列表")
    total: int = Field(default=0, description="总数")
    page: int = Field(default=1, description="当前页")
    page_size: int = Field(default=20, description="每页大小")
    pages: int = Field(default=0, description="总页数")


# ==================== 错误码定义 ====================

class ErrorCodes:
    """业务错误码"""
    
    # 通用错误 (1xxx)
    UNKNOWN_ERROR = 1000
    VALIDATION_ERROR = 1001
    NOT_FOUND = 1002
    INTERNAL_ERROR = 1003
    
    # 认证错误 (2xxx)
    AUTH_INVALID_CREDENTIALS = 2001
    AUTH_TOKEN_EXPIRED = 2002
    AUTH_TOKEN_INVALID = 2003
    AUTH_USER_DISABLED = 2004
    AUTH_PERMISSION_DENIED = 2005
    
    # 控制权错误 (3xxx)
    CONTROL_ALREADY_HELD = 3001
    CONTROL_NOT_HELD = 3002
    CONTROL_TIMEOUT = 3003
    
    # 模式错误 (4xxx)
    MODE_INVALID_TRANSITION = 4001
    MODE_OPERATION_NOT_ALLOWED = 4002
    
    # 任务错误 (5xxx)
    TASK_NOT_FOUND = 5001
    TASK_ALREADY_RUNNING = 5002
    TASK_NOT_RUNNING = 5003
    TASK_INVALID_STATE = 5004
    
    # 系统错误 (6xxx)
    ROS2_NOT_CONNECTED = 6001
    ROBOT_IN_ERROR = 6002
    EMERGENCY_STOP_ACTIVE = 6003
    ROS2_SERVICE_FAILED = 6004


# ==================== 响应构造函数 ====================

def success_response(
    data: Any = None,
    message: str = "操作成功",
    code: int = 0,
) -> ApiResponse:
    """构造成功响应"""
    return ApiResponse(
        success=True,
        code=code,
        message=message,
        data=data,
    )


def error_response(
    message: str,
    code: int = ErrorCodes.UNKNOWN_ERROR,
    data: Any = None,
) -> ApiResponse:
    """构造错误响应"""
    return ApiResponse(
        success=False,
        code=code,
        message=message,
        data=data,
    )


def paged_response(
    items: list,
    total: int,
    page: int = 1,
    page_size: int = 20,
    message: str = "查询成功",
) -> ApiResponse:
    """构造分页响应"""
    pages = (total + page_size - 1) // page_size if page_size > 0 else 0
    
    return ApiResponse(
        success=True,
        code=0,
        message=message,
        data=PagedData(
            items=items,
            total=total,
            page=page,
            page_size=page_size,
            pages=pages,
        ),
    )
