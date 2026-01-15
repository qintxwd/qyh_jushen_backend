"""
统一 API 响应模型

提供标准化的响应格式，确保所有 API 返回一致的数据结构。
"""

from typing import Any, Optional, Generic, TypeVar
from pydantic import BaseModel, Field
from datetime import datetime

T = TypeVar("T")


class ApiResponse(BaseModel, Generic[T]):
    """
    统一 API 响应模型
    
    所有 API 应使用此模型返回数据，确保响应格式一致。
    
    示例:
    ```python
    @router.get("/example")
    async def example():
        return ApiResponse(
            success=True,
            data={"key": "value"},
            message="操作成功"
        )
    ```
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


class PagedResponse(BaseModel, Generic[T]):
    """
    分页响应模型
    
    用于列表类 API 的分页响应。
    """
    success: bool = True
    code: int = 0
    message: str = ""
    data: list[T] = Field(default_factory=list, description="数据列表")
    total: int = Field(default=0, description="总记录数")
    page: int = Field(default=1, description="当前页码")
    page_size: int = Field(default=20, description="每页大小")
    timestamp: datetime = Field(default_factory=datetime.utcnow)


class ErrorDetail(BaseModel):
    """错误详情"""
    code: str = Field(..., description="错误码，如 INVALID_PARAMETER")
    message: str = Field(..., description="错误消息")
    field: Optional[str] = Field(default=None, description="出错字段")
    details: Optional[dict] = Field(default=None, description="额外详情")


class ErrorResponse(BaseModel):
    """
    错误响应模型
    
    用于 HTTP 4xx/5xx 错误响应。
    """
    success: bool = False
    code: int = Field(..., description="HTTP 状态码")
    message: str = Field(..., description="错误消息")
    error: Optional[ErrorDetail] = Field(default=None, description="错误详情")
    timestamp: datetime = Field(default_factory=datetime.utcnow)


# ==================== 便捷函数 ====================

def success_response(
    data: Any = None,
    message: str = "操作成功",
    code: int = 0
) -> dict:
    """
    创建成功响应
    
    Args:
        data: 响应数据
        message: 响应消息
        code: 业务状态码
    
    Returns:
        响应字典
    """
    return {
        "success": True,
        "code": code,
        "message": message,
        "data": data,
        "timestamp": datetime.utcnow().isoformat() + "Z"
    }


def error_response(
    message: str,
    code: int = -1,
    error_code: str = None,
    field: str = None,
    details: dict = None
) -> dict:
    """
    创建错误响应
    
    Args:
        message: 错误消息
        code: 业务状态码
        error_code: 错误码字符串
        field: 出错字段
        details: 额外详情
    
    Returns:
        响应字典
    """
    response = {
        "success": False,
        "code": code,
        "message": message,
        "timestamp": datetime.utcnow().isoformat() + "Z"
    }
    
    if error_code or field or details:
        response["error"] = {
            "code": error_code or "ERROR",
            "message": message,
            "field": field,
            "details": details
        }
    
    return response


# ==================== 常用错误码 ====================

class ErrorCodes:
    """业务错误码常量"""
    
    # 通用错误 (1xxx)
    UNKNOWN_ERROR = 1000
    INVALID_PARAMETER = 1001
    MISSING_PARAMETER = 1002
    UNAUTHORIZED = 1003
    FORBIDDEN = 1004
    NOT_FOUND = 1005
    
    # 认证错误 (11xx)
    AUTH_INVALID_CREDENTIALS = 1101
    AUTH_USER_DISABLED = 1102
    AUTH_TOKEN_EXPIRED = 1103
    AUTH_TOKEN_INVALID = 1104
    
    # 控制权错误 (2xxx)
    CONTROL_NOT_ACQUIRED = 2001
    CONTROL_HELD_BY_OTHER = 2002
    CONTROL_EXPIRED = 2003
    CONTROL_ALREADY_HELD = 2004
    CONTROL_NOT_HELD = 2005
    
    # 机械臂错误 (3xxx)
    ARM_NOT_CONNECTED = 3001
    ARM_NOT_ENABLED = 3002
    ARM_IN_ERROR = 3003
    ARM_MOTION_FAILED = 3004
    
    # 底盘错误 (4xxx)
    CHASSIS_NOT_CONNECTED = 4001
    NAVIGATION_FAILED = 4002
    LOCALIZATION_FAILED = 4003
    
    # 夹爪错误 (5xxx)
    GRIPPER_NOT_ACTIVATED = 5001
    GRIPPER_MOTION_FAILED = 5002
    
    # 机器人状态错误 (6xxx)
    ROBOT_NOT_CONNECTED = 6001
    ROBOT_NOT_READY = 6002
    
    # 任务错误 (7xxx)
    TASK_NOT_FOUND = 7001
    TASK_START_FAILED = 7002
    TASK_INVALID_STATUS = 7003
    TASK_EXECUTION_FAILED = 7004
    
    # ROS2 错误 (9xxx)
    ROS2_NOT_CONNECTED = 9001
    ROS2_SERVICE_TIMEOUT = 9002
    ROS2_SERVICE_FAILED = 9003
