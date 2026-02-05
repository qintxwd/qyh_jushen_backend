"""
QYH Jushen Control Plane - 升降柱控制 API

提供升降柱（Lift）的状态查询和运动控制
"""
from fastapi import APIRouter, Depends
from pydantic import BaseModel, Field

from app.dependencies import get_current_operator
from app.models.user import User
from app.services.ros2_client import get_ros2_client_dependency, ROS2ServiceClient, LiftCommand
from app.schemas.response import ApiResponse, success_response, error_response, ErrorCodes

router = APIRouter()


# ==================== 数据模型 ====================

class LiftStatus(BaseModel):
    """升降柱状态"""
    height: float = Field(..., description="当前高度 (m)")
    is_moving: bool = Field(default=False, description="是否正在运动")
    error_code: int = Field(default=0, description="错误码")
    is_enabled: bool = Field(default=True, description="是否使能")


class SetHeightRequest(BaseModel):
    """设置高度请求"""
    height: float = Field(..., ge=0.0, le=1.0, description="目标高度 (0-1.0米)")


# ==================== API 端点 ====================

@router.get("/status", response_model=ApiResponse)
async def get_lift_status(
    ros2: ROS2ServiceClient = Depends(get_ros2_client_dependency),
):
    """获取升降柱实时状态"""
    state = ros2.get_robot_state() or {}
    lift_state = state.get("lift", {})
    
    # 映射 ROS2 状态字段
    is_moving = abs(lift_state.get("current_speed", 0.0)) > 0.001
    
    status = LiftStatus(
        height=float(lift_state.get("current_position", 0.0)),
        is_moving=is_moving,
        error_code=lift_state.get("alarm", 0),
        is_enabled=lift_state.get("enabled", True)
    )
    
    return success_response(data=status.dict())


@router.post("/height", response_model=ApiResponse)
async def set_lift_height(
    request: SetHeightRequest,
    current_user: User = Depends(get_current_operator),
    ros2: ROS2ServiceClient = Depends(get_ros2_client_dependency)
):
    """设置升降柱高度"""
    result = await ros2.lift_control(LiftCommand.GO_POSITION, request.height)
    
    if result.success:
        return success_response(message=f"已发送升降高度命令: {request.height:.3f}m")
    else:
        return error_response(
            code=ErrorCodes.ROS_SERVICE_FAILED,
            message=result.message or "设置升降高度失败"
        )


@router.post("/stop", response_model=ApiResponse)
async def stop_lift(
    current_user: User = Depends(get_current_operator),
    ros2: ROS2ServiceClient = Depends(get_ros2_client_dependency)
):
    """停止升降柱运动"""
    result = await ros2.lift_control(LiftCommand.STOP)
    
    if result.success:
        return success_response(message="已发送升降停止命令")
    else:
        return error_response(
            code=ErrorCodes.ROS_SERVICE_FAILED,
            message=result.message or "升降停止失败"
        )


@router.post("/enable", response_model=ApiResponse)
async def enable_lift(
    enable: bool,
    current_user: User = Depends(get_current_operator),
    ros2: ROS2ServiceClient = Depends(get_ros2_client_dependency)
):
    """使能/失能升降柱"""
    cmd = LiftCommand.ENABLE if enable else LiftCommand.DISABLE
    result = await ros2.lift_control(cmd)
    
    if result.success:
        action = "使能" if enable else "失能"
        return success_response(message=f"升降柱{action}成功")
    else:
        return error_response(
            code=ErrorCodes.ROS_SERVICE_FAILED,
            message=result.message or "操作失败"
        )
