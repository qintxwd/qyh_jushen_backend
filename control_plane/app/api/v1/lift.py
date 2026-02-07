"""
QYH Jushen Control Plane - 升降柱控制 API

提供升降柱（Lift）的状态查询和运动控制
"""
from fastapi import APIRouter, Depends
from typing import Literal
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
    electromagnet_on: bool = Field(default=False, description="电磁铁状态")
    connected: bool = Field(default=False, description="通信状态")


class SetHeightRequest(BaseModel):
    """设置高度请求"""
    height: float = Field(..., ge=0.0, le=1.0, description="目标高度 (0-1.0米)")


class SetSpeedRequest(BaseModel):
    """设置速度请求"""
    speed: float = Field(..., ge=0.0, description="目标速度")


class ManualMoveRequest(BaseModel):
    """手动移动请求"""
    direction: Literal['up', 'down']
    hold: bool = True


class ElectromagnetRequest(BaseModel):
    """电磁铁控制请求"""
    enable: bool = Field(..., description="True=开启, False=关闭")


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
        is_enabled=lift_state.get("enabled", True),
        electromagnet_on=lift_state.get("electromagnet_on", False),
        connected=lift_state.get("connected", False),
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


@router.post("/speed", response_model=ApiResponse)
async def set_lift_speed(
    request: SetSpeedRequest,
    current_user: User = Depends(get_current_operator),
    ros2: ROS2ServiceClient = Depends(get_ros2_client_dependency)
):
    """设置升降柱速度"""
    result = await ros2.lift_control(LiftCommand.SET_SPEED, request.speed)

    if result.success:
        return success_response(message=f"已设置升降速度: {request.speed}")
    return error_response(
        code=ErrorCodes.ROS_SERVICE_FAILED,
        message=result.message or "设置升降速度失败"
    )


@router.post("/reset", response_model=ApiResponse)
async def reset_lift_alarm(
    current_user: User = Depends(get_current_operator),
    ros2: ROS2ServiceClient = Depends(get_ros2_client_dependency)
):
    """复位升降柱报警"""
    result = await ros2.lift_control(LiftCommand.RESET_ALARM)

    if result.success:
        return success_response(message="升降报警已复位")
    return error_response(
        code=ErrorCodes.ROS_SERVICE_FAILED,
        message=result.message or "升降报警复位失败"
    )


@router.post("/manual", response_model=ApiResponse)
async def manual_move_lift(
    request: ManualMoveRequest,
    current_user: User = Depends(get_current_operator),
    ros2: ROS2ServiceClient = Depends(get_ros2_client_dependency)
):
    """手动移动升降柱"""
    command = LiftCommand.MOVE_UP if request.direction == 'up' else LiftCommand.MOVE_DOWN
    result = await ros2.lift_control(command, 0.0, hold=request.hold)

    if result.success:
        action = "上升" if request.direction == 'up' else "下降"
        state = "开始" if request.hold else "停止"
        return success_response(message=f"升降{action}{state}")
    return error_response(
        code=ErrorCodes.ROS_SERVICE_FAILED,
        message=result.message or "升降手动控制失败"
    )


@router.post("/electromagnet", response_model=ApiResponse)
async def set_lift_electromagnet(
    request: ElectromagnetRequest,
    current_user: User = Depends(get_current_operator),
    ros2: ROS2ServiceClient = Depends(get_ros2_client_dependency)
):
    """电磁铁开关"""
    value = 1.0 if request.enable else 0.0
    result = await ros2.lift_control(LiftCommand.ELECTROMAGNET, value)

    if result.success:
        action = "开启" if request.enable else "关闭"
        return success_response(message=f"电磁铁{action}")
    return error_response(
        code=ErrorCodes.ROS_SERVICE_FAILED,
        message=result.message or "电磁铁控制失败"
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
