"""
QYH Jushen Control Plane - 腰部控制 API

提供腰部（Waist）的状态查询和运动控制
"""
from fastapi import APIRouter, Depends, HTTPException
from pydantic import BaseModel, Field
from typing import Optional, Literal

from app.dependencies import get_current_operator
from app.models.user import User
from app.services.ros2_client import get_ros2_client_dependency, ROS2ServiceClient, WaistCommand
from app.schemas.response import ApiResponse, success_response, error_response, ErrorCodes

router = APIRouter()


# ==================== 数据模型 ====================

class WaistStatus(BaseModel):
    """腰部状态"""
    angle: float = Field(..., description="当前角度 (度)")
    is_moving: bool = Field(default=False, description="是否正在运动")
    error_code: int = Field(default=0, description="错误码")
    is_enabled: bool = Field(default=True, description="是否使能")


class SetAngleRequest(BaseModel):
    """设置角度请求"""
    angle: float = Field(..., ge=-30, le=90, description="目标角度 (-30~90度)")


class SetSpeedRequest(BaseModel):
    """设置速度请求"""
    speed: float = Field(..., ge=0.0, description="目标速度")


class ManualLeanRequest(BaseModel):
    """手动前倾/后仰请求"""
    direction: Literal['forward', 'back']
    hold: bool = True


# ==================== API 端点 ====================

@router.get("/status", response_model=ApiResponse)
async def get_waist_status(
    ros2: ROS2ServiceClient = Depends(get_ros2_client_dependency),
):
    """获取腰部实时状态"""
    state = ros2.get_robot_state() or {}
    waist_state = state.get("waist", {})
    
    # 映射 ROS2 状态字段
    is_moving = abs(waist_state.get("current_speed", 0.0)) > 0.001
    
    status = WaistStatus(
        angle=float(waist_state.get("current_angle", 0.0)),
        is_moving=is_moving,
        error_code=waist_state.get("alarm", 0),
        is_enabled=waist_state.get("enabled", True)
    )
    
    return success_response(data=status.dict())


@router.post("/angle", response_model=ApiResponse)
async def set_waist_angle(
    request: SetAngleRequest,
    current_user: User = Depends(get_current_operator),
    ros2: ROS2ServiceClient = Depends(get_ros2_client_dependency)
):
    """设置腰部角度"""
    result = await ros2.waist_control(WaistCommand.GO_ANGLE, request.angle)
    
    if result.success:
        return success_response(message=f"已发送腰部角度命令: {request.angle:.1f}度")
    else:
        return error_response(
            code=ErrorCodes.ROS_SERVICE_FAILED,
            message=result.message or "设置腰部角度失败"
        )


@router.post("/upright", response_model=ApiResponse)
async def set_waist_upright(
    current_user: User = Depends(get_current_operator),
    ros2: ROS2ServiceClient = Depends(get_ros2_client_dependency)
):
    """腰部回正 (0度)"""
    result = await ros2.waist_control(WaistCommand.GO_UPRIGHT)
    
    if result.success:
        return success_response(message="已发送腰部回正命令")
    else:
        return error_response(
            code=ErrorCodes.ROS_SERVICE_FAILED,
            message=result.message or "腰部回正失败"
        )


@router.post("/stop", response_model=ApiResponse)
async def stop_waist(
    current_user: User = Depends(get_current_operator),
    ros2: ROS2ServiceClient = Depends(get_ros2_client_dependency)
):
    """停止腰部运动"""
    result = await ros2.waist_control(WaistCommand.STOP)
    
    if result.success:
        return success_response(message="已发送腰部停止命令")
    else:
        return error_response(
            code=ErrorCodes.ROS_SERVICE_FAILED,
            message=result.message or "腰部停止失败"
        )


@router.post("/enable", response_model=ApiResponse)
async def enable_waist(
    enable: bool,
    current_user: User = Depends(get_current_operator),
    ros2: ROS2ServiceClient = Depends(get_ros2_client_dependency)
):
    """使能/失能腰部电机"""
    cmd = WaistCommand.ENABLE if enable else WaistCommand.DISABLE
    result = await ros2.waist_control(cmd)
    
    if result.success:
        action = "使能" if enable else "失能"
        return success_response(message=f"腰部{action}成功")
    else:
        return error_response(
            code=ErrorCodes.ROS_SERVICE_FAILED,
            message=result.message or "操作失败"
        )


@router.post("/speed", response_model=ApiResponse)
async def set_waist_speed(
    request: SetSpeedRequest,
    current_user: User = Depends(get_current_operator),
    ros2: ROS2ServiceClient = Depends(get_ros2_client_dependency)
):
    """设置腰部速度"""
    result = await ros2.waist_control(WaistCommand.SET_SPEED, request.speed)

    if result.success:
        return success_response(message=f"已设置腰部速度: {request.speed}")
    return error_response(
        code=ErrorCodes.ROS_SERVICE_FAILED,
        message=result.message or "设置腰部速度失败"
    )


@router.post("/reset", response_model=ApiResponse)
async def reset_waist_alarm(
    current_user: User = Depends(get_current_operator),
    ros2: ROS2ServiceClient = Depends(get_ros2_client_dependency)
):
    """复位腰部报警"""
    result = await ros2.waist_control(WaistCommand.RESET_ALARM)

    if result.success:
        return success_response(message="腰部报警已复位")
    return error_response(
        code=ErrorCodes.ROS_SERVICE_FAILED,
        message=result.message or "腰部报警复位失败"
    )


@router.post("/lean", response_model=ApiResponse)
async def manual_waist_lean(
    request: ManualLeanRequest,
    current_user: User = Depends(get_current_operator),
    ros2: ROS2ServiceClient = Depends(get_ros2_client_dependency)
):
    """手动前倾/后仰腰部"""
    command = WaistCommand.LEAN_FORWARD if request.direction == 'forward' else WaistCommand.LEAN_BACK
    result = await ros2.waist_control(command, 0.0, hold=request.hold)

    if result.success:
        action = "前倾" if request.direction == 'forward' else "后仰"
        state = "开始" if request.hold else "停止"
        return success_response(message=f"腰部{action}{state}")
    return error_response(
        code=ErrorCodes.ROS_SERVICE_FAILED,
        message=result.message or "腰部手动控制失败"
    )
