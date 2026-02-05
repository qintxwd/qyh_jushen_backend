"""
QYH Jushen Control Plane - 腰部控制 API

提供腰部（Waist）的状态查询和运动控制
"""
from fastapi import APIRouter, Depends, HTTPException
from pydantic import BaseModel, Field
from typing import Optional

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


# ==================== API 端点 ====================

@router.get("/status", response_model=ApiResponse)
async def get_waist_status(
    ros2: ROS2ServiceClient = Depends(get_ros2_client_dependency),
):
    """获取腰部实时状态"""
    state = ros2.get_robot_state()
    waist_state = state.get("waist", {})
    
    status = WaistStatus(
        angle=float(waist_state.get("current_angle", 0.0)),
        is_moving=waist_state.get("is_moving", False),
        error_code=waist_state.get("error_code", 0),
        is_enabled=True  # 默认假设使能，实际应从 detailed status 获取
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
