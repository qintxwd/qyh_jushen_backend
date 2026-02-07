"""
QYH Jushen Control Plane - 夹爪控制 API
"""
from fastapi import APIRouter, Depends, Query
from app.dependencies import get_current_user
from app.models.user import User
from app.services.ros2_client import get_ros2_client_dependency
from app.schemas.response import (
    ApiResponse, success_response, error_response, ErrorCodes
)

router = APIRouter()

@router.post("/{side}/enable", summary="激活夹爪")
async def activate_gripper(
    side: str,
    client = Depends(get_ros2_client_dependency),
    current_user: User = Depends(get_current_user)
):
    """
    激活指定侧夹爪 (left/right)
    """
    if side not in ["left", "right"]:
        return error_response(
            code=ErrorCodes.INVALID_PARAMS,
            message="Side must be 'left' or 'right'"
        )

    result = await client.gripper_activate(side)
    
    if result.success:
        return success_response(message=f"{side} gripper activated successfully")
    else:
        return error_response(
            code=ErrorCodes.SERVICE_ERROR,
            message=f"Failed to activate {side} gripper: {result.message}"
        )

@router.post("/{side}/move", summary="移动夹爪")
async def move_gripper(
    side: str,
    position: int = Query(..., ge=0, le=255, description="Position (0-255)"),
    speed: int = Query(100, ge=0, le=255, description="Speed (0-255)"),
    force: int = Query(50, ge=0, le=255, description="Force (0-255)"),
    client = Depends(get_ros2_client_dependency),
    current_user: User = Depends(get_current_user)
):
    """
    移动夹爪
    position: 0-255 (0=Open, 255=Close)
    """
    if side not in ["left", "right"]:
        return error_response(
            code=ErrorCodes.INVALID_PARAMS,
            message="Side must be 'left' or 'right'"
        )

    result = await client.gripper_move(side, position, speed, force)
    
    if result.success:
        return success_response(message=f"{side} gripper moved successfully")
    else:
        return error_response(
            code=ErrorCodes.SERVICE_ERROR,
            message=f"Failed to move {side} gripper: {result.message}"
        )
