"""控制权管理 API"""
from fastapi import APIRouter, Depends
from pydantic import BaseModel
from app.dependencies import get_current_operator
from app.models.user import User
from app.core.control_lock import control_lock
from app.schemas.response import (
    ApiResponse, success_response, error_response, ErrorCodes
)

router = APIRouter()


class AcquireRequest(BaseModel):
    """获取控制权请求"""
    duration: int = 300  # 默认5分钟


@router.post("/acquire", response_model=ApiResponse)
async def acquire_control(
    request: AcquireRequest,
    current_user: User = Depends(get_current_operator)
):
    """获取机器人控制权
    
    Returns:
        ApiResponse: 统一响应格式，data 包含 holder 信息
    """
    success = control_lock.acquire(
        user_id=current_user.id,
        username=current_user.username,
        duration=request.duration
    )
    
    if success:
        holder = control_lock.get_holder()
        return success_response(
            data={"holder": holder},
            message="控制权获取成功"
        )
    else:
        holder = control_lock.get_holder()
        return error_response(
            code=ErrorCodes.CONTROL_ALREADY_HELD,
            message=f"控制权已被用户 {holder['username']} 持有",
            details={"holder": holder}
        )


@router.post("/release", response_model=ApiResponse)
async def release_control(current_user: User = Depends(get_current_operator)):
    """释放控制权
    
    Returns:
        ApiResponse: 统一响应格式
    """
    success = control_lock.release(current_user.id)
    
    if success:
        return success_response(message="控制权已释放")
    else:
        return error_response(
            code=ErrorCodes.CONTROL_NOT_HELD,
            message="您未持有控制权"
        )


@router.post("/renew", response_model=ApiResponse)
async def renew_control(
    request: AcquireRequest,
    current_user: User = Depends(get_current_operator)
):
    """续约控制权
    
    Returns:
        ApiResponse: 统一响应格式，data 包含 holder 信息
    """
    if not control_lock.is_held_by(current_user.id):
        return error_response(
            code=ErrorCodes.CONTROL_NOT_HELD,
            message="您未持有控制权"
        )
    
    control_lock.acquire(current_user.id, current_user.username, request.duration)
    holder = control_lock.get_holder()
    
    return success_response(
        data={"holder": holder},
        message="控制权续约成功"
    )


@router.get("/status", response_model=ApiResponse)
async def get_control_status():
    """查询控制权状态
    
    Returns:
        ApiResponse: 统一响应格式，data 包含 locked 和 holder
    """
    holder = control_lock.get_holder()
    
    return success_response(
        data={
            "locked": holder is not None,
            "holder": holder
        }
    )
