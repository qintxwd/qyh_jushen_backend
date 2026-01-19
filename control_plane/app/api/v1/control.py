"""
QYH Jushen Control Plane - 控制权管理 API

实现机器人控制权的获取、释放、续约等功能
"""
from fastapi import APIRouter, Depends

from app.dependencies import get_current_operator, get_current_admin
from app.models.user import User
from app.core.control_lock import control_lock
from app.schemas.response import ApiResponse, success_response, error_response, ErrorCodes
from app.schemas.control import (
    AcquireControlRequest,
    ControlHolder,
    ControlStatus,
    ForceReleaseRequest,
)

router = APIRouter()


@router.post("/acquire", response_model=ApiResponse)
async def acquire_control(
    request: AcquireControlRequest,
    current_user: User = Depends(get_current_operator),
):
    """
    获取机器人控制权
    
    同一时间只能有一个用户持有控制权
    """
    success = control_lock.acquire(
        user_id=current_user.id,
        username=current_user.username,
        duration=request.duration,
        session_type=request.session_type,
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
            data={"holder": holder}
        )


@router.post("/release", response_model=ApiResponse)
async def release_control(
    current_user: User = Depends(get_current_operator),
):
    """
    释放控制权
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
    request: AcquireControlRequest,
    current_user: User = Depends(get_current_operator),
):
    """
    续约控制权
    """
    if not control_lock.is_held_by(current_user.id):
        return error_response(
            code=ErrorCodes.CONTROL_NOT_HELD,
            message="您未持有控制权"
        )
    
    control_lock.acquire(
        current_user.id,
        current_user.username,
        request.duration,
        request.session_type,
    )
    
    holder = control_lock.get_holder()
    
    return success_response(
        data={"holder": holder},
        message="控制权续约成功"
    )


@router.get("/status", response_model=ApiResponse)
async def get_control_status():
    """
    查询控制权状态
    
    此接口不需要认证，任何人都可以查询当前控制权状态
    """
    holder = control_lock.get_holder()
    
    status = ControlStatus(
        locked=holder is not None,
        holder=ControlHolder(**holder) if holder else None,
    )
    
    return success_response(
        data=status.model_dump(),
        message="查询成功"
    )


@router.post("/force-release", response_model=ApiResponse)
async def force_release_control(
    request: ForceReleaseRequest,
    current_user: User = Depends(get_current_admin),
):
    """
    强制释放控制权（仅管理员）
    """
    old_holder = control_lock.force_release(request.reason)
    
    if old_holder:
        return success_response(
            data={"previous_holder": old_holder},
            message=f"已强制释放 {old_holder['username']} 的控制权"
        )
    else:
        return success_response(
            message="当前没有控制权持有者"
        )
