"""
QYH Jushen Control Plane - 工作模式管理 API

管理机器人的工作模式切换
"""
from fastapi import APIRouter, Depends, Request
from sqlalchemy.orm import Session

from app.dependencies import get_current_operator
from app.database import get_db
from app.models.user import User
from app.core.mode_manager import mode_manager, RobotMode
from app.core.control_lock import control_lock
from app.schemas.response import ApiResponse, success_response, error_response, ErrorCodes
from app.schemas.mode import ModeSwitchRequest, ModeStatus
from app.services.audit_service import audit_log_sync

router = APIRouter()


@router.get("/current", response_model=ApiResponse)
async def get_current_mode():
    """
    获取当前工作模式
    """
    status = mode_manager.get_status()
    
    return success_response(
        data=ModeStatus(**status).model_dump(),
        message="获取成功"
    )


@router.post("/switch", response_model=ApiResponse)
async def switch_mode(
    request: ModeSwitchRequest,
    http_request: Request,
    current_user: User = Depends(get_current_operator),
    db: Session = Depends(get_db),
):
    """
    切换工作模式
    
    模式切换规则：
    - idle -> teleop, auto, maintenance
    - teleop -> idle (需要先释放控制权)
    - auto -> idle (任务完成或取消后)
    - maintenance -> idle
    - error -> idle, maintenance
    """
    target_mode = RobotMode(request.target_mode)
    current_mode = mode_manager.current_mode
    
    # 如果从 teleop 切换出去，需要检查控制权
    if current_mode == RobotMode.TELEOP and target_mode != RobotMode.TELEOP:
        holder = control_lock.get_holder()
        if holder and holder["user_id"] != current_user.id:
            return error_response(
                code=ErrorCodes.CONTROL_ALREADY_HELD,
                message=f"控制权由 {holder['username']} 持有，无法切换模式"
            )
        # 自动释放控制权
        control_lock.release(current_user.id)
    
    # 如果切换到 teleop，需要获取控制权
    if target_mode == RobotMode.TELEOP:
        success = control_lock.acquire(
            user_id=current_user.id,
            username=current_user.username,
            duration=300,
            session_type="teleop",
        )
        if not success:
            holder = control_lock.get_holder()
            return error_response(
                code=ErrorCodes.CONTROL_ALREADY_HELD,
                message=f"无法获取控制权，当前由 {holder['username']} 持有"
            )
    
    # 执行模式切换
    old_mode = mode_manager.current_mode.value
    success, message = mode_manager.switch_to(target_mode, force=request.force)
    
    if success:
        # 记录审计日志
        audit_log_sync(
            db=db,
            action="mode_switch",
            resource="mode",
            details={
                "from_mode": old_mode,
                "to_mode": target_mode.value,
                "forced": request.force,
            },
            user=current_user,
            request=http_request,
        )
        status = mode_manager.get_status()
        return success_response(
            data=ModeStatus(**status).model_dump(),
            message=message
        )
    else:
        return error_response(
            code=ErrorCodes.MODE_INVALID_TRANSITION,
            message=message
        )


@router.get("/available", response_model=ApiResponse)
async def get_available_modes():
    """
    获取所有可用模式及其描述
    """
    modes = mode_manager.get_available_modes()
    
    return success_response(
        data={"modes": modes},
        message="获取成功"
    )


@router.get("/status", response_model=ApiResponse)
async def get_mode_status_alias():
    """
    获取当前工作模式 (Alias for /current)
    """
    return await get_current_mode()
