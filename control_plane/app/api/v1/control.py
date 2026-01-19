"""
QYH Jushen Control Plane - 控制权管理 API

实现机器人控制权的获取、释放、续约等功能
"""
from fastapi import APIRouter, Depends, Request
from sqlalchemy.ext.asyncio import AsyncSession

from app.dependencies import get_current_operator, get_current_admin
from app.database import get_async_db
from app.models.user import User
from app.core.control_lock import control_lock
from app.schemas.response import ApiResponse, success_response, error_response, ErrorCodes
from app.schemas.control import (
    AcquireControlRequest,
    ControlHolder,
    ControlStatus,
    ForceReleaseRequest,
)
from app.services.audit_service import AuditService
from app.services.control_session_service import (
    create_control_session,
    end_control_session,
    ControlSessionService,
)

router = APIRouter()


@router.post("/acquire", response_model=ApiResponse)
async def acquire_control(
    request: AcquireControlRequest,
    http_request: Request,
    current_user: User = Depends(get_current_operator),
    db: AsyncSession = Depends(get_async_db),
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
        
        # 持久化：创建控制权会话记录
        await create_control_session(
            db=db,
            user_id=current_user.id,
            username=current_user.username,
            session_type=request.session_type,
        )
        
        # 记录审计日志
        await AuditService.log(
            db=db,
            action="control_acquire",
            resource="control",
            details={"session_type": request.session_type, "duration": request.duration},
            user=current_user,
            request=http_request,
        )
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
    http_request: Request,
    current_user: User = Depends(get_current_operator),
    db: AsyncSession = Depends(get_async_db),
):
    """
    释放控制权
    """
    success = control_lock.release(current_user.id)
    
    if success:
        # 持久化：结束控制权会话
        await end_control_session(db, current_user.id, "released")
        
        # 记录审计日志
        await AuditService.log(
            db=db,
            action="control_release",
            resource="control",
            details={"reason": "user_release"},
            user=current_user,
            request=http_request,
        )
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
    http_request: Request,
    current_user: User = Depends(get_current_admin),
    db: AsyncSession = Depends(get_async_db),
):
    """
    强制释放控制权（仅管理员）
    """
    old_holder = control_lock.force_release(request.reason)
    
    if old_holder:
        # 持久化：结束被强制释放用户的会话
        await end_control_session(db, old_holder["user_id"], f"forced:{request.reason}")
        
        # 记录审计日志
        await AuditService.log(
            db=db,
            action="control_force_release",
            resource="control",
            details={
                "reason": request.reason,
                "previous_holder": old_holder["username"],
                "previous_user_id": old_holder["user_id"],
            },
            user=current_user,
            request=http_request,
        )
        return success_response(
            data={"previous_holder": old_holder},
            message=f"已强制释放 {old_holder['username']} 的控制权"
        )
    else:
        return success_response(
            message="当前没有控制权持有者"
        )


@router.get("/history", response_model=ApiResponse)
async def get_control_history(
    user_id: int = None,
    limit: int = 50,
    offset: int = 0,
    current_user: User = Depends(get_current_operator),
    db: AsyncSession = Depends(get_async_db),
):
    """
    获取控制权会话历史
    
    管理员可查看所有用户的历史，普通用户只能查看自己的
    """
    # 普通用户只能查看自己的历史
    if current_user.role != "admin" and user_id is not None and user_id != current_user.id:
        return error_response(
            code=ErrorCodes.PERMISSION_DENIED,
            message="无权查看其他用户的控制权历史"
        )
    
    # 非管理员默认只查自己的
    if current_user.role != "admin":
        user_id = current_user.id
    
    sessions = await ControlSessionService.get_history(db, user_id, limit, offset)
    
    return success_response(
        data={
            "sessions": [
                {
                    "id": s.id,
                    "user_id": s.user_id,
                    "username": s.username,
                    "session_type": s.session_type,
                    "started_at": s.started_at.isoformat() if s.started_at else None,
                    "ended_at": s.ended_at.isoformat() if s.ended_at else None,
                    "end_reason": s.end_reason,
                    "duration_seconds": s.duration_seconds,
                }
                for s in sessions
            ],
            "limit": limit,
            "offset": offset,
        },
        message="获取控制权历史成功"
    )


@router.get("/statistics", response_model=ApiResponse)
async def get_control_statistics(
    days: int = 7,
    current_user: User = Depends(get_current_admin),
    db: AsyncSession = Depends(get_async_db),
):
    """
    获取控制权统计信息（仅管理员）
    """
    stats = await ControlSessionService.get_statistics(db, days)
    
    return success_response(
        data=stats,
        message="获取统计成功"
    )
