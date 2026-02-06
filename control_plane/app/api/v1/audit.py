"""
QYH Jushen Control Plane - 审计日志 API

提供操作日志的查询和统计功能
"""
import logging
from datetime import datetime
from typing import Optional

from fastapi import APIRouter, Depends, Query
from sqlalchemy.ext.asyncio import AsyncSession

from app.database import get_async_db
from app.dependencies import get_current_user, get_current_admin
from app.models import User
from app.schemas import success_response, error_response, ErrorCodes
from app.schemas.audit import (
    AuditAction,
    AuditResource,
    AuditLogEntry,
    AuditLogQuery,
    AuditLogListResponse,
    AuditStatistics,
)
from app.services.audit_service import AuditService

logger = logging.getLogger(__name__)

router = APIRouter()


# ==================== 日志查询 ====================

@router.get(
    "",
    summary="查询审计日志",
    description="分页查询审计日志，支持多种过滤条件",
)
async def list_audit_logs(
    user_id: Optional[int] = Query(None, description="用户ID"),
    username: Optional[str] = Query(None, description="用户名（模糊匹配）"),
    action: Optional[str] = Query(None, description="操作类型"),
    resource: Optional[str] = Query(None, description="资源类型"),
    resource_id: Optional[str] = Query(None, description="资源ID"),
    start_time: Optional[datetime] = Query(None, description="开始时间"),
    end_time: Optional[datetime] = Query(None, description="结束时间"),
    ip_address: Optional[str] = Query(None, description="IP地址"),
    page: int = Query(1, ge=1, description="页码"),
    page_size: int = Query(20, ge=1, le=100, description="每页数量"),
    db: AsyncSession = Depends(get_async_db),
    current_user: User = Depends(get_current_user),
):
    """
    查询审计日志
    
    - 普通用户只能查看自己的日志
    - 管理员可以查看所有日志
    """
    # 非管理员只能查看自己的日志
    if current_user.role != "admin":
        user_id = current_user.id
    
    query = AuditLogQuery(
        user_id=user_id,
        username=username,
        action=action,
        resource=resource,
        resource_id=resource_id,
        start_time=start_time,
        end_time=end_time,
        ip_address=ip_address,
    )
    
    result = await AuditService.query(db, query, page, page_size)
    
    return success_response(
        data=result.model_dump(),
        message="查询成功",
    )


@router.get(
    "/actions",
    summary="获取操作类型列表",
    description="获取所有可用的操作类型",
)
async def list_action_types(
    current_user: User = Depends(get_current_user),
):
    """获取所有操作类型"""
    actions = [
        {"value": action.value, "label": action.value.replace("_", " ").title()}
        for action in AuditAction
    ]
    return success_response(data=actions)


@router.get(
    "/resources",
    summary="获取资源类型列表",
    description="获取所有可用的资源类型",
)
async def list_resource_types(
    current_user: User = Depends(get_current_user),
):
    """获取所有资源类型"""
    resources = [
        {"value": resource.value, "label": resource.value.title()}
        for resource in AuditResource
    ]
    return success_response(data=resources)


@router.get(
    "/statistics",
    summary="获取审计统计",
    description="获取审计日志的统计信息",
)
async def get_statistics(
    days: int = Query(7, ge=1, le=90, description="统计天数"),
    db: AsyncSession = Depends(get_async_db),
    current_user: User = Depends(get_current_admin),
):
    """
    获取审计统计信息
    
    仅管理员可用
    """
    stats = await AuditService.get_statistics(db, days)
    return success_response(data=stats.model_dump())


@router.get(
    "/my-recent",
    summary="获取我的最近操作",
    description="获取当前用户的最近操作记录",
)
async def get_my_recent_actions(
    limit: int = Query(10, ge=1, le=50, description="数量限制"),
    db: AsyncSession = Depends(get_async_db),
    current_user: User = Depends(get_current_user),
):
    """获取当前用户的最近操作"""
    logs = await AuditService.get_user_recent_actions(db, current_user.id, limit)
    return success_response(
        data=[log.model_dump() for log in logs],
    )


@router.get(
    "/{log_id}",
    summary="获取日志详情",
    description="获取单条审计日志的详细信息",
)
async def get_audit_log(
    log_id: int,
    db: AsyncSession = Depends(get_async_db),
    current_user: User = Depends(get_current_user),
):
    """获取日志详情"""
    log = await AuditService.get_by_id(db, log_id)
    
    if not log:
        return error_response(
            code=ErrorCodes.NOT_FOUND,
            message="日志不存在",
        )
    
    # 非管理员只能查看自己的日志
    if current_user.role != "admin" and log.user_id != current_user.id:
        return error_response(
            code=ErrorCodes.PERMISSION_DENIED,
            message="无权查看此日志",
        )
    
    return success_response(
        data=AuditLogEntry.model_validate(log).model_dump(),
    )


# ==================== 管理操作 ====================

@router.post(
    "/cleanup",
    summary="清理旧日志",
    description="清理指定天数之前的日志",
)
async def cleanup_logs(
    days: int = Query(90, ge=30, le=365, description="保留天数"),
    db: AsyncSession = Depends(get_async_db),
    current_user: User = Depends(get_current_admin),
):
    """
    清理旧日志
    
    仅管理员可用，默认保留 90 天
    """
    count = await AuditService.cleanup_old_logs(db, days)
    
    return success_response(
        data={"deleted_count": count},
        message=f"已清理 {count} 条日志",
    )
