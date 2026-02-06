"""
QYH Jushen Control Plane - 审计日志服务
"""
import logging
from datetime import datetime, timedelta
from typing import Optional, Any

from sqlalchemy import select, func, desc, and_
from sqlalchemy.ext.asyncio import AsyncSession
from fastapi import Request

from app.models import AuditLog, User
from app.config import settings
from app.schemas.audit import (
    AuditLogEntry,
    AuditLogQuery,
    AuditLogListResponse,
    AuditStatistics,
    AuditActionCount,
    AuditUserCount,
    AuditDailyCount,
)

logger = logging.getLogger(__name__)


class AuditService:
    """审计日志服务"""
    
    @staticmethod
    async def log(
        db: AsyncSession,
        action: str,
        resource: Optional[str] = None,
        resource_id: Optional[str] = None,
        details: Optional[dict[str, Any]] = None,
        user: Optional[User] = None,
        request: Optional[Request] = None,
    ) -> AuditLog:
        """
        记录审计日志
        
        Args:
            db: 数据库会话
            action: 操作类型
            resource: 资源类型
            resource_id: 资源ID
            details: 详细信息
            user: 当前用户
            request: HTTP 请求
        """
        # 提取用户信息
        user_id = user.id if user else None
        username = user.username if user else None
        
        # 提取请求信息
        ip_address = None
        user_agent = None
        if request:
            # 获取真实 IP（考虑代理）
            if settings.TRUST_PROXY:
                forwarded = request.headers.get("X-Forwarded-For")
            else:
                forwarded = None
            if forwarded:
                ip_address = forwarded.split(",")[0].strip()
            else:
                ip_address = request.client.host if request.client else None
            user_agent = request.headers.get("User-Agent", "")[:255]
        
        # 创建日志记录
        log_entry = AuditLog(
            user_id=user_id,
            username=username,
            action=action,
            resource=resource,
            resource_id=resource_id,
            details=details,
            ip_address=ip_address,
            user_agent=user_agent,
            created_at=datetime.utcnow(),
        )
        
        db.add(log_entry)
        await db.commit()
        await db.refresh(log_entry)
        
        logger.info(
            f"Audit: {action} on {resource}/{resource_id} by {username or 'anonymous'}"
        )
        
        return log_entry
    
    @staticmethod
    async def query(
        db: AsyncSession,
        query: AuditLogQuery,
        page: int = 1,
        page_size: int = 20,
    ) -> AuditLogListResponse:
        """
        查询审计日志
        
        Args:
            db: 数据库会话
            query: 查询参数
            page: 页码
            page_size: 每页数量
        """
        # 构建查询条件
        conditions = []
        
        if query.user_id is not None:
            conditions.append(AuditLog.user_id == query.user_id)
        if query.username:
            conditions.append(AuditLog.username.ilike(f"%{query.username}%"))
        if query.action:
            conditions.append(AuditLog.action == query.action)
        if query.resource:
            conditions.append(AuditLog.resource == query.resource)
        if query.resource_id:
            conditions.append(AuditLog.resource_id == query.resource_id)
        if query.start_time:
            conditions.append(AuditLog.created_at >= query.start_time)
        if query.end_time:
            conditions.append(AuditLog.created_at <= query.end_time)
        if query.ip_address:
            conditions.append(AuditLog.ip_address == query.ip_address)
        
        # 查询总数
        count_stmt = select(func.count(AuditLog.id))
        if conditions:
            count_stmt = count_stmt.where(and_(*conditions))
        total = (await db.execute(count_stmt)).scalar() or 0
        
        # 计算分页
        total_pages = (total + page_size - 1) // page_size if total > 0 else 1
        offset = (page - 1) * page_size
        
        # 查询数据
        stmt = select(AuditLog).order_by(desc(AuditLog.created_at))
        if conditions:
            stmt = stmt.where(and_(*conditions))
        stmt = stmt.offset(offset).limit(page_size)
        
        result = await db.execute(stmt)
        logs = result.scalars().all()
        
        return AuditLogListResponse(
            items=[AuditLogEntry.model_validate(log) for log in logs],
            total=total,
            page=page,
            page_size=page_size,
            total_pages=total_pages,
        )
    
    @staticmethod
    async def get_by_id(db: AsyncSession, log_id: int) -> Optional[AuditLog]:
        """获取单条日志"""
        stmt = select(AuditLog).where(AuditLog.id == log_id)
        result = await db.execute(stmt)
        return result.scalar_one_or_none()
    
    @staticmethod
    async def get_statistics(
        db: AsyncSession,
        days: int = 7,
    ) -> AuditStatistics:
        """
        获取审计统计信息
        
        Args:
            db: 数据库会话
            days: 统计天数
        """
        now = datetime.utcnow()
        today_start = now.replace(hour=0, minute=0, second=0, microsecond=0)
        period_start = today_start - timedelta(days=days - 1)
        
        # 总日志数
        total_stmt = select(func.count(AuditLog.id))
        total_logs = (await db.execute(total_stmt)).scalar() or 0
        
        # 今日日志数
        today_stmt = select(func.count(AuditLog.id)).where(
            AuditLog.created_at >= today_start
        )
        today_logs = (await db.execute(today_stmt)).scalar() or 0
        
        # 按操作类型统计
        action_stmt = (
            select(AuditLog.action, func.count(AuditLog.id).label("count"))
            .where(AuditLog.created_at >= period_start)
            .group_by(AuditLog.action)
            .order_by(desc("count"))
            .limit(10)
        )
        action_result = await db.execute(action_stmt)
        by_action = [
            AuditActionCount(action=row[0], count=row[1])
            for row in action_result.all()
        ]
        
        # 按用户统计
        user_stmt = (
            select(
                AuditLog.user_id,
                AuditLog.username,
                func.count(AuditLog.id).label("count")
            )
            .where(AuditLog.created_at >= period_start)
            .group_by(AuditLog.user_id, AuditLog.username)
            .order_by(desc("count"))
            .limit(10)
        )
        user_result = await db.execute(user_stmt)
        by_user = [
            AuditUserCount(user_id=row[0], username=row[1], count=row[2])
            for row in user_result.all()
        ]
        
        # 按日期统计（最近 N 天）
        # 使用 SQLite 的 date() 函数
        daily_stmt = (
            select(
                func.date(AuditLog.created_at).label("date"),
                func.count(AuditLog.id).label("count")
            )
            .where(AuditLog.created_at >= period_start)
            .group_by(func.date(AuditLog.created_at))
            .order_by("date")
        )
        daily_result = await db.execute(daily_stmt)
        recent_days = [
            AuditDailyCount(date=str(row[0]), count=row[1])
            for row in daily_result.all()
        ]
        
        return AuditStatistics(
            total_logs=total_logs,
            today_logs=today_logs,
            by_action=by_action,
            by_user=by_user,
            recent_days=recent_days,
        )
    
    @staticmethod
    async def get_user_recent_actions(
        db: AsyncSession,
        user_id: int,
        limit: int = 10,
    ) -> list[AuditLogEntry]:
        """获取用户最近操作"""
        stmt = (
            select(AuditLog)
            .where(AuditLog.user_id == user_id)
            .order_by(desc(AuditLog.created_at))
            .limit(limit)
        )
        result = await db.execute(stmt)
        logs = result.scalars().all()
        return [AuditLogEntry.model_validate(log) for log in logs]
    
    @staticmethod
    async def cleanup_old_logs(
        db: AsyncSession,
        days: int = 90,
    ) -> int:
        """
        清理旧日志
        
        Args:
            db: 数据库会话
            days: 保留天数
            
        Returns:
            删除的日志数量
        """
        cutoff = datetime.utcnow() - timedelta(days=days)
        
        # 先统计数量
        count_stmt = select(func.count(AuditLog.id)).where(
            AuditLog.created_at < cutoff
        )
        count = (await db.execute(count_stmt)).scalar() or 0
        
        if count > 0:
            # 批量删除
            from sqlalchemy import delete
            delete_stmt = delete(AuditLog).where(AuditLog.created_at < cutoff)
            await db.execute(delete_stmt)
            await db.commit()
            logger.info(f"Cleaned up {count} audit logs older than {days} days")
        
        return count


# 便捷函数：用于在其他模块中快速记录审计日志
async def audit_log(
    db: AsyncSession,
    action: str,
    resource: Optional[str] = None,
    resource_id: Optional[str] = None,
    details: Optional[dict[str, Any]] = None,
    user: Optional[User] = None,
    request: Optional[Request] = None,
) -> AuditLog:
    """便捷函数：记录审计日志（异步版本）"""
    return await AuditService.log(
        db=db,
        action=action,
        resource=resource,
        resource_id=resource_id,
        details=details,
        user=user,
        request=request,
    )


def audit_log_sync(
    db,  # 同步 Session
    action: str,
    resource: Optional[str] = None,
    resource_id: Optional[str] = None,
    details: Optional[dict[str, Any]] = None,
    user: Optional[User] = None,
    request: Optional[Request] = None,
) -> AuditLog:
    """
    便捷函数：记录审计日志（同步版本）
    
    用于同步数据库会话环境
    """
    # 提取用户信息
    user_id = user.id if user else None
    username = user.username if user else None
    
    # 提取请求信息
    ip_address = None
    user_agent = None
    if request:
        if settings.TRUST_PROXY:
            forwarded = request.headers.get("X-Forwarded-For")
        else:
            forwarded = None
        if forwarded:
            ip_address = forwarded.split(",")[0].strip()
        else:
            ip_address = request.client.host if request.client else None
        user_agent = request.headers.get("User-Agent", "")[:255]
    
    # 创建日志记录
    log_entry = AuditLog(
        user_id=user_id,
        username=username,
        action=action,
        resource=resource,
        resource_id=resource_id,
        details=details,
        ip_address=ip_address,
        user_agent=user_agent,
        created_at=datetime.utcnow(),
    )
    
    db.add(log_entry)
    db.commit()
    db.refresh(log_entry)
    
    logger.info(
        f"Audit: {action} on {resource}/{resource_id} by {username or 'anonymous'}"
    )
    
    return log_entry
