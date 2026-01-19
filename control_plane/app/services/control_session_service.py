"""
QYH Jushen Control Plane - 控制权会话服务

提供控制权会话的数据库持久化功能
与 control_lock.py 配合使用，control_lock 负责内存互斥锁，本服务负责持久化
"""
import logging
from datetime import datetime
from typing import Optional, List

from sqlalchemy import desc, select
from sqlalchemy.ext.asyncio import AsyncSession

from app.models.control_session import ControlSession

logger = logging.getLogger(__name__)


class ControlSessionService:
    """
    控制权会话服务
    
    负责将控制权会话记录到数据库，支持历史查询
    """
    
    @staticmethod
    async def create_session(
        db: AsyncSession,
        user_id: int,
        username: str,
        session_type: str,
    ) -> ControlSession:
        """
        创建新的控制权会话
        
        Args:
            db: 数据库会话
            user_id: 用户 ID
            username: 用户名
            session_type: 会话类型 (teleop, auto)
        
        Returns:
            创建的会话记录
        """
        session = ControlSession(
            user_id=user_id,
            username=username,
            session_type=session_type,
            started_at=datetime.utcnow(),
        )
        db.add(session)
        await db.commit()
        await db.refresh(session)
        
        logger.info(f"Control session created: {session.id} for user {username}")
        return session
    
    @staticmethod
    async def end_session(
        db: AsyncSession,
        user_id: int,
        end_reason: str = "released",
    ) -> Optional[ControlSession]:
        """
        结束控制权会话
        
        Args:
            db: 数据库会话
            user_id: 用户 ID
            end_reason: 结束原因 (released, timeout, forced, error, mode_change)
        
        Returns:
            结束的会话记录，如果没有活跃会话则返回 None
        """
        # 查找该用户未结束的会话
        stmt = select(ControlSession).filter(
            ControlSession.user_id == user_id,
            ControlSession.ended_at.is_(None)
        ).order_by(desc(ControlSession.started_at))
        
        result = await db.execute(stmt)
        session = result.scalars().first()
        
        if session:
            session.ended_at = datetime.utcnow()
            session.end_reason = end_reason
            await db.commit()
            await db.refresh(session)
            
            duration = session.duration_seconds
            logger.info(
                f"Control session ended: {session.id}, reason={end_reason}, "
                f"duration={duration:.1f}s"
            )
            return session
        
        return None
    
    @staticmethod
    async def end_all_active_sessions(
        db: AsyncSession,
        end_reason: str = "forced",
    ) -> int:
        """
        结束所有活跃会话
        
        通常在系统重启或强制释放时使用
        
        Args:
            db: 数据库会话
            end_reason: 结束原因
        
        Returns:
            结束的会话数量
        """
        now = datetime.utcnow()
        
        # 查找所有未结束的会话
        stmt = select(ControlSession).filter(
            ControlSession.ended_at.is_(None)
        )
        result = await db.execute(stmt)
        active_sessions = result.scalars().all()
        
        count = 0
        for session in active_sessions:
            session.ended_at = now
            session.end_reason = end_reason
            count += 1
        
        if count > 0:
            await db.commit()
            logger.info(f"Ended {count} active control sessions, reason={end_reason}")
        
        return count
    
    @staticmethod
    async def get_active_session(
        db: AsyncSession,
        user_id: int,
    ) -> Optional[ControlSession]:
        """
        获取用户当前活跃的会话
        
        Args:
            db: 数据库会话
            user_id: 用户 ID
        
        Returns:
            活跃的会话记录，如果没有则返回 None
        """
        stmt = select(ControlSession).filter(
            ControlSession.user_id == user_id,
            ControlSession.ended_at.is_(None)
        ).order_by(desc(ControlSession.started_at))
        
        result = await db.execute(stmt)
        return result.scalars().first()
    
    @staticmethod
    async def get_history(
        db: AsyncSession,
        user_id: Optional[int] = None,
        limit: int = 50,
        offset: int = 0,
    ) -> List[ControlSession]:
        """
        获取控制权会话历史
        
        Args:
            db: 数据库会话
            user_id: 用户 ID（可选，不指定则返回所有用户）
            limit: 返回数量限制
            offset: 偏移量
        
        Returns:
            会话记录列表
        """
        stmt = select(ControlSession)
        
        if user_id is not None:
            stmt = stmt.filter(ControlSession.user_id == user_id)
        
        stmt = stmt.order_by(
            desc(ControlSession.started_at)
        ).offset(offset).limit(limit)
        
        result = await db.execute(stmt)
        return result.scalars().all()
    
    @staticmethod
    async def get_statistics(
        db: AsyncSession,
        days: int = 7,
    ) -> dict:
        """
        获取控制权会话统计
        
        Args:
            db: 数据库会话
            days: 统计天数
        
        Returns:
            统计信息字典
        """
        from datetime import timedelta
        from sqlalchemy import func
        
        cutoff = datetime.utcnow() - timedelta(days=days)
        
        # 总会话数
        stmt_total = select(func.count(ControlSession.id))
        result_total = await db.execute(stmt_total)
        total = result_total.scalar() or 0
        
        # 指定时间段内的会话数
        stmt_recent = select(func.count(ControlSession.id)).filter(
            ControlSession.started_at >= cutoff
        )
        result_recent = await db.execute(stmt_recent)
        recent = result_recent.scalar() or 0
        
        # 按用户统计
        stmt_by_user = select(
            ControlSession.username,
            func.count(ControlSession.id).label("count")
        ).filter(
            ControlSession.started_at >= cutoff
        ).group_by(ControlSession.username)
        result_by_user = await db.execute(stmt_by_user)
        by_user = result_by_user.all()
        
        # 按会话类型统计
        stmt_by_type = select(
            ControlSession.session_type,
            func.count(ControlSession.id).label("count")
        ).filter(
            ControlSession.started_at >= cutoff
        ).group_by(ControlSession.session_type)
        result_by_type = await db.execute(stmt_by_type)
        by_type = result_by_type.all()
        
        # 按结束原因统计
        stmt_by_reason = select(
            ControlSession.end_reason,
            func.count(ControlSession.id).label("count")
        ).filter(
            ControlSession.started_at >= cutoff,
            ControlSession.end_reason.isnot(None)
        ).group_by(ControlSession.end_reason)
        result_by_reason = await db.execute(stmt_by_reason)
        by_reason = result_by_reason.all()
        
        return {
            "total_sessions": total,
            "recent_sessions": recent,
            "period_days": days,
            "by_user": [{"username": u, "count": c} for u, c in by_user],
            "by_type": [{"type": t, "count": c} for t, c in by_type],
            "by_reason": [{"reason": r or "active", "count": c} for r, c in by_reason],
        }


# 便捷函数
async def create_control_session(
    db: AsyncSession,
    user_id: int,
    username: str,
    session_type: str,
) -> ControlSession:
    """创建控制权会话"""
    return await ControlSessionService.create_session(db, user_id, username, session_type)


async def end_control_session(
    db: AsyncSession,
    user_id: int,
    end_reason: str = "released",
) -> Optional[ControlSession]:
    """结束控制权会话"""
    return await ControlSessionService.end_session(db, user_id, end_reason)
