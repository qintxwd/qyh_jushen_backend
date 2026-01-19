"""
QYH Jushen Control Plane - 控制权会话模型
"""
from datetime import datetime
from typing import Optional

from sqlalchemy import String, DateTime, Integer, ForeignKey
from sqlalchemy.orm import Mapped, mapped_column

from app.database import Base


class ControlSession(Base):
    """控制权会话模型"""
    
    __tablename__ = "control_sessions"
    
    id: Mapped[int] = mapped_column(Integer, primary_key=True, index=True)
    
    # 用户信息
    user_id: Mapped[int] = mapped_column(Integer, ForeignKey("users.id"), nullable=False)
    username: Mapped[str] = mapped_column(String(50), nullable=False)
    
    # 会话类型
    session_type: Mapped[str] = mapped_column(String(20), nullable=False)  # teleop, auto
    
    # 时间戳
    started_at: Mapped[datetime] = mapped_column(DateTime, default=datetime.utcnow)
    ended_at: Mapped[Optional[datetime]] = mapped_column(DateTime, nullable=True)
    
    # 结束原因
    end_reason: Mapped[Optional[str]] = mapped_column(String(50), nullable=True)
    # released, timeout, forced, error, mode_change
    
    # 持续时间（秒）
    @property
    def duration_seconds(self) -> Optional[float]:
        if self.ended_at:
            return (self.ended_at - self.started_at).total_seconds()
        return None
    
    def __repr__(self):
        return f"<ControlSession {self.id}: {self.username}>"
