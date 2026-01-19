"""
QYH Jushen Control Plane - 任务模型
"""
from datetime import datetime
from typing import Optional
from enum import Enum

from sqlalchemy import String, Text, DateTime, Integer, Float, ForeignKey, JSON
from sqlalchemy.orm import Mapped, mapped_column, relationship

from app.database import Base


class TaskStatus(str, Enum):
    """任务状态"""
    PENDING = "pending"
    RUNNING = "running"
    PAUSED = "paused"
    COMPLETED = "completed"
    FAILED = "failed"
    CANCELLED = "cancelled"


class Task(Base):
    """任务模型"""
    
    __tablename__ = "tasks"
    
    id: Mapped[int] = mapped_column(Integer, primary_key=True, index=True)
    name: Mapped[str] = mapped_column(String(100), nullable=False)
    description: Mapped[Optional[str]] = mapped_column(Text, nullable=True)
    
    # 任务程序（动作序列）
    program: Mapped[Optional[dict]] = mapped_column(JSON, nullable=True)
    
    # 状态
    status: Mapped[str] = mapped_column(String(20), default=TaskStatus.PENDING.value)
    
    # 进度
    current_step: Mapped[int] = mapped_column(Integer, default=0)
    total_steps: Mapped[int] = mapped_column(Integer, default=0)
    
    # 关联用户
    creator_id: Mapped[Optional[int]] = mapped_column(
        Integer, ForeignKey("users.id"), nullable=True
    )
    
    # 时间戳
    created_at: Mapped[datetime] = mapped_column(DateTime, default=datetime.utcnow)
    started_at: Mapped[Optional[datetime]] = mapped_column(DateTime, nullable=True)
    completed_at: Mapped[Optional[datetime]] = mapped_column(DateTime, nullable=True)
    
    # 错误信息
    error_message: Mapped[Optional[str]] = mapped_column(Text, nullable=True)
    
    @property
    def progress(self) -> float:
        """计算进度百分比"""
        if self.total_steps == 0:
            return 0.0
        return round(self.current_step / self.total_steps * 100, 2)
    
    def __repr__(self):
        return f"<Task {self.id}: {self.name}>"
