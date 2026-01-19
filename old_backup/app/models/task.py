"""任务模型"""
from sqlalchemy import Column, Integer, String, Text, DateTime, ForeignKey, JSON, Enum, Float
from datetime import datetime
from app.database import Base
import enum


class TaskStatus(str, enum.Enum):
    """任务状态"""
    PENDING = "pending"
    RUNNING = "running"
    COMPLETED = "completed"
    FAILED = "failed"
    CANCELLED = "cancelled"


class Task(Base):
    """任务表"""
    __tablename__ = "tasks"
    
    id = Column(Integer, primary_key=True, index=True)
    name = Column(String(200), nullable=False)
    description = Column(Text, nullable=True)
    program = Column(JSON, nullable=False)  # Blockly 生成的指令序列
    status = Column(Enum(TaskStatus), default=TaskStatus.PENDING)
    creator_id = Column(Integer, ForeignKey("users.id"))
    created_at = Column(DateTime, default=datetime.utcnow)
    started_at = Column(DateTime, nullable=True)
    completed_at = Column(DateTime, nullable=True)
    current_step = Column(Integer, default=0)
    total_steps = Column(Integer, nullable=False)
    progress = Column(Float, default=0.0)
    error_message = Column(Text, nullable=True)
