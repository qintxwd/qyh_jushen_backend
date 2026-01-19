"""
QYH Jushen Control Plane - 预设模型
"""
from datetime import datetime
from typing import Optional

from sqlalchemy import String, Text, DateTime, Integer, ForeignKey, JSON
from sqlalchemy.orm import Mapped, mapped_column

from app.database import Base


class Preset(Base):
    """预设模型"""
    
    __tablename__ = "presets"
    
    id: Mapped[int] = mapped_column(Integer, primary_key=True, index=True)
    name: Mapped[str] = mapped_column(String(100), nullable=False)
    description: Mapped[Optional[str]] = mapped_column(Text, nullable=True)
    
    # 预设类型
    preset_type: Mapped[str] = mapped_column(String(50), nullable=False, index=True)
    # arm_position, chassis_position, gripper, full_pose, etc.
    
    # 预设数据
    data: Mapped[dict] = mapped_column(JSON, nullable=False)
    
    # 创建者
    creator_id: Mapped[Optional[int]] = mapped_column(
        Integer, ForeignKey("users.id"), nullable=True
    )
    
    # 是否为系统预设（不可删除）
    is_system: Mapped[bool] = mapped_column(default=False)
    
    # 时间戳
    created_at: Mapped[datetime] = mapped_column(DateTime, default=datetime.utcnow)
    updated_at: Mapped[datetime] = mapped_column(
        DateTime, default=datetime.utcnow, onupdate=datetime.utcnow
    )
    
    def __repr__(self):
        return f"<Preset {self.id}: {self.name}>"
