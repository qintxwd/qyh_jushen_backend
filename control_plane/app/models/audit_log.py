"""
QYH Jushen Control Plane - 审计日志模型
"""
from datetime import datetime
from typing import Optional

from sqlalchemy import String, Text, DateTime, Integer, ForeignKey, JSON
from sqlalchemy.orm import Mapped, mapped_column

from app.database import Base


class AuditLog(Base):
    """审计日志模型"""
    
    __tablename__ = "audit_logs"
    
    id: Mapped[int] = mapped_column(Integer, primary_key=True, index=True)
    
    # 用户信息
    user_id: Mapped[Optional[int]] = mapped_column(
        Integer, ForeignKey("users.id"), nullable=True
    )
    username: Mapped[Optional[str]] = mapped_column(String(50), nullable=True)
    
    # 操作信息
    action: Mapped[str] = mapped_column(String(50), nullable=False, index=True)
    resource: Mapped[Optional[str]] = mapped_column(String(50), nullable=True)
    resource_id: Mapped[Optional[str]] = mapped_column(String(50), nullable=True)
    
    # 详细信息
    details: Mapped[Optional[dict]] = mapped_column(JSON, nullable=True)
    
    # 请求信息
    ip_address: Mapped[Optional[str]] = mapped_column(String(50), nullable=True)
    user_agent: Mapped[Optional[str]] = mapped_column(String(255), nullable=True)
    
    # 时间戳
    created_at: Mapped[datetime] = mapped_column(
        DateTime, default=datetime.utcnow, index=True
    )
    
    def __repr__(self):
        return f"<AuditLog {self.id}: {self.action}>"
