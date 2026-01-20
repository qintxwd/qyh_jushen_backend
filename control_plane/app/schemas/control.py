"""
QYH Jushen Control Plane - 控制权 Schema
"""
from typing import Optional
from datetime import datetime
from pydantic import BaseModel, Field


class AcquireControlRequest(BaseModel):
    """获取控制权请求"""
    duration: int = Field(default=300, ge=60, le=3600, description="持续时间（秒）")
    session_type: Optional[str] = Field(default="teleop", pattern="^(teleop|auto)$")
    client_type: Optional[str] = Field(None, description="Client type (alias for session_type)")
    force: bool = False
    reason: Optional[str] = None


class ControlHolder(BaseModel):
    """控制权持有者信息"""
    user_id: int
    username: str
    expires_at: Optional[str] = None
    session_type: Optional[str] = None
    remaining_seconds: int = 0


class ControlStatus(BaseModel):
    """控制权状态"""
    locked: bool = Field(..., description="是否被锁定")
    holder: Optional[ControlHolder] = Field(None, description="持有者信息")


class ForceReleaseRequest(BaseModel):
    """强制释放请求"""
    reason: str = Field(default="admin_force", description="释放原因")
    session_id: Optional[str] = Field(None, description="会话ID")
