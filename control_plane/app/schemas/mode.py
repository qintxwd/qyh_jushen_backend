"""
QYH Jushen Control Plane - 模式 Schema
"""
from typing import Optional, List
from pydantic import BaseModel, Field


class ModeSwitchRequest(BaseModel):
    """模式切换请求"""
    target_mode: str = Field(
        ..., 
        pattern="^(idle|teleop|auto|maintenance)$",
        description="目标模式"
    )
    force: bool = Field(default=False, description="是否强制切换")


class ModeInfo(BaseModel):
    """模式信息"""
    name: str
    display_name: str
    description: str
    allowed_operations: List[str]
    is_current: bool
    can_switch_to: bool


class ModeStatus(BaseModel):
    """当前模式状态"""
    current_mode: str
    previous_mode: Optional[str] = None
    mode_changed_at: str
    available_transitions: List[str]
