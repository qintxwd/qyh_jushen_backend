"""
QYH Jushen Control Plane - 审计日志 Schema
"""
from datetime import datetime
from typing import Optional, Any
from enum import Enum

from pydantic import BaseModel, Field


class AuditAction(str, Enum):
    """审计操作类型"""
    # 认证相关
    LOGIN = "login"
    LOGOUT = "logout"
    TOKEN_REFRESH = "token_refresh"
    
    # 控制权相关
    CONTROL_ACQUIRE = "control_acquire"
    CONTROL_RELEASE = "control_release"
    CONTROL_FORCE_RELEASE = "control_force_release"
    
    # 模式切换
    MODE_SWITCH = "mode_switch"
    
    # 任务相关
    TASK_CREATE = "task_create"
    TASK_UPDATE = "task_update"
    TASK_DELETE = "task_delete"
    TASK_EXECUTE = "task_execute"
    
    # 预设相关
    PRESET_CREATE = "preset_create"
    PRESET_UPDATE = "preset_update"
    PRESET_DELETE = "preset_delete"
    PRESET_APPLY = "preset_apply"
    PRESET_CAPTURE = "preset_capture"
    
    # 录制相关
    RECORDING_START = "recording_start"
    RECORDING_STOP = "recording_stop"
    RECORDING_DISCARD = "recording_discard"
    
    # 动作相关
    ACTION_CREATE = "action_create"
    ACTION_UPDATE = "action_update"
    ACTION_DELETE = "action_delete"
    ACTION_TRAIN = "action_train"
    
    # 系统相关
    SYSTEM_SHUTDOWN = "system_shutdown"
    SYSTEM_REBOOT = "system_reboot"
    CONFIG_UPDATE = "config_update"
    
    # 其他
    OTHER = "other"


class AuditResource(str, Enum):
    """审计资源类型"""
    USER = "user"
    CONTROL = "control"
    MODE = "mode"
    TASK = "task"
    PRESET = "preset"
    RECORDING = "recording"
    ACTION = "action"
    SYSTEM = "system"
    CONFIG = "config"


# ==================== 日志记录 Schema ====================

class AuditLogEntry(BaseModel):
    """审计日志条目"""
    id: int
    user_id: Optional[int] = None
    username: Optional[str] = None
    action: str
    resource: Optional[str] = None
    resource_id: Optional[str] = None
    details: Optional[dict[str, Any]] = None
    ip_address: Optional[str] = None
    user_agent: Optional[str] = None
    created_at: datetime
    
    class Config:
        from_attributes = True


class CreateAuditLogRequest(BaseModel):
    """创建审计日志请求（内部使用）"""
    action: str = Field(..., description="操作类型")
    resource: Optional[str] = Field(None, description="资源类型")
    resource_id: Optional[str] = Field(None, description="资源ID")
    details: Optional[dict[str, Any]] = Field(None, description="详细信息")


# ==================== 查询 Schema ====================

class AuditLogQuery(BaseModel):
    """审计日志查询参数"""
    user_id: Optional[int] = Field(None, description="用户ID")
    username: Optional[str] = Field(None, description="用户名")
    action: Optional[str] = Field(None, description="操作类型")
    resource: Optional[str] = Field(None, description="资源类型")
    resource_id: Optional[str] = Field(None, description="资源ID")
    start_time: Optional[datetime] = Field(None, description="开始时间")
    end_time: Optional[datetime] = Field(None, description="结束时间")
    ip_address: Optional[str] = Field(None, description="IP地址")


class AuditLogListResponse(BaseModel):
    """审计日志列表响应"""
    items: list[AuditLogEntry]
    total: int
    page: int
    page_size: int
    total_pages: int


# ==================== 统计 Schema ====================

class AuditActionCount(BaseModel):
    """操作统计"""
    action: str
    count: int


class AuditUserCount(BaseModel):
    """用户操作统计"""
    user_id: Optional[int]
    username: Optional[str]
    count: int


class AuditDailyCount(BaseModel):
    """每日统计"""
    date: str
    count: int


class AuditStatistics(BaseModel):
    """审计统计信息"""
    total_logs: int
    today_logs: int
    by_action: list[AuditActionCount]
    by_user: list[AuditUserCount]
    recent_days: list[AuditDailyCount]
