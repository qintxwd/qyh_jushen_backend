"""
QYH Jushen Control Plane - 任务 Schema
"""
from typing import Optional, List, Any, Dict
from datetime import datetime
from pydantic import BaseModel, Field


class TaskAction(BaseModel):
    """任务动作"""
    type: str = Field(..., description="动作类型")
    params: Dict[str, Any] = Field(default_factory=dict, description="动作参数")
    timeout: Optional[float] = Field(None, description="超时时间（秒）")


class CreateTaskRequest(BaseModel):
    """创建任务请求"""
    name: str = Field(..., min_length=1, max_length=100, description="任务名称")
    description: Optional[str] = Field(None, description="任务描述")
    program: List[TaskAction] = Field(..., min_length=1, description="任务程序（动作序列）")


class UpdateTaskRequest(BaseModel):
    """更新任务请求"""
    name: Optional[str] = Field(None, min_length=1, max_length=100, description="任务名称")
    description: Optional[str] = Field(None, description="任务描述")
    program: Optional[List[TaskAction]] = Field(None, min_length=1, description="任务程序（动作序列）")


class TaskInfo(BaseModel):
    """任务信息"""
    task_id: int
    name: str
    description: Optional[str] = None
    status: str
    current_step: int
    total_steps: int
    progress: float
    created_at: Optional[str] = None
    started_at: Optional[str] = None
    completed_at: Optional[str] = None
    error_message: Optional[str] = None


class TaskDetail(TaskInfo):
    """任务详情（包含程序）"""
    program: Optional[List[TaskAction]] = None
    creator_id: Optional[int] = None
