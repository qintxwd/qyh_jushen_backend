"""任务相关 Schema"""
from pydantic import BaseModel
from typing import List, Dict, Any, Optional
from datetime import datetime


class TaskAction(BaseModel):
    """任务动作"""
    type: str  # "move_to", "gripper_close", etc.
    params: Dict[str, Any]


class CreateTaskRequest(BaseModel):
    """创建任务请求"""
    name: str
    description: Optional[str] = None
    program: List[TaskAction]


class TaskResponse(BaseModel):
    """任务响应"""
    task_id: int
    name: str
    status: str
    created_at: datetime
    started_at: Optional[datetime] = None
    completed_at: Optional[datetime] = None
    current_step: int
    total_steps: int
    progress: float
    
    class Config:
        from_attributes = True


class TaskDetailResponse(TaskResponse):
    """任务详情响应"""
    description: Optional[str] = None
    program: List[Dict[str, Any]]
    error_message: Optional[str] = None
