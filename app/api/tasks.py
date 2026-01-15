"""任务管理 API"""
from fastapi import APIRouter, Depends, status
from sqlalchemy.orm import Session
from app.database import get_db
from app.dependencies import get_current_operator, get_current_user
from app.models.user import User
from app.models.task import Task, TaskStatus
from app.schemas.task import CreateTaskRequest
from app.schemas.response import (
    ApiResponse, success_response, error_response, ErrorCodes
)
from app.core.task_runner import task_runner
from app.core.control_lock import control_lock

router = APIRouter()


@router.post("", response_model=ApiResponse, status_code=status.HTTP_201_CREATED)
async def create_task(
    request: CreateTaskRequest,
    current_user: User = Depends(get_current_operator),
    db: Session = Depends(get_db)
):
    """创建新任务
    
    Returns:
        ApiResponse: 统一响应格式，data 包含任务信息
    """
    task = Task(
        name=request.name,
        description=request.description,
        program=[action.dict() for action in request.program],
        total_steps=len(request.program),
        creator_id=current_user.id,
        status=TaskStatus.PENDING
    )
    
    db.add(task)
    db.commit()
    db.refresh(task)
    
    return success_response(
        data={
            "task_id": task.id,
            "name": task.name,
            "status": task.status,
            "created_at": task.created_at.isoformat() if task.created_at else None,
            "started_at": task.started_at.isoformat() if task.started_at else None,
            "completed_at": task.completed_at.isoformat() if task.completed_at else None,
            "current_step": task.current_step,
            "total_steps": task.total_steps,
            "progress": task.progress
        },
        message="任务创建成功"
    )


@router.get("", response_model=ApiResponse)
async def list_tasks(
    status_filter: str = None,
    current_user: User = Depends(get_current_user),
    db: Session = Depends(get_db)
):
    """列出所有任务
    
    Args:
        status_filter: 状态过滤 (pending, running, completed, failed, cancelled)
    
    Returns:
        ApiResponse: 统一响应格式，data 包含任务列表
    """
    query = db.query(Task)
    
    if status_filter:
        query = query.filter(Task.status == status_filter)
    
    tasks = query.order_by(Task.created_at.desc()).limit(100).all()
    
    return success_response(
        data={
            "items": [
                {
                    "task_id": task.id,
                    "name": task.name,
                    "status": task.status,
                    "created_at": task.created_at.isoformat() if task.created_at else None,
                    "started_at": task.started_at.isoformat() if task.started_at else None,
                    "completed_at": task.completed_at.isoformat() if task.completed_at else None,
                    "current_step": task.current_step,
                    "total_steps": task.total_steps,
                    "progress": task.progress
                }
                for task in tasks
            ],
            "total": len(tasks)
        }
    )


@router.get("/{task_id}", response_model=ApiResponse)
async def get_task(
    task_id: int,
    current_user: User = Depends(get_current_user),
    db: Session = Depends(get_db)
):
    """获取任务详情
    
    Returns:
        ApiResponse: 统一响应格式，data 包含任务详情
    """
    task = db.query(Task).filter(Task.id == task_id).first()
    
    if not task:
        return error_response(
            code=ErrorCodes.TASK_NOT_FOUND,
            message="任务不存在"
        )
    
    return success_response(
        data={
            "task_id": task.id,
            "name": task.name,
            "description": task.description,
            "status": task.status,
            "program": task.program,
            "created_at": task.created_at.isoformat() if task.created_at else None,
            "started_at": task.started_at.isoformat() if task.started_at else None,
            "completed_at": task.completed_at.isoformat() if task.completed_at else None,
            "current_step": task.current_step,
            "total_steps": task.total_steps,
            "progress": task.progress,
            "error_message": task.error_message
        }
    )


@router.post("/{task_id}/start", response_model=ApiResponse)
async def start_task(
    task_id: int,
    current_user: User = Depends(get_current_operator),
    db: Session = Depends(get_db)
):
    """启动任务
    
    Returns:
        ApiResponse: 统一响应格式
    """
    # 检查控制权
    if not control_lock.is_held_by(current_user.id):
        return error_response(
            code=ErrorCodes.CONTROL_NOT_HELD,
            message="您未持有控制权，无法启动任务"
        )
    
    # 启动任务
    try:
        task_runner.start_task(task_id, db)
        return success_response(message="任务已启动")
    except Exception as e:
        return error_response(
            code=ErrorCodes.TASK_START_FAILED,
            message=str(e)
        )


@router.post("/{task_id}/cancel", response_model=ApiResponse)
async def cancel_task(
    task_id: int,
    current_user: User = Depends(get_current_operator),
    db: Session = Depends(get_db)
):
    """取消任务
    
    Returns:
        ApiResponse: 统一响应格式
    """
    task = db.query(Task).filter(Task.id == task_id).first()
    
    if not task:
        return error_response(
            code=ErrorCodes.TASK_NOT_FOUND,
            message="任务不存在"
        )
    
    if task.status != TaskStatus.RUNNING:
        return error_response(
            code=ErrorCodes.TASK_INVALID_STATUS,
            message="只能取消正在运行的任务"
        )
    
    task_runner.stop_task()
    
    return success_response(message="任务已取消")
