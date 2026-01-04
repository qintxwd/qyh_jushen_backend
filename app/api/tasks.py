"""任务管理 API"""
from fastapi import APIRouter, Depends, HTTPException, status
from sqlalchemy.orm import Session
from typing import List
from app.database import get_db
from app.dependencies import get_current_operator, get_current_user
from app.models.user import User
from app.models.task import Task, TaskStatus
from app.schemas.task import CreateTaskRequest, TaskResponse, TaskDetailResponse
from app.core.task_runner import task_runner
from app.core.control_lock import control_lock

router = APIRouter()


@router.post("", response_model=TaskResponse, status_code=status.HTTP_201_CREATED)
async def create_task(
    request: CreateTaskRequest,
    current_user: User = Depends(get_current_operator),
    db: Session = Depends(get_db)
):
    """创建新任务"""
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
    
    return {
        "task_id": task.id,
        "name": task.name,
        "status": task.status,
        "created_at": task.created_at,
        "started_at": task.started_at,
        "completed_at": task.completed_at,
        "current_step": task.current_step,
        "total_steps": task.total_steps,
        "progress": task.progress
    }


@router.get("", response_model=List[TaskResponse])
async def list_tasks(
    status: str = None,
    current_user: User = Depends(get_current_user),
    db: Session = Depends(get_db)
):
    """列出所有任务"""
    query = db.query(Task)
    
    if status:
        query = query.filter(Task.status == status)
    
    tasks = query.order_by(Task.created_at.desc()).limit(100).all()
    
    return [
        {
            "task_id": task.id,
            "name": task.name,
            "status": task.status,
            "created_at": task.created_at,
            "started_at": task.started_at,
            "completed_at": task.completed_at,
            "current_step": task.current_step,
            "total_steps": task.total_steps,
            "progress": task.progress
        }
        for task in tasks
    ]


@router.get("/{task_id}", response_model=TaskDetailResponse)
async def get_task(
    task_id: int,
    current_user: User = Depends(get_current_user),
    db: Session = Depends(get_db)
):
    """获取任务详情"""
    task = db.query(Task).filter(Task.id == task_id).first()
    
    if not task:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail="任务不存在"
        )
    
    return {
        "task_id": task.id,
        "name": task.name,
        "description": task.description,
        "status": task.status,
        "program": task.program,
        "created_at": task.created_at,
        "started_at": task.started_at,
        "completed_at": task.completed_at,
        "current_step": task.current_step,
        "total_steps": task.total_steps,
        "progress": task.progress,
        "error_message": task.error_message
    }


@router.post("/{task_id}/start")
async def start_task(
    task_id: int,
    current_user: User = Depends(get_current_operator),
    db: Session = Depends(get_db)
):
    """启动任务"""
    # 检查控制权
    if not control_lock.is_held_by(current_user.id):
        raise HTTPException(
            status_code=status.HTTP_403_FORBIDDEN,
            detail="您未持有控制权，无法启动任务"
        )
    
    # 启动任务
    try:
        task_runner.start_task(task_id, db)
        return {"message": "任务已启动"}
    except Exception as e:
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail=str(e)
        )


@router.post("/{task_id}/cancel")
async def cancel_task(
    task_id: int,
    current_user: User = Depends(get_current_operator),
    db: Session = Depends(get_db)
):
    """取消任务"""
    task = db.query(Task).filter(Task.id == task_id).first()
    
    if not task:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail="任务不存在"
        )
    
    if task.status != TaskStatus.RUNNING:
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail="只能取消正在运行的任务"
        )
    
    task_runner.stop_task()
    
    return {"message": "任务已取消"}
