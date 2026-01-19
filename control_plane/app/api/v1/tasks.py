"""
QYH Jushen Control Plane - 任务管理 API

管理任务的创建、查询、启动、暂停、取消等生命周期
"""
import json
from datetime import datetime

from fastapi import APIRouter, Depends, status
from sqlalchemy.orm import Session

from app.database import get_db
from app.dependencies import get_current_operator, get_current_user
from app.models.user import User
from app.models.task import Task, TaskStatus
from app.core.mode_manager import mode_manager, RobotMode
from app.schemas.response import (
    ApiResponse, 
    success_response, 
    error_response, 
    paged_response,
    ErrorCodes,
)
from app.schemas.task import CreateTaskRequest, TaskInfo, TaskDetail
from app.services.ros2_client import get_ros2_client

router = APIRouter()


def task_to_info(task: Task) -> dict:
    """将 Task 模型转换为 TaskInfo"""
    return TaskInfo(
        task_id=task.id,
        name=task.name,
        description=task.description,
        status=task.status,
        current_step=task.current_step,
        total_steps=task.total_steps,
        progress=task.progress,
        created_at=task.created_at.isoformat() if task.created_at else None,
        started_at=task.started_at.isoformat() if task.started_at else None,
        completed_at=task.completed_at.isoformat() if task.completed_at else None,
        error_message=task.error_message,
    ).model_dump()


@router.post("", response_model=ApiResponse, status_code=status.HTTP_201_CREATED)
async def create_task(
    request: CreateTaskRequest,
    current_user: User = Depends(get_current_operator),
    db: Session = Depends(get_db),
):
    """
    创建新任务
    """
    task = Task(
        name=request.name,
        description=request.description,
        program=[action.model_dump() for action in request.program],
        total_steps=len(request.program),
        creator_id=current_user.id,
        status=TaskStatus.PENDING.value,
    )
    
    db.add(task)
    db.commit()
    db.refresh(task)
    
    return success_response(
        data=task_to_info(task),
        message="任务创建成功"
    )


@router.get("", response_model=ApiResponse)
async def list_tasks(
    status_filter: str = None,
    page: int = 1,
    page_size: int = 20,
    current_user: User = Depends(get_current_user),
    db: Session = Depends(get_db),
):
    """
    列出所有任务
    """
    query = db.query(Task)
    
    if status_filter:
        query = query.filter(Task.status == status_filter)
    
    total = query.count()
    tasks = (
        query
        .order_by(Task.created_at.desc())
        .offset((page - 1) * page_size)
        .limit(page_size)
        .all()
    )
    
    return paged_response(
        items=[task_to_info(task) for task in tasks],
        total=total,
        page=page,
        page_size=page_size,
    )


@router.get("/{task_id}", response_model=ApiResponse)
async def get_task(
    task_id: int,
    current_user: User = Depends(get_current_user),
    db: Session = Depends(get_db),
):
    """
    获取任务详情
    """
    task = db.query(Task).filter(Task.id == task_id).first()
    
    if not task:
        return error_response(
            code=ErrorCodes.TASK_NOT_FOUND,
            message="任务不存在"
        )
    
    detail = TaskDetail(
        task_id=task.id,
        name=task.name,
        description=task.description,
        status=task.status,
        current_step=task.current_step,
        total_steps=task.total_steps,
        progress=task.progress,
        created_at=task.created_at.isoformat() if task.created_at else None,
        started_at=task.started_at.isoformat() if task.started_at else None,
        completed_at=task.completed_at.isoformat() if task.completed_at else None,
        error_message=task.error_message,
        program=task.program,
        creator_id=task.creator_id,
    )
    
    return success_response(
        data=detail.model_dump(),
        message="获取成功"
    )


@router.post("/{task_id}/start", response_model=ApiResponse)
async def start_task(
    task_id: int,
    current_user: User = Depends(get_current_operator),
    db: Session = Depends(get_db),
):
    """
    启动任务
    """
    task = db.query(Task).filter(Task.id == task_id).first()
    
    if not task:
        return error_response(
            code=ErrorCodes.TASK_NOT_FOUND,
            message="任务不存在"
        )
    
    if task.status not in (TaskStatus.PENDING.value, TaskStatus.PAUSED.value):
        return error_response(
            code=ErrorCodes.TASK_INVALID_STATE,
            message=f"任务状态为 {task.status}，无法启动"
        )
    
    # 检查是否有其他任务在运行
    running_task = db.query(Task).filter(Task.status == TaskStatus.RUNNING.value).first()
    if running_task:
        return error_response(
            code=ErrorCodes.TASK_ALREADY_RUNNING,
            message=f"任务 {running_task.name} 正在运行中"
        )
    
    # 切换到 auto 模式
    success, msg = mode_manager.switch_to(RobotMode.AUTO)
    if not success:
        return error_response(
            code=ErrorCodes.MODE_INVALID_TRANSITION,
            message=f"无法切换到自主模式: {msg}"
        )
    
    # 更新任务状态
    task.status = TaskStatus.RUNNING.value
    task.started_at = datetime.utcnow()
    db.commit()
    
    # 通知 ROS2 启动任务
    ros2_client = get_ros2_client()
    if not ros2_client._initialized:
        await ros2_client.initialize()
    
    task_json = json.dumps(task.program) if task.program else "{}"
    ros2_result = await ros2_client.execute_task(task_json, debug_mode=False)
    
    if not ros2_result.success:
        # ROS2 调用失败，回滚状态
        task.status = TaskStatus.PENDING.value
        task.started_at = None
        db.commit()
        mode_manager.switch_to(RobotMode.IDLE, force=True)
        return error_response(
            code=ErrorCodes.INTERNAL_ERROR,
            message=f"ROS2 任务启动失败: {ros2_result.message}"
        )
    
    return success_response(
        data=task_to_info(task),
        message="任务已启动"
    )


@router.post("/{task_id}/pause", response_model=ApiResponse)
async def pause_task(
    task_id: int,
    current_user: User = Depends(get_current_operator),
    db: Session = Depends(get_db),
):
    """
    暂停任务
    """
    task = db.query(Task).filter(Task.id == task_id).first()
    
    if not task:
        return error_response(
            code=ErrorCodes.TASK_NOT_FOUND,
            message="任务不存在"
        )
    
    if task.status != TaskStatus.RUNNING.value:
        return error_response(
            code=ErrorCodes.TASK_NOT_RUNNING,
            message="任务未在运行中"
        )
    
    task.status = TaskStatus.PAUSED.value
    db.commit()
    
    # 通知 ROS2 暂停任务
    ros2_client = get_ros2_client()
    if not ros2_client._initialized:
        await ros2_client.initialize()
    await ros2_client.pause_task(str(task.id))
    
    return success_response(
        data=task_to_info(task),
        message="任务已暂停"
    )


@router.post("/{task_id}/resume", response_model=ApiResponse)
async def resume_task(
    task_id: int,
    current_user: User = Depends(get_current_operator),
    db: Session = Depends(get_db),
):
    """
    恢复任务
    """
    task = db.query(Task).filter(Task.id == task_id).first()
    
    if not task:
        return error_response(
            code=ErrorCodes.TASK_NOT_FOUND,
            message="任务不存在"
        )
    
    if task.status != TaskStatus.PAUSED.value:
        return error_response(
            code=ErrorCodes.TASK_INVALID_STATE,
            message="任务未暂停"
        )
    
    task.status = TaskStatus.RUNNING.value
    db.commit()
    
    # 通知 ROS2 恢复任务
    ros2_client = get_ros2_client()
    if not ros2_client._initialized:
        await ros2_client.initialize()
    await ros2_client.resume_task(str(task.id))
    
    return success_response(
        data=task_to_info(task),
        message="任务已恢复"
    )


@router.post("/{task_id}/cancel", response_model=ApiResponse)
async def cancel_task(
    task_id: int,
    current_user: User = Depends(get_current_operator),
    db: Session = Depends(get_db),
):
    """
    取消任务
    """
    task = db.query(Task).filter(Task.id == task_id).first()
    
    if not task:
        return error_response(
            code=ErrorCodes.TASK_NOT_FOUND,
            message="任务不存在"
        )
    
    if task.status not in (TaskStatus.PENDING.value, TaskStatus.RUNNING.value, TaskStatus.PAUSED.value):
        return error_response(
            code=ErrorCodes.TASK_INVALID_STATE,
            message=f"任务状态为 {task.status}，无法取消"
        )
    
    task.status = TaskStatus.CANCELLED.value
    task.completed_at = datetime.utcnow()
    db.commit()
    
    # 切换回 idle 模式
    mode_manager.switch_to(RobotMode.IDLE, force=True)
    
    # 通知 ROS2 取消任务
    ros2_client = get_ros2_client()
    if not ros2_client._initialized:
        await ros2_client.initialize()
    await ros2_client.cancel_task(str(task.id))
    
    return success_response(
        data=task_to_info(task),
        message="任务已取消"
    )


@router.delete("/{task_id}", response_model=ApiResponse)
async def delete_task(
    task_id: int,
    current_user: User = Depends(get_current_operator),
    db: Session = Depends(get_db),
):
    """
    删除任务
    """
    task = db.query(Task).filter(Task.id == task_id).first()
    
    if not task:
        return error_response(
            code=ErrorCodes.TASK_NOT_FOUND,
            message="任务不存在"
        )
    
    if task.status == TaskStatus.RUNNING.value:
        return error_response(
            code=ErrorCodes.TASK_ALREADY_RUNNING,
            message="运行中的任务无法删除"
        )
    
    db.delete(task)
    db.commit()
    
    return success_response(message="任务已删除")
