"""Task management API (file-backed)."""
from __future__ import annotations

import json
import uuid
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, List, Optional

from fastapi import APIRouter, Depends, HTTPException
from pydantic import BaseModel

from app.core.control_lock import control_lock
from app.core.paths import get_repo_root
from app.dependencies import get_current_operator, get_current_user

router = APIRouter(prefix="/api/v1/tasks", tags=["tasks"])

TASKS_DIR = get_repo_root() / "persistent" / "tasks"
TASKS_DIR.mkdir(parents=True, exist_ok=True)


class CreateTaskRequest(BaseModel):
    name: str
    description: str | None = ""
    program: List[Dict[str, Any]] = []


class TaskItem(BaseModel):
    task_id: str
    name: str
    status: str
    created_at: Optional[str] = None
    started_at: Optional[str] = None
    completed_at: Optional[str] = None
    current_step: int = 0
    total_steps: int = 0
    progress: float = 0.0


def _task_path(task_id: str) -> Path:
    return TASKS_DIR / f"{task_id}.json"


def _load_task(task_id: str) -> Dict[str, Any]:
    file_path = _task_path(task_id)
    if not file_path.exists():
        raise HTTPException(status_code=404, detail="Task not found")
    with open(file_path, "r", encoding="utf-8") as handle:
        return json.load(handle)


def _save_task(task_id: str, data: Dict[str, Any]) -> Dict[str, Any]:
    with open(_task_path(task_id), "w", encoding="utf-8") as handle:
        json.dump(data, handle, ensure_ascii=False, indent=2)
    return data


@router.post("")
async def create_task(
    request: CreateTaskRequest,
    current_user=Depends(get_current_operator),
):
    task_id = str(uuid.uuid4())[:8]
    now = datetime.now().isoformat()
    task_data = {
        "id": task_id,
        "name": request.name,
        "description": request.description,
        "program": request.program,
        "status": "pending",
        "created_at": now,
        "updated_at": now,
        "current_step": 0,
        "total_steps": len(request.program),
        "progress": 0.0,
        "started_at": None,
        "completed_at": None,
    }
    _save_task(task_id, task_data)
    return {"task_id": task_id, "status": "pending"}


@router.get("")
async def list_tasks(
    status_filter: str | None = None,
    current_user=Depends(get_current_user),
):
    items: List[TaskItem] = []
    for file_path in TASKS_DIR.glob("*.json"):
        try:
            with open(file_path, "r", encoding="utf-8") as handle:
                data = json.load(handle)
            if status_filter and data.get("status") != status_filter:
                continue
            items.append(
                TaskItem(
                    task_id=data.get("id", file_path.stem),
                    name=data.get("name", "Unnamed"),
                    status=data.get("status", "pending"),
                    created_at=data.get("created_at"),
                    started_at=data.get("started_at"),
                    completed_at=data.get("completed_at"),
                    current_step=data.get("current_step", 0),
                    total_steps=data.get("total_steps", 0),
                    progress=data.get("progress", 0.0),
                )
            )
        except Exception:
            continue
    return {
        "items": [item.model_dump() for item in items],
        "total": len(items),
    }


@router.get("/{task_id}")
async def get_task(task_id: str, current_user=Depends(get_current_user)):
    data = _load_task(task_id)
    return data


@router.post("/{task_id}/start")
async def start_task(task_id: str, current_user=Depends(get_current_operator)):
    if not control_lock.is_held_by(current_user.id):
        raise HTTPException(status_code=403, detail="Control not held")
    data = _load_task(task_id)
    if data.get("status") == "running":
        return {"ok": True}
    data["status"] = "running"
    data["started_at"] = datetime.now().isoformat()
    data["updated_at"] = data["started_at"]
    _save_task(task_id, data)
    return {"ok": True}


@router.post("/{task_id}/cancel")
async def cancel_task(
    task_id: str,
    current_user=Depends(get_current_operator),
):
    data = _load_task(task_id)
    if data.get("status") != "running":
        raise HTTPException(status_code=400, detail="Task not running")
    data["status"] = "cancelled"
    data["completed_at"] = datetime.now().isoformat()
    data["updated_at"] = data["completed_at"]
    _save_task(task_id, data)
    return {"ok": True}
