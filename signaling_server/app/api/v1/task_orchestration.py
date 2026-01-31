"""Task orchestration API (file-based + mock execution)."""
from __future__ import annotations

import asyncio
import copy
import json
import uuid
from datetime import datetime
from typing import Any, Dict, List, Optional

from fastapi import APIRouter, Depends, HTTPException
from pydantic import BaseModel, Field

from app.core.paths import get_repo_root
from app.dependencies import get_current_admin
from app.safety.watchdog import watchdog

router = APIRouter(prefix="/api/v1", tags=["task-orchestration"])

TASKS_DIR = get_repo_root() / "persistent" / "tasks"
TASKS_DIR.mkdir(parents=True, exist_ok=True)


class TaskNode(BaseModel):
    id: str
    type: str
    params: Dict[str, Any] = {}
    children: List["TaskNode"] = []


TaskNode.model_rebuild()


class TaskDefinition(BaseModel):
    id: Optional[str] = None
    name: str
    description: Optional[str] = ""
    category: Optional[str] = "default"
    root: TaskNode
    created_at: Optional[str] = None
    updated_at: Optional[str] = None


class TaskListItem(BaseModel):
    id: str
    name: str
    description: Optional[str] = ""
    category: Optional[str] = "default"
    created_at: Optional[str] = None
    updated_at: Optional[str] = None


class TaskExecuteRequest(BaseModel):
    task_json: Dict[str, Any] | None = Field(default=None)
    debug_mode: bool = Field(default=False)
    id: Optional[str] = None
    name: Optional[str] = None
    root: Optional[Dict[str, Any]] = None


class TaskResponse(BaseModel):
    success: bool
    task_id: Optional[str] = None
    message: str


class TaskStatusResponse(BaseModel):
    task_id: Optional[str] = None
    task_name: Optional[str] = None
    status: str = "idle"
    current_node_id: Optional[str] = None
    completed_nodes: int = 0
    total_nodes: int = 0
    progress: float = 0.0
    elapsed_time: float = 0.0
    node_statuses: List[Dict[str, Any]] = []


_mock_task_status: Dict[str, Any] = {
    "task_id": None,
    "task_name": None,
    "status": "idle",
    "current_node_id": None,
    "completed_nodes": 0,
    "total_nodes": 0,
    "progress": 0.0,
    "elapsed_time": 0.0,
    "node_statuses": [],
}

_mock_execution_task: Optional[asyncio.Task] = None


def _get_status_snapshot() -> Dict[str, Any]:
    return {
        "task_id": _mock_task_status["task_id"],
        "task_name": _mock_task_status["task_name"],
        "status": _mock_task_status["status"],
        "current_node_id": _mock_task_status["current_node_id"],
        "completed_nodes": _mock_task_status["completed_nodes"],
        "total_nodes": _mock_task_status["total_nodes"],
        "progress": _mock_task_status["progress"],
        "elapsed_time": _mock_task_status["elapsed_time"],
        "node_statuses": copy.deepcopy(_mock_task_status["node_statuses"]),
    }


def _collect_nodes(
    node: Dict[str, Any],
    nodes: List[Dict[str, Any]] | None = None,
) -> List[Dict[str, Any]]:
    if nodes is None:
        nodes = []
    if not node:
        return nodes
    node_id = node.get("id", f"node_{len(nodes)}")
    node_type = node.get("type", "Unknown")
    node_name = node.get("name", node_type)
    children = node.get("children", [])
    params = node.get("params", {})
    node_status = {
        "node_id": node_id,
        "node_type": node_type,
        "node_name": node_name,
        "status": "idle",
        "message": "",
        "duration": 0.0,
        "children_count": len(children),
        "current_child_index": 0,
        "current_iteration": 0,
        "params": params,
    }
    nodes.append(node_status)
    for child in children:
        _collect_nodes(child, nodes)
    return nodes


def _load_task_file(task_id: str) -> Dict[str, Any]:
    file_path = TASKS_DIR / f"{task_id}.json"
    if not file_path.exists():
        raise HTTPException(status_code=404, detail="Task not found")
    with open(file_path, "r", encoding="utf-8") as handle:
        return json.load(handle)


def _save_task_file(task_id: str, data: Dict[str, Any]) -> Dict[str, Any]:
    file_path = TASKS_DIR / f"{task_id}.json"
    with open(file_path, "w", encoding="utf-8") as handle:
        json.dump(data, handle, ensure_ascii=False, indent=2)
    return data


@router.get("/task/list", response_model=List[TaskListItem])
async def list_tasks(
    category: Optional[str] = None,
    current_user=Depends(get_current_admin),
):
    tasks: List[TaskListItem] = []
    for file_path in TASKS_DIR.glob("*.json"):
        try:
            with open(file_path, "r", encoding="utf-8") as handle:
                task_data = json.load(handle)
            if category and task_data.get("category") != category:
                continue
            tasks.append(
                TaskListItem(
                    id=task_data.get("id", file_path.stem),
                    name=task_data.get("name", "Unnamed"),
                    description=task_data.get("description", ""),
                    category=task_data.get("category", "default"),
                    created_at=task_data.get("created_at"),
                    updated_at=task_data.get("updated_at"),
                )
            )
        except Exception:
            continue
    tasks.sort(key=lambda item: item.updated_at or "", reverse=True)
    return tasks


@router.get("/task/{task_id}", response_model=TaskDefinition)
async def get_task(task_id: str, current_user=Depends(get_current_admin)):
    data = _load_task_file(task_id)
    return TaskDefinition(**data)


@router.post("/task/create", response_model=TaskDefinition)
async def create_task(
    task: TaskDefinition,
    current_user=Depends(get_current_admin),
):
    task_id = task.id or str(uuid.uuid4())[:8]
    now = datetime.now().isoformat()
    task_data = task.model_dump()
    task_data["id"] = task_id
    task_data["created_at"] = now
    task_data["updated_at"] = now
    _save_task_file(task_id, task_data)
    return TaskDefinition(**task_data)


@router.put("/task/{task_id}", response_model=TaskDefinition)
async def update_task(
    task_id: str,
    task: TaskDefinition,
    current_user=Depends(get_current_admin),
):
    file_path = TASKS_DIR / f"{task_id}.json"
    if not file_path.exists():
        raise HTTPException(status_code=404, detail="Task not found")
    with open(file_path, "r", encoding="utf-8") as handle:
        old_data = json.load(handle)
    task_data = task.model_dump()
    task_data["id"] = task_id
    task_data["created_at"] = old_data.get("created_at")
    task_data["updated_at"] = datetime.now().isoformat()
    _save_task_file(task_id, task_data)
    return TaskDefinition(**task_data)


@router.delete("/task/{task_id}")
async def delete_task(task_id: str, current_user=Depends(get_current_admin)):
    file_path = TASKS_DIR / f"{task_id}.json"
    if not file_path.exists():
        raise HTTPException(status_code=404, detail="Task not found")
    file_path.unlink()
    return {"success": True, "message": f"Task '{task_id}' deleted"}


@router.post("/task/execute", response_model=TaskResponse)
async def execute_task(
    request: TaskExecuteRequest,
    current_user=Depends(get_current_admin),
):
    global _mock_execution_task
    watchdog.heartbeat()

    if request.task_json:
        task_data = request.task_json
    elif request.root:
        task_data = {
            "id": request.id,
            "name": request.name or "Unnamed Task",
            "root": request.root,
        }
    else:
        return TaskResponse(success=False, message="Missing task data")

    task_id = task_data.get("id") or str(uuid.uuid4())[:8]
    task_name = task_data.get("name", "Task")
    root_node = task_data.get("root", {})

    all_nodes = _collect_nodes(root_node)
    _mock_task_status.update(
        {
            "task_id": task_id,
            "task_name": task_name,
            "status": "running",
            "progress": 0.0,
            "elapsed_time": 0.0,
            "current_node_id": None,
            "completed_nodes": 0,
            "total_nodes": len(all_nodes),
            "node_statuses": all_nodes,
        }
    )

    if _mock_execution_task and not _mock_execution_task.done():
        _mock_execution_task.cancel()

    async def _runner():
        start_time = asyncio.get_event_loop().time()
        try:
            for idx, node in enumerate(_mock_task_status["node_statuses"]):
                _mock_task_status["current_node_id"] = node["node_id"]
                node["status"] = "running"
                await asyncio.sleep(0.2)
                node["status"] = "success"
                _mock_task_status["completed_nodes"] = idx + 1
                _mock_task_status["progress"] = (
                    _mock_task_status["completed_nodes"]
                    / max(_mock_task_status["total_nodes"], 1)
                )
            _mock_task_status["status"] = "completed"
        except asyncio.CancelledError:
            _mock_task_status["status"] = "cancelled"
        finally:
            _mock_task_status["elapsed_time"] = (
                asyncio.get_event_loop().time() - start_time
            )
            _mock_task_status["current_node_id"] = None

    _mock_execution_task = asyncio.create_task(_runner())

    return TaskResponse(
        success=True,
        task_id=task_id,
        message=f"Task '{task_name}' started (mock)",
    )


@router.post("/task/cancel")
async def cancel_task(current_user=Depends(get_current_admin)):
    global _mock_execution_task
    if _mock_execution_task and not _mock_execution_task.done():
        _mock_execution_task.cancel()
    _mock_task_status["status"] = "cancelled"
    _mock_task_status["current_node_id"] = None
    return {"success": True}


@router.get("/task/status", response_model=TaskStatusResponse)
async def get_task_status(current_user=Depends(get_current_admin)):
    return TaskStatusResponse(**_get_status_snapshot())
