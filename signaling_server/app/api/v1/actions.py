"""Model actions management API (filesystem-backed)."""
from __future__ import annotations

import os
from pathlib import Path
from typing import Any, Dict, List, Optional

import yaml
from fastapi import APIRouter, Query
from pydantic import BaseModel

from app.core.paths import get_repo_root

router = APIRouter(prefix="/api/v1/actions", tags=["actions"])

MODEL_ACTIONS_BASE = get_repo_root() / "model_actions"
DEFAULT_ROBOT_NAME = os.environ.get("GLOBAL_ROBOT_NAME", "general")
DEFAULT_ROBOT_VERSION = os.environ.get("GLOBAL_ROBOT_VERSION", "1.0")


class ActionSummary(BaseModel):
    id: str
    name: str
    description: str = ""
    version: str = "1.0.0"
    tags: List[str] = []
    status: str = "collecting"
    has_model: bool = False
    episode_count: int = 0
    model_version: Optional[int] = None
    topics: List[str] = []
    camera_count: int = 0
    robot_name: str = DEFAULT_ROBOT_NAME
    robot_version: str = DEFAULT_ROBOT_VERSION


class ActionDetail(BaseModel):
    id: str
    name: str
    description: str = ""
    version: str = "1.0.0"
    tags: List[str] = []
    status: str = "collecting"
    has_model: bool = False
    episode_count: int = 0
    model_version: Optional[int] = None
    robot_name: str = DEFAULT_ROBOT_NAME
    robot_version: str = DEFAULT_ROBOT_VERSION
    collection: Dict[str, Any] = {}
    training: Dict[str, Any] = {}
    inference: Dict[str, Any] = {}


def _actions_dir(robot_name: str | None, robot_version: str | None) -> Path:
    rn = robot_name or DEFAULT_ROBOT_NAME
    rv = robot_version or DEFAULT_ROBOT_VERSION
    return MODEL_ACTIONS_BASE / rn / rv


def _read_action_yaml(action_dir: Path) -> Dict[str, Any]:
    action_file = action_dir / "action.yaml"
    if not action_file.exists():
        return {}
    with open(action_file, "r", encoding="utf-8") as handle:
        return yaml.safe_load(handle) or {}


def _has_model(action_dir: Path) -> bool:
    return (action_dir / "model" / "policy.pt").exists()


def _count_episodes(action_dir: Path) -> int:
    count = 0
    episodes_dir = action_dir / "data" / "episodes"
    if episodes_dir.exists():
        count += len(list(episodes_dir.glob("*.hdf5")))
    bags_dir = action_dir / "data" / "bags"
    if bags_dir.exists():
        for item in bags_dir.iterdir():
            if item.is_dir() and (item / "metadata.yaml").exists():
                count += 1
    return count


@router.get("/list")
async def list_actions(
    robot_name: Optional[str] = Query(None),
    robot_version: Optional[str] = Query(None),
):
    actions: List[ActionSummary] = []
    base_dir = _actions_dir(robot_name, robot_version)
    base_dir.mkdir(parents=True, exist_ok=True)
    for action_dir in base_dir.iterdir():
        if not action_dir.is_dir():
            continue
        meta = _read_action_yaml(action_dir)
        actions.append(
            ActionSummary(
                id=action_dir.name,
                name=meta.get("name", action_dir.name),
                description=meta.get("description", ""),
                version=meta.get("version", "1.0.0"),
                tags=meta.get("tags", []),
                status=meta.get(
                    "status",
                    "trained" if _has_model(action_dir) else "collecting",
                ),
                has_model=_has_model(action_dir),
                episode_count=_count_episodes(action_dir),
                model_version=meta.get("model_version"),
                topics=meta.get("topics", []),
                camera_count=meta.get("camera_count", 0),
                robot_name=robot_name or DEFAULT_ROBOT_NAME,
                robot_version=robot_version or DEFAULT_ROBOT_VERSION,
            )
        )
    return {
        "success": True,
        "actions": [action.model_dump() for action in actions],
        "total": len(actions),
        "robot_name": robot_name or DEFAULT_ROBOT_NAME,
        "robot_version": robot_version or DEFAULT_ROBOT_VERSION,
    }


@router.get("/{action_id}")
async def get_action(
    action_id: str,
    robot_name: Optional[str] = Query(None),
    robot_version: Optional[str] = Query(None),
):
    action_dir = _actions_dir(robot_name, robot_version) / action_id
    if not action_dir.exists():
        return {"success": False, "message": "Action not found"}
    meta = _read_action_yaml(action_dir)
    detail = ActionDetail(
        id=action_id,
        name=meta.get("name", action_id),
        description=meta.get("description", ""),
        version=meta.get("version", "1.0.0"),
        tags=meta.get("tags", []),
        status=meta.get(
            "status",
            "trained" if _has_model(action_dir) else "collecting",
        ),
        has_model=_has_model(action_dir),
        episode_count=_count_episodes(action_dir),
        model_version=meta.get("model_version"),
        robot_name=robot_name or DEFAULT_ROBOT_NAME,
        robot_version=robot_version or DEFAULT_ROBOT_VERSION,
        collection=meta.get("collection", {}),
        training=meta.get("training", {}),
        inference=meta.get("inference", {}),
    )
    return {"success": True, "action": detail.model_dump()}
