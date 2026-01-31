"""Preset management API (file-backed)."""
from __future__ import annotations

import json
import uuid
from pathlib import Path
from typing import Any, Dict, List, Optional

from fastapi import APIRouter, Depends, HTTPException
from pydantic import BaseModel, Field

from app.core.paths import get_repo_root
from app.dependencies import get_current_admin

router = APIRouter(prefix="/api/v1", tags=["presets"])

PRESET_DIR = get_repo_root() / "persistent" / "preset"
PRESET_DIR.mkdir(parents=True, exist_ok=True)


class PresetCreateRequest(BaseModel):
    name: str = Field(...)
    description: str = Field(default="")
    category: str = Field(default="custom")
    data: Dict[str, Any] = Field(default_factory=dict)


class PresetUpdateRequest(BaseModel):
    name: Optional[str] = None
    description: Optional[str] = None
    category: Optional[str] = None
    data: Optional[Dict[str, Any]] = None


class PresetResponse(BaseModel):
    success: bool
    message: str
    data: Optional[Dict[str, Any]] = None


def _preset_file(preset_type: str) -> Path:
    mapping = {
        "arm": "arm_points.json",
        "arm_pose": "arm_points.json",
        "arm_points": "arm_points.json",
        "head": "head_points.json",
        "head_position": "head_points.json",
        "head_points": "head_points.json",
        "lift": "lift_points.json",
        "lift_height": "lift_points.json",
        "lift_points": "lift_points.json",
        "waist": "waist_points.json",
        "waist_position": "waist_points.json",
        "waist_points": "waist_points.json",
    }
    filename = mapping.get(preset_type, f"{preset_type}.json")
    return PRESET_DIR / filename


def _load_items(preset_type: str) -> List[Dict[str, Any]]:
    file_path = _preset_file(preset_type)
    if not file_path.exists():
        return []
    with open(file_path, "r", encoding="utf-8") as handle:
        data = json.load(handle)
    if isinstance(data, dict) and "points" in data:
        return data.get("points", [])
    if isinstance(data, dict) and "items" in data:
        return data.get("items", [])
    if isinstance(data, list):
        return data
    return []


def _save_items(preset_type: str, items: List[Dict[str, Any]]) -> None:
    file_path = _preset_file(preset_type)
    if file_path.name.endswith("_points.json"):
        payload = {"points": items}
    else:
        payload = {"items": items}
    with open(file_path, "w", encoding="utf-8") as handle:
        json.dump(payload, handle, ensure_ascii=False, indent=2)


@router.get("/presets/{preset_type}")
async def list_presets(
    preset_type: str,
    category: Optional[str] = None,
    current_user=Depends(get_current_admin),
):
    items = _load_items(preset_type)
    if category:
        items = [item for item in items if item.get("category") == category]
    return {"type": preset_type, "total": len(items), "items": items}


@router.get("/presets/{preset_type}/{preset_id}")
async def get_preset(
    preset_type: str,
    preset_id: str,
    current_user=Depends(get_current_admin),
):
    items = _load_items(preset_type)
    for item in items:
        if item.get("id") == preset_id or item.get("name") == preset_id:
            return item
    raise HTTPException(status_code=404, detail="Preset not found")


@router.post("/presets/{preset_type}")
async def create_preset(
    preset_type: str,
    request: PresetCreateRequest,
    current_user=Depends(get_current_admin),
):
    items = _load_items(preset_type)
    preset_id = str(uuid.uuid4())[:8]
    data = {
        "id": preset_id,
        "name": request.name,
        "description": request.description,
        "category": request.category,
        **request.data,
    }
    items.append(data)
    _save_items(preset_type, items)
    return PresetResponse(success=True, message="Preset created", data=data)


@router.put("/presets/{preset_type}/{preset_id}")
async def update_preset(
    preset_type: str,
    preset_id: str,
    request: PresetUpdateRequest,
    current_user=Depends(get_current_admin),
):
    items = _load_items(preset_type)
    for item in items:
        if item.get("id") == preset_id:
            if request.name is not None:
                item["name"] = request.name
            if request.description is not None:
                item["description"] = request.description
            if request.category is not None:
                item["category"] = request.category
            if request.data is not None:
                item.update(request.data)
            _save_items(preset_type, items)
            return PresetResponse(
                success=True,
                message="Preset updated",
                data=item,
            )
    raise HTTPException(status_code=404, detail="Preset not found")


@router.delete("/presets/{preset_type}/{preset_id}")
async def delete_preset(
    preset_type: str,
    preset_id: str,
    current_user=Depends(get_current_admin),
):
    items = _load_items(preset_type)
    filtered = [item for item in items if item.get("id") != preset_id]
    if len(filtered) == len(items):
        raise HTTPException(status_code=404, detail="Preset not found")
    _save_items(preset_type, filtered)
    return PresetResponse(success=True, message="Preset deleted")


@router.get("/presets/{preset_type}/categories")
async def get_categories(
    preset_type: str,
    current_user=Depends(get_current_admin),
):
    items = _load_items(preset_type)
    categories = sorted(
        {
            item.get("category", "")
            for item in items
            if item.get("category")
        }
    )
    return {"categories": categories}
