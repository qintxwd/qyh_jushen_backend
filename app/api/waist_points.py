"""
腰部点位管理 API (Waist Points API)

提供腰部角度点位的 CRUD 操作
"""

from fastapi import APIRouter, Depends, HTTPException
from pydantic import BaseModel, Field
from typing import Optional, List
from pathlib import Path
import json
import uuid
from datetime import datetime
import threading

from app.dependencies import get_current_admin
from app.ros2_bridge.bridge import ros2_bridge

router = APIRouter()


# ==================== 数据模型 ====================

class WaistPoint(BaseModel):
    """腰部点位模型"""
    id: str = Field(..., description="点位唯一ID")
    name: str = Field(..., description="点位名称")
    description: str = Field(default="", description="点位描述")
    angle: float = Field(..., description="角度值 (度)", ge=0, le=45)
    is_builtin: bool = Field(default=False, description="是否为内置点位")
    created_at: str = Field(default_factory=lambda: datetime.now().isoformat())
    updated_at: str = Field(default_factory=lambda: datetime.now().isoformat())


# ==================== 点位管理器 ====================

class WaistPointsManager:
    """腰部点位管理器"""
    
    def __init__(self, storage_path: Path):
        self.storage_path = storage_path
        self.storage_path.parent.mkdir(parents=True, exist_ok=True)
        self._lock = threading.Lock()
        self._ensure_builtin_points()
    
    def _load(self) -> List[WaistPoint]:
        """加载点位列表"""
        if not self.storage_path.exists():
            return []
        
        try:
            with open(self.storage_path, 'r', encoding='utf-8') as f:
                data = json.load(f)
                return [WaistPoint(**item) for item in data]
        except Exception as e:
            print(f"Error loading waist points: {e}")
            return []
    
    def _save(self, points: List[WaistPoint]):
        """保存点位列表"""
        try:
            with open(self.storage_path, 'w', encoding='utf-8') as f:
                json.dump(
                    [p.model_dump() for p in points],
                    f,
                    ensure_ascii=False,
                    indent=2
                )
        except Exception as e:
            print(f"Error saving waist points: {e}")
            raise
    
    def _ensure_builtin_points(self):
        """确保内置点位存在"""
        with self._lock:
            points = self._load()
            
            # 内置点位定义
            builtin_points = [
                {
                    "id": "upright",
                    "name": "竖直",
                    "description": "腰部竖直站立",
                    "angle": 0.0,
                    "is_builtin": True
                },
                {
                    "id": "lean_15",
                    "name": "前倾15度",
                    "description": "腰部前倾15度",
                    "angle": 15.0,
                    "is_builtin": True
                },
                {
                    "id": "lean_30",
                    "name": "前倾30度",
                    "description": "腰部前倾30度",
                    "angle": 30.0,
                    "is_builtin": True
                }
            ]
            
            # 检查并添加缺失的内置点位
            existing_ids = {p.id for p in points}
            modified = False
            
            for builtin in builtin_points:
                if builtin["id"] not in existing_ids:
                    points.append(WaistPoint(**builtin))
                    modified = True
            
            if modified:
                self._save(points)
    
    def list_all(self) -> List[WaistPoint]:
        """获取所有点位"""
        with self._lock:
            return self._load()
    
    def get(self, point_id: str) -> Optional[WaistPoint]:
        """获取指定点位"""
        with self._lock:
            points = self._load()
            for point in points:
                if point.id == point_id:
                    return point
            return None
    
    def create(
        self,
        name: str,
        angle: float,
        description: str = ""
    ) -> WaistPoint:
        """创建新点位"""
        with self._lock:
            points = self._load()
            
            # 检查名称是否重复
            if any(p.name == name for p in points):
                raise ValueError(f"点位名称 '{name}' 已存在")
            
            # 创建新点位
            new_point = WaistPoint(
                id=str(uuid.uuid4())[:8],
                name=name,
                description=description,
                angle=angle,
                is_builtin=False
            )
            
            points.append(new_point)
            self._save(points)
            
            return new_point
    
    def update(
        self,
        point_id: str,
        name: Optional[str] = None,
        description: Optional[str] = None,
        angle: Optional[float] = None
    ) -> WaistPoint:
        """更新点位"""
        with self._lock:
            points = self._load()
            
            # 查找点位
            point_index = None
            for i, p in enumerate(points):
                if p.id == point_id:
                    point_index = i
                    break
            
            if point_index is None:
                raise ValueError(f"点位 {point_id} 不存在")
            
            point = points[point_index]
            
            # 内置点位不可修改
            if point.is_builtin:
                raise ValueError("内置点位不可修改")
            
            # 检查名称重复
            if name is not None and name != point.name:
                if any(p.name == name for p in points if p.id != point_id):
                    raise ValueError(f"点位名称 '{name}' 已存在")
            
            # 更新字段
            if name is not None:
                point.name = name
            if description is not None:
                point.description = description
            if angle is not None:
                point.angle = angle
            
            point.updated_at = datetime.now().isoformat()
            
            points[point_index] = point
            self._save(points)
            
            return point
    
    def delete(self, point_id: str):
        """删除点位"""
        with self._lock:
            points = self._load()
            
            # 查找点位
            point_index = None
            for i, p in enumerate(points):
                if p.id == point_id:
                    point_index = i
                    break
            
            if point_index is None:
                raise ValueError(f"点位 {point_id} 不存在")
            
            point = points[point_index]
            
            # 内置点位不可删除
            if point.is_builtin:
                raise ValueError("内置点位不可删除")
            
            del points[point_index]
            self._save(points)
    
    def get_point_angle(self, point_id: str) -> float:
        """获取点位的角度值（用于任务执行）"""
        point = self.get(point_id)
        if point is None:
            raise ValueError(f"点位 {point_id} 不存在")
        return point.angle


# 初始化管理器
STORAGE_PATH = Path.home() / "qyh-robot-system" / "persistent" / "preset" / "waist_points.json"
waist_points_manager = WaistPointsManager(STORAGE_PATH)


# ==================== API 端点 ====================


# 请求体模型：用于创建与采集
class WaistPointCreate(BaseModel):
    name: str = Field(..., description="点位名称")
    angle: float = Field(..., description="角度值 (度)", ge=0, le=45)
    description: str = Field(default="", description="点位描述")


# 更新模型
class WaistPointUpdate(BaseModel):
    name: Optional[str] = None
    description: Optional[str] = None
    angle: Optional[float] = Field(None, ge=0, le=45)


@router.get("/waist/points", response_model=List[WaistPoint])
async def list_waist_points(current_user=Depends(get_current_admin)):
    """获取所有腰部点位"""
    return waist_points_manager.list_all()


@router.get("/waist/points/{point_id}", response_model=WaistPoint)
async def get_waist_point(
    point_id: str,
    current_user=Depends(get_current_admin)
):
    """获取指定腰部点位"""
    point = waist_points_manager.get(point_id)
    if point is None:
        raise HTTPException(status_code=404, detail="点位不存在")
    return point


@router.post("/waist/points", response_model=WaistPoint)
async def create_waist_point(
    data: 'WaistPointCreate',
    current_user=Depends(get_current_admin)
):
    """创建腰部点位"""
    try:
        return waist_points_manager.create(
            name=data.name,
            angle=data.angle,
            description=data.description
        )
    except ValueError as e:
        raise HTTPException(status_code=400, detail=str(e))


@router.put("/waist/points/{point_id}", response_model=WaistPoint)
async def update_waist_point(
    point_id: str,
    data: 'WaistPointUpdate',
    current_user=Depends(get_current_admin)
):
    """更新腰部点位"""
    try:
        return waist_points_manager.update(
            point_id=point_id,
            name=data.name,
            description=data.description,
            angle=data.angle
        )
    except ValueError as e:
        raise HTTPException(status_code=400, detail=str(e))


@router.delete("/waist/points/{point_id}")
async def delete_waist_point(
    point_id: str,
    current_user=Depends(get_current_admin)
):
    """删除腰部点位"""
    try:
        waist_points_manager.delete(point_id)
        return {"message": "点位已删除"}
    except ValueError as e:
        raise HTTPException(status_code=400, detail=str(e))


@router.post("/waist/points/capture", response_model=WaistPoint)
async def capture_current_position(
    data: 'WaistPointCreate',
    current_user=Depends(get_current_admin)
):
    """采集当前腰部角度为点位（请求体：WaistPointCreate）"""
    # 从 ROS2 获取当前状态
    if ros2_bridge.is_connected():
        state = ros2_bridge.get_waist_state()
        if state:
            current_angle = state.get("current_angle", 0.0)
        else:
            raise HTTPException(status_code=400, detail="无法获取腰部状态")
    else:
        # Mock 模式：使用默认值
        current_angle = 0.0
    
    try:
        return waist_points_manager.create(
            name=data.name,
            angle=current_angle,
            description=data.description
        )
    except ValueError as e:
        raise HTTPException(status_code=400, detail=str(e))
