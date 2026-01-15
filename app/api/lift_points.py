"""
升降点位管理 API (Lift Points API)

提供升降高度点位的 CRUD 操作
"""

from fastapi import APIRouter, Depends
from pydantic import BaseModel, Field
from typing import Optional, List
from pathlib import Path
import json
import uuid
from datetime import datetime
import threading

from app.dependencies import get_current_admin
from app.ros2_bridge.bridge import ros2_bridge
from app.schemas.response import (
    ApiResponse, success_response, error_response, ErrorCodes
)

router = APIRouter()


# ==================== 数据模型 ====================

class LiftPoint(BaseModel):
    """升降点位模型"""
    id: str = Field(..., description="点位唯一ID")
    name: str = Field(..., description="点位名称")
    description: str = Field(default="", description="点位描述")
    height: float = Field(..., description="高度值 (mm)", ge=0, le=500)
    is_builtin: bool = Field(default=False, description="是否为内置点位")
    created_at: str = Field(default_factory=lambda: datetime.now().isoformat())
    updated_at: str = Field(default_factory=lambda: datetime.now().isoformat())


# ==================== 点位管理器 ====================

class LiftPointCreate(BaseModel):
    name: str = Field(..., description="点位名称")
    height: float = Field(..., description="高度值 (mm)", ge=0, le=500)
    description: str = Field(default="", description="点位描述")
    
class LiftPointsManager:
    """升降点位管理器"""
    
    def __init__(self, storage_path: Path):
        self.storage_path = storage_path
        self.storage_path.parent.mkdir(parents=True, exist_ok=True)
        self._lock = threading.Lock()
        self._ensure_builtin_points()
    
    def _load(self) -> List[LiftPoint]:
        """加载点位列表"""
        if not self.storage_path.exists():
            return []
        
        try:
            with open(self.storage_path, 'r', encoding='utf-8') as f:
                data = json.load(f)
                return [LiftPoint(**item) for item in data]
        except Exception as e:
            print(f"Error loading lift points: {e}")
            return []
    
    def _save(self, points: List[LiftPoint]):
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
            print(f"Error saving lift points: {e}")
            raise
    
    def _ensure_builtin_points(self):
        """确保内置点位存在"""
        with self._lock:
            points = self._load()
            
            # 内置点位定义
            builtin_points = [
                {
                    "id": "lowest",
                    "name": "最低",
                    "description": "升降最低位置",
                    "height": 0.0,
                    "is_builtin": True
                },
                {
                    "id": "middle",
                    "name": "中间",
                    "description": "升降中间位置",
                    "height": 250.0,
                    "is_builtin": True
                },
                {
                    "id": "highest",
                    "name": "最高",
                    "description": "升降最高位置",
                    "height": 500.0,
                    "is_builtin": True
                }
            ]
            
            # 检查并添加缺失的内置点位
            existing_ids = {p.id for p in points}
            modified = False
            
            for builtin in builtin_points:
                if builtin["id"] not in existing_ids:
                    points.append(LiftPoint(**builtin))
                    modified = True
            
            # 只在有新增内置点位时才保存（_save会修改各点位的updated_at，但这是合理的）
            if modified:
                self._save(points)
    
    def list_all(self) -> List[LiftPoint]:
        """获取所有点位"""
        with self._lock:
            return self._load()
    
    def get(self, point_id: str) -> Optional[LiftPoint]:
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
        height: float,
        description: str = ""
    ) -> LiftPoint:
        """创建新点位"""
        with self._lock:
            points = self._load()
            
            # 检查名称是否重复
            if any(p.name == name for p in points):
                raise ValueError(f"点位名称 '{name}' 已存在")
            
            # 创建新点位
            new_point = LiftPoint(
                id=str(uuid.uuid4())[:8],
                name=name,
                description=description,
                height=height,
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
        height: Optional[float] = None
    ) -> LiftPoint:
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
            if height is not None:
                point.height = height
            
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
    
    def get_point_height(self, point_id: str) -> float:
        """获取点位的高度值（用于任务执行）"""
        point = self.get(point_id)
        if point is None:
            raise ValueError(f"点位 {point_id} 不存在")
        return point.height


# 初始化管理器
STORAGE_PATH = Path.home() / "qyh-robot-system" / "persistent" / "preset" / "lift_points.json"
lift_points_manager = LiftPointsManager(STORAGE_PATH)


# ==================== API 端点 ====================

@router.get("/lift/points", response_model=List[LiftPoint])
async def list_lift_points(current_user=Depends(get_current_admin)):
    """获取所有升降点位"""
    return lift_points_manager.list_all()


@router.get("/lift/points/{point_id}", response_model=LiftPoint)
async def get_lift_point(
    point_id: str,
    current_user=Depends(get_current_admin)
):
    """获取指定升降点位"""
    point = lift_points_manager.get(point_id)
    if point is None:
        raise HTTPException(status_code=404, detail="点位不存在")
    return point


@router.post("/lift/points", response_model=LiftPoint)
async def create_lift_point(
    data: LiftPointCreate,
    current_user=Depends(get_current_admin)
):
    """创建升降点位"""
    try:
        return lift_points_manager.create(
            name=data.name,
            height=data.height,
            description=data.description
        )
    except ValueError as e:
        raise HTTPException(status_code=400, detail=str(e))



# 新增：用于更新的模型
class LiftPointUpdate(BaseModel):
    name: Optional[str] = None
    description: Optional[str] = None
    height: Optional[float] = Field(None, ge=0, le=500)

@router.put("/lift/points/{point_id}", response_model=LiftPoint)
async def update_lift_point(
    point_id: str,
    data: LiftPointUpdate,
    current_user=Depends(get_current_admin)
):
    """更新升降点位"""
    try:
        return lift_points_manager.update(
            point_id=point_id,
            name=data.name,
            description=data.description,
            height=data.height
        )
    except ValueError as e:
        raise HTTPException(status_code=400, detail=str(e))


@router.delete("/lift/points/{point_id}")
async def delete_lift_point(
    point_id: str,
    current_user=Depends(get_current_admin)
):
    """删除升降点位"""
    try:
        lift_points_manager.delete(point_id)
        return {"message": "点位已删除"}
    except ValueError as e:
        raise HTTPException(status_code=400, detail=str(e))


@router.post("/lift/points/capture", response_model=LiftPoint)
async def capture_current_position(
    data: LiftPointCreate,
    current_user=Depends(get_current_admin)
):
    """采集当前升降高度为点位（请求体：LiftPointCreate）"""
    # 从 ROS2 获取当前状态
    if ros2_bridge.is_connected():
        state = ros2_bridge.get_lift_state()
        if state:
            current_height = state.get("current_position", 0.0)
        else:
            raise HTTPException(status_code=400, detail="无法获取升降状态")
    else:
        # Mock 模式：使用默认值
        current_height = 0.0

    try:
        return lift_points_manager.create(
            name=data.name,
            height=current_height,
            description=data.description
        )
    except ValueError as e:
        raise HTTPException(status_code=400, detail=str(e))
