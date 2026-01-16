"""
头部点位管理 API

功能:
- 头部点位的增删改查
- 内置点位（零位、前方）不可删除
- 点位名称唯一
- 支持按名称执行点位
- 持久化到 head_points.json
"""
import os
import json
from pathlib import Path
from datetime import datetime
from typing import Optional, List, Dict, Any
from threading import Lock

from fastapi import APIRouter, Depends, HTTPException
from pydantic import BaseModel, Field

from app.dependencies import get_current_admin
from app.ros2_bridge.bridge import ros2_bridge
from app.safety.watchdog import watchdog
from app.schemas.response import (
    ApiResponse, success_response, error_response, ErrorCodes
)

router = APIRouter(prefix="/head/points", tags=["head_points"])

# 存储路径
STORAGE_PATH = Path.home() / "qyh-robot-system" / "persistent" / "preset"
POINTS_FILE = STORAGE_PATH / "head_points.json"

# 内置点位 ID（不可删除）
BUILTIN_POINT_IDS = ["zero", "forward"]

# 线程锁
_lock = Lock()


# ==================== 数据模型 ====================

class HeadPoint(BaseModel):
    """头部点位"""
    id: str = Field(..., description="点位唯一标识")
    name: str = Field(..., description="点位显示名称")
    description: str = Field(default="", description="点位描述")
    is_builtin: bool = Field(default=False, description="是否为内置点位")
    pan: float = Field(default=0.0, ge=-1.0, le=1.0, description="水平角度 (-1.0 到 1.0)")
    tilt: float = Field(default=0.0, ge=-1.0, le=1.0, description="上下角度 (-1.0 到 1.0)")
    created_at: str = Field(default_factory=lambda: datetime.now().isoformat())
    updated_at: str = Field(default_factory=lambda: datetime.now().isoformat())


class PointCreateRequest(BaseModel):
    """创建点位请求"""
    name: str = Field(..., min_length=1, max_length=50, description="点位名称")
    description: str = Field(default="", description="点位描述")


class PointUpdateRequest(BaseModel):
    """更新点位请求"""
    name: Optional[str] = Field(None, min_length=1, max_length=50, description="新名称")
    description: Optional[str] = Field(None, description="新描述")
    update_position: bool = Field(default=False, description="是否更新位置为当前位置")


class PointResponse(BaseModel):
    """点位响应"""
    success: bool
    message: str
    data: Optional[Dict[str, Any]] = None


class MoveToPointRequest(BaseModel):
    """移动到点位请求"""
    pass  # 头部不需要额外参数


# ==================== 点位管理器 ====================

class HeadPointsManager:
    """头部点位管理器"""
    
    def __init__(self):
        self._points: Dict[str, HeadPoint] = {}
        self._file_updated_at: Optional[str] = None
        self._load()
        self._ensure_builtin()
    
    def _load(self):
        """从文件加载点位"""
        STORAGE_PATH.mkdir(parents=True, exist_ok=True)
        
        if POINTS_FILE.exists():
            try:
                with open(POINTS_FILE, 'r', encoding='utf-8') as f:
                    data = json.load(f)
                    self._file_updated_at = data.get('updated_at')
                    for point_data in data.get('points', []):
                        point = HeadPoint(**point_data)
                        self._points[point.id] = point
            except Exception as e:
                print(f"加载头部点位失败: {e}")
    
    def _save(self, force_update_time: bool = True):
        """保存点位到文件
        
        Args:
            force_update_time: 是否强制更新时间戳（默认True，创建/更新/删除时使用）
        """
        data = {
            "version": "1.0",
            "updated_at": datetime.now().isoformat() if force_update_time else (self._file_updated_at or datetime.now().isoformat()),
            "points": [p.model_dump() for p in self._points.values()]
        }
        
        STORAGE_PATH.mkdir(parents=True, exist_ok=True)
        temp_file = POINTS_FILE.with_suffix('.tmp')
        
        with open(temp_file, 'w', encoding='utf-8') as f:
            json.dump(data, f, indent=2, ensure_ascii=False)
        
        temp_file.rename(POINTS_FILE)
    
    def _ensure_builtin(self):
        """确保内置点位存在"""
        modified = False
        
        # 零位（正中）
        if "zero" not in self._points:
            self._points["zero"] = HeadPoint(
                id="zero",
                name="零位",
                description="头部水平和垂直都在中间位置",
                is_builtin=True,
                pan=0.0,
                tilt=0.0
            )
            modified = True
        else:
            self._points["zero"].is_builtin = True
            self._points["zero"].name = "零位"
        
        # 正前方（略微向下）
        if "forward" not in self._points:
            self._points["forward"] = HeadPoint(
                id="forward",
                name="正前方",
                description="头部看向正前方，略微向下",
                is_builtin=True,
                pan=0.0,
                tilt=-0.2
            )
            modified = True
        else:
            self._points["forward"].is_builtin = True
        
        # 只在有新增点位时才保存，且不更新时间戳
        if modified:
            self._save(force_update_time=False)
    
    def list_all(self) -> List[HeadPoint]:
        """获取所有点位，内置点位排在前面"""
        with _lock:
            builtin = [p for p in self._points.values() if p.is_builtin]
            custom = [p for p in self._points.values() if not p.is_builtin]
            return builtin + custom
    
    def get(self, point_id: str) -> Optional[HeadPoint]:
        """获取点位"""
        with _lock:
            return self._points.get(point_id)
    
    def get_by_name(self, name: str) -> Optional[HeadPoint]:
        """通过名称获取点位"""
        with _lock:
            for point in self._points.values():
                if point.name == name:
                    return point
            return None
    
    def create(self, name: str, description: str, pan: float, tilt: float) -> HeadPoint:
        """创建新点位"""
        with _lock:
            # 检查名称是否已存在
            if self.get_by_name(name):
                raise ValueError(f"点位名称 '{name}' 已存在")
            
            # 生成唯一 ID
            point_id = f"point_{datetime.now().strftime('%s_%f')}"
            
            point = HeadPoint(
                id=point_id,
                name=name,
                description=description,
                is_builtin=False,
                pan=pan,
                tilt=tilt
            )
            
            self._points[point_id] = point
            self._save()
            return point
    
    def update(
        self, 
        point_id: str, 
        name: Optional[str] = None, 
        description: Optional[str] = None,
        pan: Optional[float] = None,
        tilt: Optional[float] = None
    ) -> Optional[HeadPoint]:
        """更新点位"""
        with _lock:
            point = self._points.get(point_id)
            if not point:
                return None
            
            # 检查新名称是否与其他点位冲突
            if name and name != point.name:
                existing = self.get_by_name(name)
                if existing and existing.id != point_id:
                    raise ValueError(f"点位名称 '{name}' 已被使用")
                point.name = name
            
            if description is not None:
                point.description = description
            
            if pan is not None:
                point.pan = pan
            
            if tilt is not None:
                point.tilt = tilt
            
            point.updated_at = datetime.now().isoformat()
            self._save()
            return point
    
    def delete(self, point_id: str) -> bool:
        """删除点位"""
        with _lock:
            point = self._points.get(point_id)
            if not point:
                return False
            
            if point.is_builtin:
                raise ValueError("不能删除内置点位")
            
            del self._points[point_id]
            self._save()
            return True


# 全局实例
points_manager = HeadPointsManager()


# ==================== API 端点 ====================

@router.get("")
async def list_points(current_user=Depends(get_current_admin)):
    """获取所有点位列表"""
    points = points_manager.list_all()
    return {
        "total": len(points),
        "points": [p.model_dump() for p in points]
    }


@router.get("/{point_id}")
async def get_point(point_id: str, current_user=Depends(get_current_admin)):
    """获取单个点位详情"""
    point = points_manager.get(point_id)
    
    # 尝试通过名称查找
    if not point:
        point = points_manager.get_by_name(point_id)
    
    if not point:
        raise HTTPException(status_code=404, detail=f"点位 '{point_id}' 不存在")
    
    return point.model_dump()


@router.post("")
async def create_point(
    request: PointCreateRequest,
    current_user=Depends(get_current_admin)
):
    """
    创建新点位（采集当前头部位置）
    """
    watchdog.heartbeat()
    
    # 获取当前头部位置
    if ros2_bridge.is_connected():
        head_state = ros2_bridge.get_head_state()
        if not head_state:
            raise HTTPException(status_code=400, detail="无法获取头部当前位置")
        pan = head_state.get('pan_normalized', 0.0)
        tilt = head_state.get('tilt_normalized', 0.0)
    else:
        raise HTTPException(status_code=400, detail="ROS2 未连接，无法获取头部位置")
    
    try:
        point = points_manager.create(
            name=request.name,
            description=request.description,
            pan=pan,
            tilt=tilt
        )
        return PointResponse(
            success=True,
            message=f"点位 '{request.name}' 创建成功",
            data=point.model_dump()
        )
    except ValueError as e:
        raise HTTPException(status_code=400, detail=str(e))


@router.put("/{point_id}")
async def update_point(
    point_id: str,
    request: PointUpdateRequest,
    current_user=Depends(get_current_admin)
):
    """更新点位信息或位置数据"""
    watchdog.heartbeat()
    
    pan = None
    tilt = None
    
    # 如果要更新位置数据
    if request.update_position:
        if ros2_bridge.is_connected():
            head_state = ros2_bridge.get_head_state()
            if not head_state:
                raise HTTPException(status_code=400, detail="无法获取头部当前位置")
            pan = head_state.get('pan_normalized', 0.0)
            tilt = head_state.get('tilt_normalized', 0.0)
        else:
            raise HTTPException(status_code=400, detail="ROS2 未连接，无法获取头部位置")
    
    try:
        point = points_manager.update(
            point_id=point_id,
            name=request.name,
            description=request.description,
            pan=pan,
            tilt=tilt
        )
        if not point:
            raise HTTPException(status_code=404, detail=f"点位 '{point_id}' 不存在")
        
        return PointResponse(
            success=True,
            message=f"点位 '{point.name}' 更新成功",
            data=point.model_dump()
        )
    except ValueError as e:
        raise HTTPException(status_code=400, detail=str(e))


@router.delete("/{point_id}")
async def delete_point(point_id: str, current_user=Depends(get_current_admin)):
    """删除点位"""
    try:
        if points_manager.delete(point_id):
            return PointResponse(success=True, message="点位删除成功")
        else:
            raise HTTPException(status_code=404, detail=f"点位 '{point_id}' 不存在")
    except ValueError as e:
        raise HTTPException(status_code=400, detail=str(e))


@router.post("/{point_id}/go")
async def go_to_point(
    point_id: str,
    request: MoveToPointRequest = MoveToPointRequest(),
    current_user=Depends(get_current_admin)
):
    """
    移动到指定点位
    
    支持通过 ID 或名称指定点位
    """
    watchdog.heartbeat()
    
    # 查找点位
    point = points_manager.get(point_id)
    if not point:
        point = points_manager.get_by_name(point_id)
    
    if not point:
        raise HTTPException(status_code=404, detail=f"点位 '{point_id}' 不存在")
    
    # 执行移动
    if ros2_bridge.is_connected():
        try:
            result = await ros2_bridge.send_head_command(
                pan=point.pan,
                tilt=point.tilt
            )
            if result and result.get('success'):
                return PointResponse(
                    success=True,
                    message=f"正在移动到点位 '{point.name}'",
                    data=point.model_dump()
                )
            else:
                raise HTTPException(status_code=500, detail="头部控制失败")
        except Exception as e:
            raise HTTPException(status_code=500, detail=f"移动失败: {str(e)}")
    else:
        raise HTTPException(status_code=400, detail="ROS2 未连接")


# ==================== 任务引擎集成 ====================

def get_point_position(point_name: str) -> Optional[Dict[str, float]]:
    """
    供任务引擎调用：获取点位的位置数据
    
    Args:
        point_name: 点位名称或ID
    
    Returns:
        {"pan": ..., "tilt": ...} 或 None
    """
    point = points_manager.get(point_name)
    if not point:
        point = points_manager.get_by_name(point_name)
    
    if point:
        return {
            "pan": point.pan,
            "tilt": point.tilt
        }
    return None
