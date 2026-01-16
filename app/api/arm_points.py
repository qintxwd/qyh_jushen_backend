"""
机械臂点位管理 API

功能:
- 点位的增删改查
- 内置点位（零位、初始点）不可删除
- 点位名称唯一
- 支持按名称执行点位
- 持久化到 arm_points.json
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

router = APIRouter(prefix="/arm/points", tags=["arm_points"])

# 存储路径
STORAGE_PATH = Path.home() / "qyh-robot-system" / "persistent" / "preset"
POINTS_FILE = STORAGE_PATH / "arm_points.json"

# 内置点位 ID（不可删除）
BUILTIN_POINT_IDS = ["zero", "home"]

# 线程锁
_lock = Lock()


# ==================== 数据模型 ====================

class ArmPoint(BaseModel):
    """机械臂点位"""
    id: str = Field(..., description="点位唯一标识")
    name: str = Field(..., description="点位显示名称")
    description: str = Field(default="", description="点位描述")
    is_builtin: bool = Field(default=False, description="是否为内置点位")
    left_joints: List[float] = Field(default_factory=lambda: [0.0] * 7, description="左臂关节角度 (弧度)")
    right_joints: List[float] = Field(default_factory=lambda: [0.0] * 7, description="右臂关节角度 (弧度)")
    created_at: str = Field(default_factory=lambda: datetime.now().isoformat())
    updated_at: str = Field(default_factory=lambda: datetime.now().isoformat())


class PointCreateRequest(BaseModel):
    """创建点位请求"""
    name: str = Field(..., min_length=1, max_length=50, description="点位名称")
    description: str = Field(default="", description="点位描述")
    side: str = Field(default="both", description="采集哪一侧: left, right, both")


class PointUpdateRequest(BaseModel):
    """更新点位请求"""
    name: Optional[str] = Field(None, min_length=1, max_length=50, description="新名称")
    description: Optional[str] = Field(None, description="新描述")
    update_joints: bool = Field(default=False, description="是否更新关节数据为当前位置")
    side: Optional[str] = Field(None, description="更新哪一侧: left, right, both")


class PointResponse(BaseModel):
    """点位响应"""
    success: bool
    message: str
    data: Optional[Dict[str, Any]] = None


class MoveToPointRequest(BaseModel):
    """移动到点位请求"""
    velocity: float = Field(default=0.5, gt=0, le=3.14, description="速度 (rad/s)")
    acceleration: float = Field(default=0.3, gt=0, le=10.0, description="加速度 (rad/s²)")
    side: str = Field(default="both", description="移动哪一侧: left, right, both")


# ==================== 点位管理器 ====================

class ArmPointsManager:
    """机械臂点位管理器"""
    
    def __init__(self):
        self._points: Dict[str, ArmPoint] = {}
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
                
                # 保存文件的 updated_at
                self._file_updated_at = data.get('updated_at')
                
                for item in data.get('points', []):
                    try:
                        point = ArmPoint(**item)
                        self._points[point.id] = point
                    except Exception as e:
                        print(f"⚠️ 加载点位失败: {e}")
                
                print(f"✅ 加载 {len(self._points)} 个机械臂点位")
            except Exception as e:
                print(f"❌ 加载点位文件失败: {e}")
    
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
            json.dump(data, f, ensure_ascii=False, indent=2)
        
        temp_file.rename(POINTS_FILE)
    
    def _ensure_builtin(self):
        """确保内置点位存在"""
        modified = False
        
        # 零位
        if "zero" not in self._points:
            self._points["zero"] = ArmPoint(
                id="zero",
                name="零位",
                description="机械臂各关节角度为0的位置",
                is_builtin=True,
                left_joints=[0.0] * 7,
                right_joints=[0.0] * 7
            )
            modified = True
        else:
            # 确保零位始终是0（但不影响时间戳）
            self._points["zero"].left_joints = [0.0] * 7
            self._points["zero"].right_joints = [0.0] * 7
            self._points["zero"].is_builtin = True
        
        # 初始点
        if "home" not in self._points:
            self._points["home"] = ArmPoint(
                id="home",
                name="初始点",
                description="机械臂工作初始位置（可更新）",
                is_builtin=True,
                left_joints=[0.0] * 7,
                right_joints=[0.0] * 7
            )
            modified = True
        else:
            self._points["home"].is_builtin = True
        
        # 只在有新增点位时才保存，且不更新时间戳
        if modified:
            self._save(force_update_time=False)
    
    def list_all(self) -> List[ArmPoint]:
        """获取所有点位，内置点位排在前面"""
        with _lock:
            builtin = [p for p in self._points.values() if p.is_builtin]
            custom = [p for p in self._points.values() if not p.is_builtin]
            # 按名称排序
            custom.sort(key=lambda x: x.name)
            return builtin + custom
    
    def get(self, point_id: str) -> Optional[ArmPoint]:
        """获取点位"""
        with _lock:
            return self._points.get(point_id)
    
    def get_by_name(self, name: str) -> Optional[ArmPoint]:
        """通过名称获取点位"""
        with _lock:
            for point in self._points.values():
                if point.name == name:
                    return point
            return None
    
    def create(self, name: str, description: str, left_joints: List[float], right_joints: List[float]) -> ArmPoint:
        """创建新点位"""
        with _lock:
            # 检查名称是否重复
            for p in self._points.values():
                if p.name == name:
                    raise ValueError(f"点位名称 '{name}' 已存在")
            
            # 生成 ID
            import uuid
            point_id = f"point_{uuid.uuid4().hex[:8]}"
            
            point = ArmPoint(
                id=point_id,
                name=name,
                description=description,
                is_builtin=False,
                left_joints=left_joints,
                right_joints=right_joints
            )
            
            self._points[point_id] = point
            self._save()
            
            return point
    
    def update(
        self, 
        point_id: str, 
        name: Optional[str] = None, 
        description: Optional[str] = None,
        left_joints: Optional[List[float]] = None,
        right_joints: Optional[List[float]] = None
    ) -> Optional[ArmPoint]:
        """更新点位"""
        with _lock:
            point = self._points.get(point_id)
            if not point:
                return None
            
            # 零位不允许更新关节数据
            if point_id == "zero" and (left_joints is not None or right_joints is not None):
                raise ValueError("零位的关节数据不可修改")
            
            # 检查名称是否与其他点位重复
            if name is not None and name != point.name:
                for p in self._points.values():
                    if p.id != point_id and p.name == name:
                        raise ValueError(f"点位名称 '{name}' 已存在")
                point.name = name
            
            if description is not None:
                point.description = description
            
            if left_joints is not None:
                point.left_joints = left_joints
            
            if right_joints is not None:
                point.right_joints = right_joints
            
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
                raise ValueError(f"内置点位 '{point.name}' 不可删除")
            
            del self._points[point_id]
            self._save()
            return True


# 全局实例
points_manager = ArmPointsManager()


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
    if not point:
        # 尝试通过名称查找
        point = points_manager.get_by_name(point_id)
    
    if not point:
        raise HTTPException(status_code=404, detail="点位不存在")
    
    return point.model_dump()


@router.post("")
async def create_point(
    request: PointCreateRequest,
    current_user=Depends(get_current_admin)
):
    """
    创建新点位（采集当前机械臂位置）
    """
    watchdog.heartbeat()
    
    # 获取当前关节位置
    joint_positions = ros2_bridge.joint_positions
    
    if not joint_positions or len(joint_positions) < 14:
        # 提供更详细的错误信息
        if not ros2_bridge.is_connected():
            raise HTTPException(status_code=400, detail="ROS2未连接，无法获取关节数据。请确保机械臂系统已启动。")
        else:
            raise HTTPException(status_code=400, detail=f"无法获取完整的关节数据（当前: {len(joint_positions) if joint_positions else 0}/14）")
    
    left_joints = list(joint_positions[:7])
    right_joints = list(joint_positions[7:14])
    
    # 根据 side 参数决定保存哪一侧
    if request.side == "left":
        right_joints = [0.0] * 7  # 右臂使用零位
    elif request.side == "right":
        left_joints = [0.0] * 7  # 左臂使用零位
    
    try:
        point = points_manager.create(
            name=request.name,
            description=request.description,
            left_joints=left_joints,
            right_joints=right_joints
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
    """更新点位信息或关节数据"""
    watchdog.heartbeat()
    
    left_joints = None
    right_joints = None
    
    # 如果要更新关节数据
    if request.update_joints:
        joint_positions = ros2_bridge.joint_positions
        if not joint_positions or len(joint_positions) < 14:
            # 提供更详细的错误信息
            if not ros2_bridge.is_connected():
                raise HTTPException(status_code=400, detail="ROS2未连接，无法获取关节数据。请确保机械臂系统已启动。")
            else:
                raise HTTPException(status_code=400, detail=f"无法获取完整的关节数据（当前: {len(joint_positions) if joint_positions else 0}/14）")
        
        side = request.side or "both"
        
        if side in ["left", "both"]:
            left_joints = list(joint_positions[:7])
        if side in ["right", "both"]:
            right_joints = list(joint_positions[7:14])
    
    try:
        point = points_manager.update(
            point_id=point_id,
            name=request.name,
            description=request.description,
            left_joints=left_joints,
            right_joints=right_joints
        )
        
        if not point:
            raise HTTPException(status_code=404, detail="点位不存在")
        
        return PointResponse(
            success=True,
            message=f"点位 '{point.name}' 已更新",
            data=point.model_dump()
        )
    except ValueError as e:
        raise HTTPException(status_code=400, detail=str(e))


@router.delete("/{point_id}")
async def delete_point(point_id: str, current_user=Depends(get_current_admin)):
    """删除点位"""
    try:
        success = points_manager.delete(point_id)
        if not success:
            raise HTTPException(status_code=404, detail="点位不存在")
        
        return PointResponse(success=True, message="点位已删除")
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
    
    # 构建关节目标
    # 14 个关节: [左臂7, 右臂7]
    left_joints = point.left_joints
    right_joints = point.right_joints
    
    results = []
    
    # 根据 side 决定移动哪只手臂
    if request.side in ["left", "both"]:
        if ros2_bridge.is_connected():
            result = await ros2_bridge.call_move_j(
                robot_id=0,  # 左臂
                joint_positions=left_joints + [0.0] * 7,  # API 需要 14 个关节
                velocity=request.velocity,
                acceleration=request.acceleration,
                is_block=False
            )
            results.append({"left": result})
        else:
            results.append({"left": {"success": True, "message": "mock mode"}})
    
    if request.side in ["right", "both"]:
        if ros2_bridge.is_connected():
            result = await ros2_bridge.call_move_j(
                robot_id=1,  # 右臂
                joint_positions=[0.0] * 7 + right_joints,  # API 需要 14 个关节
                velocity=request.velocity,
                acceleration=request.acceleration,
                is_block=False
            )
            results.append({"right": result})
        else:
            results.append({"right": {"success": True, "message": "mock mode"}})
    
    side_text = {"left": "左臂", "right": "右臂", "both": "双臂"}[request.side]
    return PointResponse(
        success=True,
        message=f"正在移动{side_text}到 '{point.name}'",
        data={
            "point": point.model_dump(),
            "results": results
        }
    )


# ==================== 任务引擎集成 ====================

def get_point_joints(point_name: str) -> Optional[Dict[str, List[float]]]:
    """
    供任务引擎调用：获取点位的关节数据
    
    Args:
        point_name: 点位名称或ID
    
    Returns:
        {"left": [...], "right": [...]} 或 None
    """
    point = points_manager.get(point_name)
    if not point:
        point = points_manager.get_by_name(point_name)
    
    if point:
        return {
            "left": point.left_joints,
            "right": point.right_joints
        }
    return None
