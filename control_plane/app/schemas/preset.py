"""
QYH Jushen Control Plane - 预设 Schema

定义预设相关的请求/响应模型
"""
from enum import Enum
from typing import Optional, List, Dict, Any
from pydantic import BaseModel, Field
from datetime import datetime


class PresetType(str, Enum):
    """预设类型枚举"""
    ARM_POSE = "arm_pose"               # 机械臂点位
    HEAD_POSITION = "head_position"     # 头部点位
    LIFT_HEIGHT = "lift_height"         # 升降高度
    WAIST_ANGLE = "waist_angle"         # 腰部角度
    LOCATION = "location"               # 底盘导航点
    GRIPPER_POSITION = "gripper_position"  # 夹爪位置
    FULL_POSE = "full_pose"             # 完整姿态


# ==================== 预设数据结构 ====================

class ArmPoseData(BaseModel):
    """机械臂点位数据"""
    side: str = Field(default="both", description="适用侧: left, right, both")
    pose_type: str = Field(default="joint", description="姿态类型: joint, cartesian")
    left_joints: Optional[List[float]] = Field(default=None, description="左臂关节角度 (7个值)")
    right_joints: Optional[List[float]] = Field(default=None, description="右臂关节角度 (7个值)")
    velocity: float = Field(default=0.5, description="运动速度")
    acceleration: float = Field(default=0.3, description="运动加速度")


class HeadPositionData(BaseModel):
    """头部点位数据"""
    pan: float = Field(default=0.0, ge=-1.0, le=1.0, description="水平角度 (-1.0 到 1.0)")
    tilt: float = Field(default=0.0, ge=-1.0, le=1.0, description="上下角度 (-1.0 到 1.0)")


class LiftHeightData(BaseModel):
    """升降高度数据"""
    height: float = Field(..., ge=0, le=500, description="高度值 (mm)")


class WaistAngleData(BaseModel):
    """腰部角度数据"""
    angle: float = Field(..., ge=0, le=45, description="角度值 (度)")


class LocationData(BaseModel):
    """底盘导航点数据"""
    x: float = Field(..., description="X 坐标 (米)")
    y: float = Field(..., description="Y 坐标 (米)")
    theta: float = Field(default=0.0, description="朝向角度 (弧度)")
    frame_id: str = Field(default="map", description="坐标系")
    station_id: Optional[int] = Field(default=None, description="地图站点 ID")


class GripperPositionData(BaseModel):
    """夹爪位置数据"""
    side: str = Field(default="both", description="适用侧: left, right, both")
    left_position: Optional[float] = Field(default=None, ge=0, le=1, description="左夹爪位置 (0-1)")
    right_position: Optional[float] = Field(default=None, ge=0, le=1, description="右夹爪位置 (0-1)")
    force: float = Field(default=20.0, description="夹持力 (N)")


class FullPoseData(BaseModel):
    """完整姿态数据"""
    arm: Optional[ArmPoseData] = None
    head: Optional[HeadPositionData] = None
    lift: Optional[LiftHeightData] = None
    waist: Optional[WaistAngleData] = None
    gripper: Optional[GripperPositionData] = None


# ==================== 请求模型 ====================

class CreatePresetRequest(BaseModel):
    """创建预设请求"""
    name: str = Field(..., min_length=1, max_length=50, description="预设名称")
    description: str = Field(default="", max_length=200, description="描述")
    category: str = Field(default="custom", description="分类")
    preset_type: PresetType = Field(..., description="预设类型")
    data: Dict[str, Any] = Field(..., description="预设数据")


class UpdatePresetRequest(BaseModel):
    """更新预设请求"""
    name: Optional[str] = Field(None, min_length=1, max_length=50, description="新名称")
    description: Optional[str] = Field(None, max_length=200, description="新描述")
    category: Optional[str] = Field(None, description="新分类")
    data: Optional[Dict[str, Any]] = Field(None, description="新数据")


class CapturePresetRequest(BaseModel):
    """采集当前状态为预设"""
    name: str = Field(..., min_length=1, max_length=50, description="预设名称")
    description: str = Field(default="", description="描述")
    preset_type: PresetType = Field(..., description="预设类型")
    side: str = Field(default="both", description="采集侧 (用于 arm_pose): left, right, both")


class ApplyPresetRequest(BaseModel):
    """应用预设请求"""
    velocity: float = Field(default=0.5, gt=0, le=3.14, description="运动速度")
    acceleration: float = Field(default=0.3, gt=0, le=10.0, description="运动加速度")
    side: str = Field(default="both", description="应用侧: left, right, both")
    wait: bool = Field(default=True, description="是否等待执行完成")


# ==================== 响应模型 ====================

class PresetInfo(BaseModel):
    """预设信息"""
    id: str = Field(..., description="预设唯一标识")
    name: str = Field(..., description="显示名称")
    description: str = Field(default="", description="描述")
    category: str = Field(default="custom", description="分类")
    preset_type: str = Field(..., description="预设类型")
    is_builtin: bool = Field(default=False, description="是否为内置预设")
    data: Dict[str, Any] = Field(..., description="预设数据")
    created_at: str = Field(..., description="创建时间")
    updated_at: str = Field(..., description="更新时间")


class PresetListResponse(BaseModel):
    """预设列表响应"""
    preset_type: str
    total: int
    items: List[PresetInfo]


class PresetTypeInfo(BaseModel):
    """预设类型信息"""
    type: str
    name: str
    description: str
    data_schema: Dict[str, Any]
