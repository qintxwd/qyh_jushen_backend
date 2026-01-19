"""
预设数据模型 (Preset Models)

定义各种预设的数据结构
"""

from enum import Enum
from typing import Optional, List, Dict, Any
from pydantic import BaseModel, Field
from datetime import datetime


class PresetType(str, Enum):
    """预设类型枚举"""
    LOCATION = "location"           # 底盘点位
    ARM_POSE = "arm_pose"           # 手臂姿态
    LIFT_HEIGHT = "lift_height"     # 升降高度
    HEAD_POSITION = "head_position" # 头部位置
    GRIPPER_POSITION = "gripper_position"  # 夹爪位置
    TASK_TEMPLATE = "task_template" # 任务模板


class PresetBase(BaseModel):
    """预设基类"""
    id: str = Field(..., description="唯一标识符")
    name: str = Field(..., description="显示名称")
    description: str = Field(default="", description="描述说明")
    category: str = Field(default="default", description="分类标签")
    is_builtin: bool = Field(default=False, description="是否为内置预设")
    created_at: str = Field(default_factory=lambda: datetime.now().isoformat())
    updated_at: str = Field(default_factory=lambda: datetime.now().isoformat())
    
    class Config:
        extra = "allow"


# ==================== 底盘点位 ====================

class Location(PresetBase):
    """
    底盘点位预设
    
    用于导航到指定位置
    """
    x: float = Field(..., description="X 坐标 (米)")
    y: float = Field(..., description="Y 坐标 (米)")
    theta: float = Field(default=0.0, description="朝向角度 (弧度)")
    frame_id: str = Field(default="map", description="坐标系")
    
    # 可选：导航参数
    tolerance_xy: float = Field(default=0.1, description="位置容差 (米)")
    tolerance_theta: float = Field(default=0.1, description="角度容差 (弧度)")


# ==================== 手臂姿态 ====================

class ArmPose(PresetBase):
    """
    手臂姿态预设
    
    支持单臂或双臂姿态定义
    """
    # 姿态类型
    pose_type: str = Field(
        default="joint", 
        description="姿态类型: joint (关节角度) 或 cartesian (笛卡尔坐标)"
    )
    
    # 适用的手臂
    side: str = Field(
        default="both",
        description="适用手臂: left, right, both"
    )
    
    # 关节角度模式 (7个关节)
    left_joints: Optional[List[float]] = Field(
        default=None, 
        description="左臂关节角度 (弧度), 7个值"
    )
    right_joints: Optional[List[float]] = Field(
        default=None, 
        description="右臂关节角度 (弧度), 7个值"
    )
    
    # 笛卡尔坐标模式
    left_cartesian: Optional[Dict[str, float]] = Field(
        default=None,
        description="左臂末端位姿 {x, y, z, rx, ry, rz}"
    )
    right_cartesian: Optional[Dict[str, float]] = Field(
        default=None,
        description="右臂末端位姿 {x, y, z, rx, ry, rz}"
    )
    
    # 运动参数
    velocity: float = Field(default=0.5, description="运动速度")
    acceleration: float = Field(default=0.3, description="运动加速度")


# ==================== 升降高度 ====================

class LiftHeight(PresetBase):
    """
    升降高度预设
    
    定义升降电机的常用高度位置
    """
    height: float = Field(..., description="目标高度 (毫米)")
    
    # 可选参数
    speed: float = Field(default=50.0, description="运动速度 (mm/s)")
    
    # 安全限制
    min_height: float = Field(default=0.0, description="最小高度限制")
    max_height: float = Field(default=500.0, description="最大高度限制")


# ==================== 头部位置 ====================

class HeadPosition(PresetBase):
    """
    头部位置预设
    
    定义头部云台的常用朝向
    """
    # 归一化值 (-1.0 到 1.0)
    pan: float = Field(..., description="水平角度 (归一化: -1.0 ~ 1.0)")
    tilt: float = Field(..., description="垂直角度 (归一化: -1.0 ~ 1.0)")
    
    # 或者使用绝对角度
    pan_degrees: Optional[float] = Field(default=None, description="水平角度 (度)")
    tilt_degrees: Optional[float] = Field(default=None, description="垂直角度 (度)")


# ==================== 夹爪位置 ====================

class GripperPosition(PresetBase):
    """
    夹爪位置预设
    
    定义夹爪的开合程度
    """
    side: str = Field(
        default="both",
        description="适用夹爪: left, right, both"
    )
    
    # 开合程度 (0.0=完全关闭, 1.0=完全打开)
    left_position: Optional[float] = Field(
        default=None,
        description="左夹爪位置 (0.0-1.0)"
    )
    right_position: Optional[float] = Field(
        default=None,
        description="右夹爪位置 (0.0-1.0)"
    )
    
    # 力控制
    left_force: Optional[float] = Field(default=None, description="左夹爪夹持力")
    right_force: Optional[float] = Field(default=None, description="右夹爪夹持力")


# ==================== 任务模板 ====================

class TaskTemplate(PresetBase):
    """
    任务模板预设
    
    可复用的任务定义，支持被其他任务引用
    """
    # 任务树 JSON
    task_tree: Dict[str, Any] = Field(..., description="任务行为树 JSON")
    
    # 输入参数定义
    input_params: List[Dict[str, Any]] = Field(
        default_factory=list,
        description="任务输入参数定义"
    )
    
    # 标签
    tags: List[str] = Field(default_factory=list, description="任务标签")
    
    # 版本
    version: str = Field(default="1.0.0", description="任务版本")


# ==================== API 请求/响应模型 ====================

class PresetCreate(BaseModel):
    """创建预设请求"""
    name: str
    description: str = ""
    category: str = "default"
    data: Dict[str, Any]  # 具体预设数据


class PresetUpdate(BaseModel):
    """更新预设请求"""
    name: Optional[str] = None
    description: Optional[str] = None
    category: Optional[str] = None
    data: Optional[Dict[str, Any]] = None


class PresetListResponse(BaseModel):
    """预设列表响应"""
    type: PresetType
    total: int
    items: List[Dict[str, Any]]


class PresetExecuteRequest(BaseModel):
    """执行预设请求"""
    preset_id: str
    override_params: Dict[str, Any] = Field(default_factory=dict)


class CapturePresetRequest(BaseModel):
    """采集当前状态为预设"""
    name: str
    description: str = ""
    category: str = "default"
    # 对于不同类型，可能需要额外参数
    side: Optional[str] = None  # 用于手臂/夹爪，指定采集哪一侧
