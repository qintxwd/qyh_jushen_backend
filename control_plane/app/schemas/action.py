"""
QYH Jushen Control Plane - 动作 Schema

定义动作管理相关的请求/响应模型
"""
from typing import Optional, List, Dict, Any, Literal
from datetime import datetime
from pydantic import BaseModel, Field
import os


# 默认机器人类型和版本
DEFAULT_ROBOT_NAME = os.environ.get('GLOBAL_ROBOT_NAME', 'general')
DEFAULT_ROBOT_VERSION = os.environ.get('GLOBAL_ROBOT_VERSION', '1.0')

# 动作状态类型
ActionStatus = Literal["collecting", "trained"]


class ActionSummary(BaseModel):
    """动作摘要"""
    id: str = Field(..., description="动作 ID")
    name: str = Field(..., description="动作名称")
    description: str = Field(default="", description="描述")
    version: str = Field(default="1.0.0", description="动作版本")
    tags: List[str] = Field(default_factory=list, description="标签")
    status: ActionStatus = Field(default="collecting", description="状态")
    has_model: bool = Field(default=False, description="是否有模型")
    episode_count: int = Field(default=0, description="轨迹数量")
    model_version: Optional[int] = Field(default=0, description="模型版本")
    topics: List[str] = Field(default_factory=list, description="录制话题")
    camera_count: int = Field(default=0, description="相机数量")
    robot_name: str = Field(default=DEFAULT_ROBOT_NAME, description="机器人类型")
    robot_version: str = Field(default=DEFAULT_ROBOT_VERSION, description="机器人版本")
    created_at: str = Field(default="", description="创建时间")
    updated_at: str = Field(default="", description="更新时间")


class ActionDetail(BaseModel):
    """动作详情"""
    id: str
    name: str
    description: str = ""
    version: str = "1.0.0"
    tags: List[str] = []
    status: ActionStatus = "collecting"
    has_model: bool = False
    episode_count: int = 0
    last_training: Optional[str] = None
    model_version: Optional[str] = None
    robot_name: str = DEFAULT_ROBOT_NAME
    robot_version: str = DEFAULT_ROBOT_VERSION
    
    # 采集配置
    collection: Dict[str, Any] = {}
    
    # 训练配置
    training: Dict[str, Any] = {}
    
    # 推理配置
    inference: Dict[str, Any] = {}


class CreateActionRequest(BaseModel):
    """创建动作请求"""
    id: str = Field(..., description="动作 ID（英文，如 pickup_cube）")
    name: str = Field(..., description="显示名称（如 夹取方块）")
    description: str = Field(default="", description="描述")
    template: Optional[str] = Field(default=None, description="模板动作 ID")


class UpdateActionRequest(BaseModel):
    """更新动作请求"""
    name: Optional[str] = Field(default=None, description="显示名称")
    description: Optional[str] = Field(default=None, description="描述")
    tags: Optional[List[str]] = Field(default=None, description="标签")
    collection: Optional[Dict[str, Any]] = Field(default=None, description="采集配置")
    training: Optional[Dict[str, Any]] = Field(default=None, description="训练配置")
    inference: Optional[Dict[str, Any]] = Field(default=None, description="推理配置")


class MarkTrainedRequest(BaseModel):
    """标记已训练请求"""
    model_version: str = Field(default="1.0.0", description="模型版本")


class EpisodeInfo(BaseModel):
    """轨迹信息"""
    id: str = Field(..., description="轨迹 ID")
    name: str = Field(..., description="轨迹名称")
    path: str = Field(..., description="文件路径")
    type: Literal["hdf5", "bag"] = Field(..., description="文件类型")
    size_mb: float = Field(default=0.0, description="文件大小 (MB)")
    created_at: str = Field(default="", description="创建时间")


class InferenceConfig(BaseModel):
    """推理配置"""
    action_id: str
    action_name: str
    model_path: str
    normalization_path: str = ""
    
    # 时间窗口配置
    observation_horizon: int = 10
    action_horizon: int = 20
    action_steps: int = 3
    
    # 控制频率
    control_frequency: float = 20.0
    inference_frequency: float = 10.0
    action_scale: float = 0.4
    smoothing_alpha: float = 0.3
    
    # 安全限制
    max_joint_velocity: float = 1.0
    max_joint_delta: float = 0.05
    gripper_threshold: float = 0.6
    
    # 动作类型
    action_type: str = "absolute"
    
    # 话题配置
    head_camera_topic: str = "/camera/head/color/image_raw"
    head_camera_depth_topic: str = "/camera/head/depth/image_raw"
    right_arm_state_topic: str = "/right_arm/joint_states"
    
    # 功能开关
    use_depth: bool = False
    camera_names: List[str] = []
