"""
QYH Jushen Control Plane - 录制 Schema

定义录制相关的请求/响应模型
"""
from typing import Optional, List
from pydantic import BaseModel, Field


class StartRecordingRequest(BaseModel):
    """开始录制请求"""
    action_name: Optional[str] = Field(None, description="动作名称/ID（如 pickup_cube）")
    action_id: Optional[str] = Field(None, description="前端传递的动作ID")
    user_name: Optional[str] = Field(None, description="用户名")
    version: str = Field(default="1.0", description="版本号")
    topics: List[str] = Field(
        default_factory=list,
        description="要录制的话题列表，空则使用默认话题"
    )
    episode_name: Optional[str] = Field(None, description="前端传递的 episode_name")
    

class RecordingStatus(BaseModel):
    """录制状态"""
    is_recording: bool = Field(default=False, description="是否正在录制")
    status: str = Field(default="idle", description="状态字符串: idle, recording, paused")
    action_name: str = Field(default="", description="当前动作名称")
    user_name: str = Field(default="", description="用户名")
    version: str = Field(default="", description="版本号")
    duration_seconds: float = Field(default=0.0, description="录制时长（秒）")
    bag_path: str = Field(default="", description="录制文件路径")
    topics: List[str] = Field(default_factory=list, description="录制的话题")
    started_at: Optional[str] = Field(default=None, description="开始时间")


class RecordingResult(BaseModel):
    """录制结果"""
    success: bool = Field(..., description="是否成功")
    message: str = Field(default="", description="消息")
    duration_seconds: float = Field(default=0.0, description="录制时长（秒）")
    bag_path: str = Field(default="", description="录制文件路径")


class RecordingFile(BaseModel):
    """录制文件信息"""
    id: str = Field(..., description="文件 ID")
    name: str = Field(..., description="文件名")
    path: str = Field(..., description="文件路径")
    action_name: str = Field(..., description="动作名称")
    user_name: str = Field(default="", description="用户名")
    version: str = Field(default="", description="版本号")
    duration_seconds: float = Field(default=0.0, description="录制时长")
    size_mb: float = Field(default=0.0, description="文件大小 (MB)")
    created_at: str = Field(..., description="创建时间")


class DefaultTopicsConfig(BaseModel):
    """默认录制话题配置"""
    camera_topics: List[str] = Field(
        default_factory=lambda: [
            "/camera/head/color/image_raw/compressed",
            "/camera/left_hand/color/image_raw/compressed",
            "/camera/right_hand/color/image_raw/compressed",
        ],
        description="相机话题"
    )
    joint_topics: List[str] = Field(
        default_factory=lambda: [
            "/joint_states",
            "/left_arm/joint_states",
            "/right_arm/joint_states",
        ],
        description="关节状态话题"
    )
    gripper_topics: List[str] = Field(
        default_factory=lambda: [
            "/left_gripper/state",
            "/right_gripper/state",
        ],
        description="夹爪话题"
    )
    other_topics: List[str] = Field(
        default_factory=lambda: [
            "/tf",
            "/tf_static",
        ],
        description="其他话题"
    )
