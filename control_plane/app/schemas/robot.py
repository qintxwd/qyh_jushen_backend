"""
QYH Jushen Control Plane - 机器人信息 Schema

定义机器人信息相关的响应模型
"""
from typing import Optional, List, Dict, Any
from pydantic import BaseModel, Field


class JointState(BaseModel):
    """关节状态"""
    name: str = Field(..., description="关节名称")
    position: float = Field(default=0.0, description="位置（弧度）")
    velocity: float = Field(default=0.0, description="速度")
    effort: float = Field(default=0.0, description="力矩")


class GripperState(BaseModel):
    """夹爪状态"""
    position: float = Field(default=0.0, description="开合位置 (0-1)")
    force: float = Field(default=0.0, description="夹持力 (N)")
    is_grasping: bool = Field(default=False, description="是否正在抓取")


class BaseState(BaseModel):
    """底盘状态"""
    x: float = Field(default=0.0, description="X 坐标 (m)")
    y: float = Field(default=0.0, description="Y 坐标 (m)")
    theta: float = Field(default=0.0, description="航向角 (rad)")
    linear_velocity: float = Field(default=0.0, description="线速度 (m/s)")
    angular_velocity: float = Field(default=0.0, description="角速度 (rad/s)")


class SystemState(BaseModel):
    """系统状态"""
    cpu_temp: float = Field(default=0.0, description="CPU 温度 (°C)")
    gpu_temp: float = Field(default=0.0, description="GPU 温度 (°C)")
    battery: float = Field(default=100.0, description="电池电量 (%)")
    mode: str = Field(default="idle", description="工作模式")
    uptime_seconds: int = Field(default=0, description="运行时间（秒）")


class SubsystemStatus(BaseModel):
    """子系统状态"""
    name: str = Field(..., description="子系统名称")
    connected: bool = Field(default=False, description="是否连接")
    status: str = Field(default="unknown", description="状态: ok/warning/error/unknown")
    message: str = Field(default="", description="状态消息")
    last_seen: Optional[str] = Field(default=None, description="最后活动时间")


class RobotOverview(BaseModel):
    """机器人状态概览"""
    timestamp: str = Field(..., description="时间戳")
    name: str = Field(default="QYH Jushen", description="机器人名称")
    model: str = Field(default="general", description="机型")
    version: str = Field(default="1.0", description="版本")
    
    # 子系统连接状态
    subsystems: List[SubsystemStatus] = Field(
        default_factory=list,
        description="子系统状态列表"
    )
    
    # 主要组件状态
    left_arm: Optional[Dict[str, Any]] = Field(default=None, description="左臂状态")
    right_arm: Optional[Dict[str, Any]] = Field(default=None, description="右臂状态")
    head: Optional[Dict[str, Any]] = Field(default=None, description="头部状态")
    lift: Optional[Dict[str, Any]] = Field(default=None, description="升降状态")
    waist: Optional[Dict[str, Any]] = Field(default=None, description="腰部状态")
    chassis: Optional[BaseState] = Field(default=None, description="底盘状态")
    left_gripper: Optional[GripperState] = Field(default=None, description="左夹爪")
    right_gripper: Optional[GripperState] = Field(default=None, description="右夹爪")
    
    # 系统状态
    system: Optional[SystemState] = Field(default=None, description="系统状态")


class RobotInfo(BaseModel):
    """机器人基本信息"""
    name: str = Field(default="QYH Jushen", description="机器人名称")
    model: str = Field(default="general", description="机型")
    version: str = Field(default="1.0", description="版本")
    serial_number: str = Field(default="", description="序列号")
    
    # 硬件配置
    has_left_arm: bool = Field(default=True, description="是否有左臂")
    has_right_arm: bool = Field(default=True, description="是否有右臂")
    has_head: bool = Field(default=True, description="是否有头部")
    has_lift: bool = Field(default=True, description="是否有升降")
    has_waist: bool = Field(default=True, description="是否有腰部")
    has_chassis: bool = Field(default=True, description="是否有底盘")
    
    # 关节配置
    left_arm_joints: int = Field(default=7, description="左臂关节数")
    right_arm_joints: int = Field(default=7, description="右臂关节数")
    head_joints: int = Field(default=2, description="头部关节数")
    
    # URDF 路径
    urdf_path: str = Field(default="", description="URDF 文件路径")


class ShutdownState(BaseModel):
    """关机状态"""
    shutdown_in_progress: bool = Field(default=False, description="是否正在关机")
    trigger_source: int = Field(default=0, description="触发源: 0=无, 1=硬件, 2=软件")
    trigger_source_text: str = Field(default="", description="触发源描述")
    countdown_seconds: int = Field(default=-1, description="倒计时（秒）")
    plc_connected: bool = Field(default=False, description="PLC 是否连接")


class ShutdownRequest(BaseModel):
    """关机请求"""
    reason: str = Field(default="user_request", description="关机原因")
    delay_seconds: int = Field(default=0, description="延迟秒数")
