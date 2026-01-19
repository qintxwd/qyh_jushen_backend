"""机器人状态 Schema"""
from pydantic import BaseModel
from typing import List, Dict, Any
from datetime import datetime


class JointState(BaseModel):
    """关节状态"""
    left_arm: List[float]
    right_arm: List[float]
    head: List[float]


class GripperState(BaseModel):
    """夹爪状态"""
    position: float
    force: float


class BaseState(BaseModel):
    """底盘状态"""
    x: float
    y: float
    theta: float
    velocity: Dict[str, float]


class SystemState(BaseModel):
    """系统状态"""
    cpu_temp: float
    battery: float
    mode: str


class RobotStatus(BaseModel):
    """机器人完整状态"""
    timestamp: datetime
    joints: JointState
    grippers: Dict[str, GripperState]
    base: BaseState
    system: SystemState
