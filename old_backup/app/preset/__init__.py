"""
预设资产管理模块 (Preset Manager)

管理机器人各种预设数据的持久化存储和访问:
- 底盘点位 (locations)
- 手臂姿态 (arm_poses)
- 升降高度 (lift_heights)
- 头部位置 (head_positions)
- 夹爪位置 (gripper_positions)
- 任务模板 (task_templates)
"""

from .manager import PresetManager, preset_manager
from .models import (
    Location,
    ArmPose,
    LiftHeight,
    HeadPosition,
    GripperPosition,
    TaskTemplate,
    PresetType
)

__all__ = [
    'PresetManager',
    'preset_manager',
    'Location',
    'ArmPose', 
    'LiftHeight',
    'HeadPosition',
    'GripperPosition',
    'TaskTemplate',
    'PresetType'
]
