"""VR遥操作 API - 提供Clutch状态和控制接口"""
from fastapi import APIRouter, Depends
from pydantic import BaseModel
from typing import Optional, List
from app.dependencies import get_current_admin
from app.ros2_bridge.bridge import ros2_bridge
from app.schemas.response import (
    ApiResponse, success_response, error_response, ErrorCodes
)

router = APIRouter(prefix="/vr", tags=["vr_teleoperation"])


class Pose(BaseModel):
    """位姿模型"""
    position: List[float] = [0.0, 0.0, 0.0]  # [x, y, z]
    orientation: List[float] = [0.0, 0.0, 0.0, 1.0]  # [x, y, z, w] quaternion


class ControllerState(BaseModel):
    """手柄状态模型"""
    active: bool = False
    pose: Pose = Pose()
    joystick: List[float] = [0.0, 0.0]  # [x, y]
    trigger: float = 0.0
    grip: float = 0.0
    buttons: List[int] = [0, 0, 0, 0]  # [Button1, Button2, Menu/Home, JoyClick]
    clutch_engaged: bool = False


class ClutchStatus(BaseModel):
    """Clutch状态模型"""
    left_clutch_engaged: bool = False
    right_clutch_engaged: bool = False
    left_grip_value: float = 0.0
    right_grip_value: float = 0.0


class VRStatus(BaseModel):
    """VR系统整体状态"""
    connected: bool = False
    head_pose: Pose = Pose()
    left_controller: ControllerState = ControllerState()
    right_controller: ControllerState = ControllerState()
    # 保持向后兼容
    left_hand_active: bool = False
    right_hand_active: bool = False
    clutch: ClutchStatus = ClutchStatus()


@router.get("/status", response_model=VRStatus)
async def get_vr_status(current_user=Depends(get_current_admin)):
    """
    获取VR遥操作系统状态
    
    包括:
    - VR连接状态
    - 头部位姿
    - 左/右手控制器状态（位姿、按钮、摇杆、Trigger、Grip）
    - Clutch接合状态
    """
    if not ros2_bridge.is_connected():
        # Mock模式返回模拟数据
        return VRStatus()
    
    # 从ROS2桥接获取实际状态
    vr_state = getattr(ros2_bridge, 'vr_state', None)
    
    if vr_state is None:
        return VRStatus()
    
    # 头部位姿
    head_pose = Pose(
        position=vr_state.get('head_position', [0.0, 0.0, 0.0]),
        orientation=vr_state.get('head_orientation', [0.0, 0.0, 0.0, 1.0])
    )
    
    # 左手控制器
    left_active = vr_state.get('left_hand_active', False)
    left_controller = ControllerState(
        active=left_active,
        pose=Pose(
            position=vr_state.get('left_position', [0.0, 0.0, 0.0]),
            orientation=vr_state.get('left_orientation', [0.0, 0.0, 0.0, 1.0])
        ),
        joystick=vr_state.get('left_joystick', [0.0, 0.0]),
        trigger=vr_state.get('left_trigger', 0.0),
        grip=vr_state.get('left_grip_value', 0.0),
        buttons=vr_state.get('left_buttons', [0, 0, 0, 0]),
        clutch_engaged=vr_state.get('left_clutch_engaged', False)
    )
    
    # 右手控制器
    right_active = vr_state.get('right_hand_active', False)
    right_controller = ControllerState(
        active=right_active,
        pose=Pose(
            position=vr_state.get('right_position', [0.0, 0.0, 0.0]),
            orientation=vr_state.get('right_orientation', [0.0, 0.0, 0.0, 1.0])
        ),
        joystick=vr_state.get('right_joystick', [0.0, 0.0]),
        trigger=vr_state.get('right_trigger', 0.0),
        grip=vr_state.get('right_grip_value', 0.0),
        buttons=vr_state.get('right_buttons', [0, 0, 0, 0]),
        clutch_engaged=vr_state.get('right_clutch_engaged', False)
    )
    
    return VRStatus(
        connected=vr_state.get('connected', False),
        head_pose=head_pose,
        left_controller=left_controller,
        right_controller=right_controller,
        left_hand_active=left_active,
        right_hand_active=right_active,
        clutch=ClutchStatus(
            left_clutch_engaged=left_controller.clutch_engaged,
            right_clutch_engaged=right_controller.clutch_engaged,
            left_grip_value=left_controller.grip,
            right_grip_value=right_controller.grip
        )
    )


@router.get("/clutch", response_model=ClutchStatus)
async def get_clutch_status(current_user=Depends(get_current_admin)):
    """
    获取Clutch状态
    
    返回:
    - left_clutch_engaged: 左手Clutch是否接合
    - right_clutch_engaged: 右手Clutch是否接合
    - left_grip_value: 左手握力值 (0.0-1.0)
    - right_grip_value: 右手握力值 (0.0-1.0)
    """
    if not ros2_bridge.is_connected():
        return ClutchStatus()
    
    vr_state = getattr(ros2_bridge, 'vr_state', None)
    
    if vr_state is None:
        return ClutchStatus()
    
    return ClutchStatus(
        left_clutch_engaged=vr_state.get('left_clutch_engaged', False),
        right_clutch_engaged=vr_state.get('right_clutch_engaged', False),
        left_grip_value=vr_state.get('left_grip_value', 0.0),
        right_grip_value=vr_state.get('right_grip_value', 0.0)
    )
