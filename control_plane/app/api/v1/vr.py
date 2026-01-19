"""VR遥操作状态查询 API

提供VR系统状态查询接口，包括:
- VR连接状态
- Clutch(离合)状态
- 手柄位姿和按钮状态

注意: 这是低频查询接口(补充WebSocket推送)
"""
from typing import List
from fastapi import APIRouter, Depends
from pydantic import BaseModel, Field

from app.dependencies import get_current_admin
from app.models.user import User
from app.schemas.response import ApiResponse, success_response
from app.services.ros2_client import get_ros2_client_dependency, ROS2ServiceClient

router = APIRouter()


# ==================== 数据模型 ====================

class Pose(BaseModel):
    """位姿模型"""
    position: List[float] = Field(
        default=[0.0, 0.0, 0.0],
        description="位置 [x, y, z]"
    )
    orientation: List[float] = Field(
        default=[0.0, 0.0, 0.0, 1.0],
        description="姿态四元数 [x, y, z, w]"
    )


class ControllerState(BaseModel):
    """VR手柄状态"""
    active: bool = Field(default=False, description="手柄是否激活")
    pose: Pose = Field(default_factory=Pose, description="手柄位姿")
    joystick: List[float] = Field(
        default=[0.0, 0.0], description="摇杆位置 [x, y]"
    )
    trigger: float = Field(default=0.0, description="扳机值 (0.0-1.0)")
    grip: float = Field(default=0.0, description="握力值 (0.0-1.0)")
    buttons: List[int] = Field(
        default=[0, 0, 0, 0],
        description="按钮状态 [Button1, Button2, Menu, JoyClick]"
    )
    clutch_engaged: bool = Field(
        default=False, description="Clutch是否接合"
    )


class ClutchStatus(BaseModel):
    """Clutch状态"""
    left_clutch_engaged: bool = Field(default=False, description="左手Clutch接合")
    right_clutch_engaged: bool = Field(default=False, description="右手Clutch接合")
    left_grip_value: float = Field(default=0.0, description="左手握力 (0.0-1.0)")
    right_grip_value: float = Field(default=0.0, description="右手握力 (0.0-1.0)")


class VRStatus(BaseModel):
    """VR系统状态"""
    connected: bool = Field(default=False, description="VR系统连接状态")
    head_pose: Pose = Field(default_factory=Pose, description="头部位姿")
    left_controller: ControllerState = Field(
        default_factory=ControllerState,
        description="左手控制器"
    )
    right_controller: ControllerState = Field(
        default_factory=ControllerState,
        description="右手控制器"
    )
    # 保持向后兼容
    left_hand_active: bool = Field(
        default=False, description="左手激活状态"
    )
    right_hand_active: bool = Field(
        default=False, description="右手激活状态"
    )
    clutch: ClutchStatus = Field(
        default_factory=ClutchStatus, description="Clutch状态"
    )


# ==================== API 端点 ====================

@router.get("/status", response_model=ApiResponse)
async def get_vr_status(
    current_user: User = Depends(get_current_admin),
    ros2: ROS2ServiceClient = Depends(get_ros2_client_dependency)
) -> ApiResponse:
    """获取VR系统完整状态
    
    包括:
    - VR连接状态
    - 头部位姿
    - 左/右手控制器状态(位姿、按钮、摇杆、Trigger、Grip)
    - Clutch接合状态
    
    权限: 需要管理员权限
    """
    # 从ROS2服务获取VR状态
    state = ros2.get_vr_state()
    
    # 解析头部位姿
    head_pose = Pose(
        position=state.get("head_position", [0.0, 0.0, 0.0]),
        orientation=state.get("head_orientation", [0.0, 0.0, 0.0, 1.0])
    )
    
    # 解析左手控制器
    left_active = state.get("left_hand_active", False)
    left_clutch = state.get("left_clutch_engaged", False)
    left_grip = state.get("left_grip_value", 0.0)
    
    left_controller = ControllerState(
        active=left_active,
        pose=Pose(
            position=state.get("left_position", [0.0, 0.0, 0.0]),
            orientation=state.get("left_orientation", [0.0, 0.0, 0.0, 1.0])
        ),
        joystick=state.get("left_joystick", [0.0, 0.0]),
        trigger=state.get("left_trigger", 0.0),
        grip=left_grip,
        buttons=state.get("left_buttons", [0, 0, 0, 0]),
        clutch_engaged=left_clutch
    )
    
    # 解析右手控制器
    right_active = state.get("right_hand_active", False)
    right_clutch = state.get("right_clutch_engaged", False)
    right_grip = state.get("right_grip_value", 0.0)
    
    right_controller = ControllerState(
        active=right_active,
        pose=Pose(
            position=state.get("right_position", [0.0, 0.0, 0.0]),
            orientation=state.get("right_orientation", [0.0, 0.0, 0.0, 1.0])
        ),
        joystick=state.get("right_joystick", [0.0, 0.0]),
        trigger=state.get("right_trigger", 0.0),
        grip=right_grip,
        buttons=state.get("right_buttons", [0, 0, 0, 0]),
        clutch_engaged=right_clutch
    )
    
    status = VRStatus(
        connected=state.get("connected", False),
        head_pose=head_pose,
        left_controller=left_controller,
        right_controller=right_controller,
        left_hand_active=left_active,
        right_hand_active=right_active,
        clutch=ClutchStatus(
            left_clutch_engaged=left_clutch,
            right_clutch_engaged=right_clutch,
            left_grip_value=left_grip,
            right_grip_value=right_grip
        )
    )
    return success_response(data=status.model_dump())


@router.get("/clutch", response_model=ApiResponse)
async def get_clutch_status(
    current_user: User = Depends(get_current_admin),
    ros2: ROS2ServiceClient = Depends(get_ros2_client_dependency)
) -> ApiResponse:
    """获取VR Clutch状态
    
    Clutch用于VR遥操作的"离合"功能:
    - 握紧时: 机械臂跟随手柄运动
    - 松开时: 机械臂保持当前位置，可以重新定位手柄
    
    返回:
    - left/right_clutch_engaged: Clutch是否接合
    - left/right_grip_value: 握力值 (0.0-1.0)
    
    权限: 需要管理员权限
    """
    # 从ROS2服务获取VR状态
    state = ros2.get_vr_state()

    clutch = ClutchStatus(
        left_clutch_engaged=state.get("left_clutch_engaged", False),
        right_clutch_engaged=state.get("right_clutch_engaged", False),
        left_grip_value=state.get("left_grip_value", 0.0),
        right_grip_value=state.get("right_grip_value", 0.0)
    )
    return success_response(data=clutch.model_dump())
