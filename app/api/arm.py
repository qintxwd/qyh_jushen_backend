"""
JAKA 双臂机械臂控制 API
提供机械臂状态查询和控制功能
"""

from fastapi import APIRouter, Depends, HTTPException
from pydantic import BaseModel, Field
from typing import Optional, List
from enum import IntEnum

from app.dependencies import get_current_admin
from app.ros2_bridge.bridge import ros2_bridge
from app.safety.watchdog import watchdog
from app.schemas.response import (
    ApiResponse, success_response, error_response, ErrorCodes
)

router = APIRouter()


class RobotId(IntEnum):
    """机器人ID"""
    LEFT = 0
    RIGHT = 1
    DUAL = -1


class ArmState(BaseModel):
    """机械臂状态 - 对应 RobotState.msg"""
    connected: bool = False
    robot_ip: str = ""
    powered_on: bool = False
    enabled: bool = False
    in_estop: bool = False
    in_error: bool = False
    servo_mode_enabled: bool = False  # 伺服模式是否开启
    error_message: str = ""
    left_in_position: bool = True
    right_in_position: bool = True
    # 关节位置 (弧度)
    left_joint_positions: List[float] = [0.0] * 7
    right_joint_positions: List[float] = [0.0] * 7
    # 笛卡尔位姿
    left_cartesian_pose: dict = {}
    right_cartesian_pose: dict = {}


class ServoStatus(BaseModel):
    """伺服状态 - 对应 JakaServoStatus.msg"""
    mode: str = "idle"
    is_abs: bool = True
    cycle_time_ns: int = 8000000
    publish_rate_hz: float = 125.0
    latency_ms: float = 0.0
    packet_loss_rate: float = 0.0
    error_code: int = 0


class ControlResponse(BaseModel):
    """控制响应"""
    success: bool
    message: str


class MoveJRequest(BaseModel):
    """MoveJ 请求"""
    robot_id: int = Field(
        default=0, ge=-1, le=1,
        description="机器人ID: -1=双臂, 0=左臂, 1=右臂"
    )
    joint_positions: List[float] = Field(
        ..., min_length=14, max_length=14,
        description="14个关节位置 (弧度): [左臂7, 右臂7]"
    )
    velocity: float = Field(
        default=0.5, gt=0, le=3.14,
        description="关节速度 (rad/s)"
    )
    acceleration: float = Field(
        default=0.3, gt=0, le=10.0,
        description="关节加速度 (rad/s²)"
    )
    is_block: bool = Field(
        default=True,
        description="是否阻塞等待完成"
    )


class MoveLRequest(BaseModel):
    """MoveL 请求"""
    robot_id: int = Field(
        default=0, ge=0, le=1,
        description="机器人ID: 0=左臂, 1=右臂"
    )
    x: float = Field(..., description="X 位置 (米)")
    y: float = Field(..., description="Y 位置 (米)")
    z: float = Field(..., description="Z 位置 (米)")
    rx: float = Field(default=0.0, description="RX 姿态 (弧度)")
    ry: float = Field(default=0.0, description="RY 姿态 (弧度)")
    rz: float = Field(default=0.0, description="RZ 姿态 (弧度)")
    velocity: float = Field(
        default=100.0, gt=0, le=500.0,
        description="笛卡尔速度 (mm/s)"
    )
    acceleration: float = Field(
        default=50.0, gt=0, le=1000.0,
        description="笛卡尔加速度 (mm/s²)"
    )
    is_block: bool = Field(
        default=True,
        description="是否阻塞等待完成"
    )


class JogRequest(BaseModel):
    """点动控制请求"""
    robot_id: int = Field(
        default=0, ge=0, le=1,
        description="机器人ID: 0=左臂, 1=右臂"
    )
    axis_num: int = Field(
        ..., ge=1, le=7,
        description="轴号: 关节模式1-7, 笛卡尔模式1-6 (X/Y/Z/RX/RY/RZ)"
    )
    move_mode: int = Field(
        default=1, ge=0, le=2,
        description="运动模式: 0=绝对, 1=步进(INCR), 2=连续(CONTINUE)"
    )
    coord_type: int = Field(
        default=1, ge=0, le=2,
        description="坐标类型: 0=基坐标系, 1=关节空间, 2=工具坐标系"
    )
    velocity: float = Field(
        default=0.1,
        description="速度: 关节/旋转 rad/s, 直线 mm/s"
    )
    position: float = Field(
        default=0.01,
        description="位置增量: 关节/旋转 rad, 直线 mm (仅步进模式)"
    )


class JogStopRequest(BaseModel):
    """点动停止请求"""
    robot_id: int = Field(
        default=0, ge=0, le=1,
        description="机器人ID: 0=左臂, 1=右臂"
    )
    axis_num: int = Field(
        default=0, ge=0, le=7,
        description="轴号: 0=停止所有轴"
    )


class SetPayloadRequest(BaseModel):
    """设置负载请求"""
    robot_id: int = Field(
        default=0, ge=0, le=1,
        description="机器人ID: 0=左臂, 1=右臂"
    )
    mass: float = Field(
        ..., ge=0, le=10.0,
        description="总质量 (kg) = 夹爪质量 + 物品质量"
    )


class GetPayloadRequest(BaseModel):
    """获取负载请求"""
    robot_id: int = Field(
        default=0, ge=0, le=1,
        description="机器人ID: 0=左臂, 1=右臂"
    )


class PayloadResponse(BaseModel):
    """负载响应"""
    success: bool
    message: str
    mass: float = 0.0
    centroid_x: float = 0.0
    centroid_y: float = 0.0
    centroid_z: float = 0.0


class GripperPayloadConfig(BaseModel):
    """夹爪负载配置"""
    left_gripper_mass: float = Field(
        default=0.8, ge=0, le=5.0,
        description="左夹爪质量 (kg)"
    )
    right_gripper_mass: float = Field(
        default=0.8, ge=0, le=5.0,
        description="右夹爪质量 (kg)"
    )


# Mock 状态 - 默认未连接
_mock_arm_state = ArmState(
    connected=False,  # 默认未连接，只有真实硬件连接时才为 True
    robot_ip="",
    powered_on=False,
    enabled=False
)
_mock_servo_status = ServoStatus()


@router.get("/arm/state", response_model=ArmState)
async def get_arm_state(current_user=Depends(get_current_admin)):
    """获取机械臂状态"""
    watchdog.heartbeat()
    
    if ros2_bridge.is_connected():
        state = ros2_bridge.get_arm_state()
        if state:
            return ArmState(**state)
    
    # ROS2 未连接或未获取到状态，返回断开状态
    return ArmState(
        connected=False,
        robot_ip="",
        powered_on=False,
        enabled=False,
        in_estop=False,
        in_error=False,
        servo_mode_enabled=False,
        error_message="ROS2 未连接"
    )


@router.get("/arm/servo/status", response_model=ServoStatus)
async def get_servo_status(current_user=Depends(get_current_admin)):
    """获取伺服状态"""
    watchdog.heartbeat()
    
    if ros2_bridge.is_connected():
        status = ros2_bridge.get_servo_status()
        if status:
            return ServoStatus(**status)
    
    return _mock_servo_status


# ========== 连接控制 ==========

@router.post("/arm/connect", response_model=ControlResponse)
async def connect_arm(current_user=Depends(get_current_admin)):
    """连接机械臂"""
    watchdog.heartbeat()
    
    if ros2_bridge.is_connected():
        result = await ros2_bridge.call_arm_service('connect')
        if result:
            return ControlResponse(**result)
    
    # Mock
    _mock_arm_state.connected = True
    return ControlResponse(success=True, message="机器人已连接")


@router.post("/arm/disconnect", response_model=ControlResponse)
async def disconnect_arm(current_user=Depends(get_current_admin)):
    """断开机械臂连接"""
    watchdog.heartbeat()
    
    if ros2_bridge.is_connected():
        result = await ros2_bridge.call_arm_service('disconnect')
        if result:
            return ControlResponse(**result)
    
    # Mock
    _mock_arm_state.connected = False
    _mock_arm_state.powered_on = False
    _mock_arm_state.enabled = False
    _mock_arm_state.servo_mode_enabled = False
    return ControlResponse(success=True, message="机器人已断开连接")


# ========== 基础控制 ==========

@router.post("/arm/power_on", response_model=ControlResponse)
async def power_on(current_user=Depends(get_current_admin)):
    """上电"""
    watchdog.heartbeat()
    
    if ros2_bridge.is_connected():
        result = await ros2_bridge.call_arm_service('power_on')
        if result:
            return ControlResponse(**result)
    
    # Mock
    _mock_arm_state.powered_on = True
    return ControlResponse(success=True, message="机器人已上电")


@router.post("/arm/power_off", response_model=ControlResponse)
async def power_off(current_user=Depends(get_current_admin)):
    """下电"""
    watchdog.heartbeat()
    
    if ros2_bridge.is_connected():
        result = await ros2_bridge.call_arm_service('power_off')
        if result:
            return ControlResponse(**result)
    
    # Mock
    _mock_arm_state.powered_on = False
    _mock_arm_state.enabled = False
    return ControlResponse(success=True, message="机器人已下电")


@router.post("/arm/enable", response_model=ControlResponse)
async def enable_arm(current_user=Depends(get_current_admin)):
    """使能机械臂"""
    watchdog.heartbeat()
    
    if ros2_bridge.is_connected():
        result = await ros2_bridge.call_arm_service('enable')
        if result:
            return ControlResponse(**result)
    
    # Mock
    if not _mock_arm_state.powered_on:
        return ControlResponse(success=False, message="请先上电")
    _mock_arm_state.enabled = True
    return ControlResponse(success=True, message="机器人已使能")


@router.post("/arm/disable", response_model=ControlResponse)
async def disable_arm(current_user=Depends(get_current_admin)):
    """去使能机械臂"""
    watchdog.heartbeat()
    
    if ros2_bridge.is_connected():
        result = await ros2_bridge.call_arm_service('disable')
        if result:
            return ControlResponse(**result)
    
    # Mock
    _mock_arm_state.enabled = False
    return ControlResponse(success=True, message="机器人已去使能")


@router.post("/arm/clear_error", response_model=ControlResponse)
async def clear_error(current_user=Depends(get_current_admin)):
    """清除错误"""
    watchdog.heartbeat()
    
    if ros2_bridge.is_connected():
        result = await ros2_bridge.call_arm_service('clear_error')
        if result:
            return ControlResponse(**result)
    
    # Mock
    _mock_arm_state.in_error = False
    _mock_arm_state.error_message = ""
    return ControlResponse(success=True, message="错误已清除")


@router.post("/arm/motion_abort", response_model=ControlResponse)
async def motion_abort(current_user=Depends(get_current_admin)):
    """急停"""
    watchdog.heartbeat()
    
    if ros2_bridge.is_connected():
        result = await ros2_bridge.call_arm_service('motion_abort')
        if result:
            return ControlResponse(**result)
    
    return ControlResponse(success=True, message="急停已执行")


# ========== 伺服模式控制 ==========

class ServoModeRequest(BaseModel):
    """伺服模式请求"""
    enable: bool = Field(..., description="是否开启伺服模式")


@router.post("/arm/servo/start", response_model=ControlResponse)
async def start_servo(current_user=Depends(get_current_admin)):
    """启动伺服模式"""
    watchdog.heartbeat()
    
    if ros2_bridge.is_connected():
        result = await ros2_bridge.call_arm_service('start_servo')
        if result:
            return ControlResponse(**result)
    
    # Mock
    _mock_servo_status.mode = "joint"
    _mock_arm_state.servo_mode_enabled = True
    return ControlResponse(success=True, message="伺服模式已启动")


@router.post("/arm/servo/stop", response_model=ControlResponse)
async def stop_servo(current_user=Depends(get_current_admin)):
    """停止伺服模式"""
    watchdog.heartbeat()
    
    if ros2_bridge.is_connected():
        result = await ros2_bridge.call_arm_service('stop_servo')
        if result:
            return ControlResponse(**result)
    
    # Mock
    _mock_servo_status.mode = "idle"
    _mock_arm_state.servo_mode_enabled = False
    return ControlResponse(success=True, message="伺服模式已停止")


@router.post("/arm/servo_mode", response_model=ControlResponse)
async def set_servo_mode(
    request: ServoModeRequest,
    current_user=Depends(get_current_admin)
):
    """设置伺服模式（开启/关闭）"""
    watchdog.heartbeat()
    
    if request.enable:
        if ros2_bridge.is_connected():
            result = await ros2_bridge.call_arm_service('start_servo')
            if result:
                return ControlResponse(**result)
        # Mock
        _mock_servo_status.mode = "joint"
        _mock_arm_state.servo_mode_enabled = True
        return ControlResponse(success=True, message="伺服模式已开启")
    else:
        if ros2_bridge.is_connected():
            result = await ros2_bridge.call_arm_service('stop_servo')
            if result:
                return ControlResponse(**result)
        # Mock
        _mock_servo_status.mode = "idle"
        _mock_arm_state.servo_mode_enabled = False
        return ControlResponse(success=True, message="伺服模式已关闭")


# ========== 运动控制 ==========

@router.post("/arm/move_j", response_model=ControlResponse)
async def move_j(
    request: MoveJRequest,
    current_user=Depends(get_current_admin)
):
    """关节空间运动 (MoveJ)"""
    watchdog.heartbeat()
    
    if ros2_bridge.is_connected():
        result = await ros2_bridge.call_move_j(
            robot_id=request.robot_id,
            joint_positions=request.joint_positions,
            velocity=request.velocity,
            acceleration=request.acceleration,
            is_block=request.is_block
        )
        if result:
            return ControlResponse(**result)
    
    # Mock - 更新关节位置
    _mock_arm_state.left_joint_positions = request.joint_positions[:7]
    _mock_arm_state.right_joint_positions = request.joint_positions[7:]
    return ControlResponse(success=True, message="MoveJ 执行成功")


@router.post("/arm/move_l", response_model=ControlResponse)
async def move_l(
    request: MoveLRequest,
    current_user=Depends(get_current_admin)
):
    """笛卡尔空间直线运动 (MoveL)"""
    watchdog.heartbeat()
    
    if ros2_bridge.is_connected():
        result = await ros2_bridge.call_move_l(
            robot_id=request.robot_id,
            x=request.x, y=request.y, z=request.z,
            rx=request.rx, ry=request.ry, rz=request.rz,
            velocity=request.velocity,
            acceleration=request.acceleration,
            is_block=request.is_block
        )
        if result:
            return ControlResponse(**result)
    
    return ControlResponse(success=True, message="MoveL 执行成功")


@router.post("/arm/move_to_zero", response_model=ControlResponse)
async def move_to_zero(current_user=Depends(get_current_admin)):
    """移动到零位"""
    watchdog.heartbeat()
    
    # 调用 MoveJ 移动到零位
    request = MoveJRequest(
        robot_id=-1,  # 双臂
        joint_positions=[0.0] * 14,
        velocity=0.5,
        acceleration=0.3,
        is_block=True
    )
    
    if ros2_bridge.is_connected():
        result = await ros2_bridge.call_move_j(
            robot_id=request.robot_id,
            joint_positions=request.joint_positions,
            velocity=request.velocity,
            acceleration=request.acceleration,
            is_block=request.is_block
        )
        if result:
            return ControlResponse(**result)
    
    # Mock
    _mock_arm_state.left_joint_positions = [0.0] * 7
    _mock_arm_state.right_joint_positions = [0.0] * 7
    return ControlResponse(success=True, message="已移动到零位")


# ========== 点动控制 (Jog) ==========

@router.post("/arm/jog", response_model=ControlResponse)
async def jog(
    request: JogRequest,
    current_user=Depends(get_current_admin)
):
    """点动控制 - 用于手动调节机械臂位置
    
    支持两种模式:
    - 步进模式 (move_mode=1): 点击一次移动固定距离
    - 连续模式 (move_mode=2): 按住持续移动，松开停止
    
    支持两种坐标系:
    - 关节空间 (coord_type=1): axis_num=1-7 对应 J1-J7
    - 笛卡尔空间 (coord_type=0/2): axis_num=1-6 对应 X/Y/Z/RX/RY/RZ
    """
    watchdog.heartbeat()
    
    if ros2_bridge.is_connected():
        result = await ros2_bridge.call_jog(
            robot_id=request.robot_id,
            axis_num=request.axis_num,
            move_mode=request.move_mode,
            coord_type=request.coord_type,
            velocity=request.velocity,
            position=request.position
        )
        if result:
            return ControlResponse(**result)
    
    # Mock
    return ControlResponse(success=True, message="Jog 命令已执行")


@router.post("/arm/jog_stop", response_model=ControlResponse)
async def jog_stop(
    request: JogStopRequest,
    current_user=Depends(get_current_admin)
):
    """停止点动控制"""
    watchdog.heartbeat()
    
    if ros2_bridge.is_connected():
        result = await ros2_bridge.call_jog_stop(
            robot_id=request.robot_id,
            axis_num=request.axis_num
        )
        if result:
            return ControlResponse(**result)
    
    return ControlResponse(success=True, message="Jog 已停止")


# ========== 负载管理 (Payload) ==========

import yaml
import os

# 负载配置文件路径
PAYLOAD_CONFIG_PATH = os.path.join(
    os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(__file__))))),
    "persistent", "preset", "payload_config.yaml"
)


def load_gripper_payload_config() -> dict:
    """加载夹爪负载配置"""
    try:
        if os.path.exists(PAYLOAD_CONFIG_PATH):
            with open(PAYLOAD_CONFIG_PATH, 'r', encoding='utf-8') as f:
                config = yaml.safe_load(f)
                return {
                    "left_gripper_mass": config.get("left_gripper", {}).get("mass", 0.8),
                    "right_gripper_mass": config.get("right_gripper", {}).get("mass", 0.8)
                }
    except Exception as e:
        print(f"Failed to load payload config: {e}")
    return {"left_gripper_mass": 0.8, "right_gripper_mass": 0.8}


def save_gripper_payload_config(left_mass: float, right_mass: float):
    """保存夹爪负载配置"""
    try:
        config = {
            "left_gripper": {"mass": left_mass},
            "right_gripper": {"mass": right_mass}
        }
        os.makedirs(os.path.dirname(PAYLOAD_CONFIG_PATH), exist_ok=True)
        with open(PAYLOAD_CONFIG_PATH, 'w', encoding='utf-8') as f:
            yaml.dump(config, f, allow_unicode=True, default_flow_style=False)
        return True
    except Exception as e:
        print(f"Failed to save payload config: {e}")
        return False


# 当前夹取物品重量 (不持久化)
_current_object_mass = {"left": 0.0, "right": 0.0}


@router.get("/arm/payload/gripper_config", response_model=GripperPayloadConfig)
async def get_gripper_payload_config(current_user=Depends(get_current_admin)):
    """获取夹爪负载配置 (持久化存储)"""
    config = load_gripper_payload_config()
    return GripperPayloadConfig(**config)


@router.post("/arm/payload/gripper_config", response_model=ControlResponse)
async def set_gripper_payload_config(
    config: GripperPayloadConfig,
    current_user=Depends(get_current_admin)
):
    """设置夹爪负载配置 (持久化存储)"""
    if save_gripper_payload_config(config.left_gripper_mass, config.right_gripper_mass):
        return ControlResponse(success=True, message="夹爪负载配置已保存")
    return ControlResponse(success=False, message="保存夹爪负载配置失败")


@router.post("/arm/payload/set", response_model=ControlResponse)
async def set_payload(
    request: SetPayloadRequest,
    current_user=Depends(get_current_admin)
):
    """设置工具负载 (夹爪 + 物品)
    
    注意: 质心固定为 x=150mm (末端向前15cm)
    """
    watchdog.heartbeat()
    
    if ros2_bridge.is_connected():
        result = await ros2_bridge.call_set_payload(
            robot_id=request.robot_id,
            mass=request.mass
        )
        if result:
            return ControlResponse(**result)
    
    return ControlResponse(success=True, message=f"负载设置: {request.mass} kg")


@router.post("/arm/payload/get", response_model=PayloadResponse)
async def get_payload(
    request: GetPayloadRequest,
    current_user=Depends(get_current_admin)
):
    """获取当前工具负载"""
    watchdog.heartbeat()
    
    if ros2_bridge.is_connected():
        result = await ros2_bridge.call_get_payload(
            robot_id=request.robot_id
        )
        if result:
            return PayloadResponse(**result)
    
    return PayloadResponse(success=True, message="Mock", mass=0.8)


@router.post("/arm/payload/apply_gripper", response_model=ControlResponse)
async def apply_gripper_payload(
    robot_id: int = 0,
    current_user=Depends(get_current_admin)
):
    """应用夹爪负载 (设置负载为夹爪重量)
    
    通常在上电使能后调用，防止因负载设置为0导致报错
    """
    watchdog.heartbeat()
    
    config = load_gripper_payload_config()
    mass = config["left_gripper_mass"] if robot_id == 0 else config["right_gripper_mass"]
    
    if ros2_bridge.is_connected():
        result = await ros2_bridge.call_set_payload(
            robot_id=robot_id,
            mass=mass
        )
        if result and result.get("success"):
            # 清除物品重量
            key = "left" if robot_id == 0 else "right"
            _current_object_mass[key] = 0.0
            return ControlResponse(success=True, message=f"夹爪负载已应用: {mass} kg")
        if result:
            return ControlResponse(**result)
    
    return ControlResponse(success=True, message=f"夹爪负载已应用: {mass} kg (Mock)")


@router.post("/arm/payload/set_object", response_model=ControlResponse)
async def set_object_payload(
    robot_id: int = 0,
    object_mass: float = 0.0,
    current_user=Depends(get_current_admin)
):
    """设置夹取物品的重量
    
    实际负载 = 夹爪重量 + 物品重量
    设置 object_mass=0 可清除物品重量
    """
    watchdog.heartbeat()
    
    config = load_gripper_payload_config()
    gripper_mass = config["left_gripper_mass"] if robot_id == 0 else config["right_gripper_mass"]
    total_mass = gripper_mass + object_mass
    
    if ros2_bridge.is_connected():
        result = await ros2_bridge.call_set_payload(
            robot_id=robot_id,
            mass=total_mass
        )
        if result and result.get("success"):
            key = "left" if robot_id == 0 else "right"
            _current_object_mass[key] = object_mass
            return ControlResponse(
                success=True, 
                message=f"负载已更新: 夹爪={gripper_mass}kg + 物品={object_mass}kg = {total_mass}kg"
            )
        if result:
            return ControlResponse(**result)
    
    key = "left" if robot_id == 0 else "right"
    _current_object_mass[key] = object_mass
    return ControlResponse(
        success=True, 
        message=f"负载已更新: 夹爪={gripper_mass}kg + 物品={object_mass}kg = {total_mass}kg (Mock)"
    )


@router.get("/arm/payload/status")
async def get_payload_status(current_user=Depends(get_current_admin)):
    """获取负载状态 (夹爪配置 + 当前物品重量)"""
    config = load_gripper_payload_config()
    return {
        "left": {
            "gripper_mass": config["left_gripper_mass"],
            "object_mass": _current_object_mass["left"],
            "total_mass": config["left_gripper_mass"] + _current_object_mass["left"]
        },
        "right": {
            "gripper_mass": config["right_gripper_mass"],
            "object_mass": _current_object_mass["right"],
            "total_mass": config["right_gripper_mass"] + _current_object_mass["right"]
        }
    }
