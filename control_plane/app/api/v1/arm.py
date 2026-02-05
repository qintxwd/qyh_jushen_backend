"""
QYH Jushen Control Plane - 机械臂控制 API

⚠️ 接口职责分离说明（严格遵守架构原则）：

✅ 本文件（FastAPI）只处理低频配置操作 (<5Hz)：
- 电源控制（上电/下电）
- 使能控制（使能/去使能）
- 伺服模式控制（启动/停止）
- 清除错误
- 负载配置管理
- 点位 CRUD 管理
- 状态查询

❌ 以下高频控制必须走 WebSocket (Data Plane)：
- Jog 点动控制 → MSG_ARM_JOG (269)
- MoveJ/MoveL 运动控制 → MSG_ARM_MOVE (268)
- 关节控制 → MSG_JOINT_COMMAND (258)

参考: README.md 中的"接口职责严格分离"章节
"""

import os
import yaml
from typing import Optional, List
from fastapi import APIRouter, Depends, Query
from pydantic import BaseModel, Field

from app.dependencies import get_current_user, get_current_operator
from app.models.user import User
from app.services.ros2_client import get_ros2_client, get_ros2_client_dependency, RobotSide
from app.schemas.response import (
    ApiResponse, success_response, error_response, ErrorCodes
)

router = APIRouter()


# ==================== 数据模型 ====================

class GripperPayloadConfig(BaseModel):
    """夹爪负载配置"""
    left_gripper_mass: float = Field(default=0.8, ge=0, le=5.0)
    right_gripper_mass: float = Field(default=0.8, ge=0, le=5.0)


# 负载配置文件路径
PAYLOAD_CONFIG_PATH = os.path.join(
    os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(__file__)))))),
    "persistent", "preset", "payload_config.yaml"
)

# 当前夹取物品重量 (不持久化)
_current_object_mass = {"left": 0.0, "right": 0.0}


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


def save_gripper_payload_config(left_mass: float, right_mass: float) -> bool:
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


# ==================== 状态查询（低频） ====================

@router.get("/state", response_model=ApiResponse)
async def get_arm_state(current_user: User = Depends(get_current_user)):
    """
    获取机械臂状态
    
    返回：连接状态、上电状态、使能状态、伺服状态、错误信息等
    """
    ros2 = get_ros2_client()
    if not ros2._initialized:
        await ros2.initialize()
    if ros2:
        state = ros2.get_jaka_robot_state()
        if state:
            return success_response(data=state, message="获取机械臂状态成功")
    
    return success_response(
        data={
            "connected": False,
            "robot_ip": "",
            "powered_on": False,
            "enabled": False,
            "in_estop": False,
            "in_error": False,
            "servo_mode_enabled": False,
            "error_message": "ROS2 未连接"
        },
        message="ROS2 未连接"
    )


@router.get("/servo/status", response_model=ApiResponse)
async def get_servo_status(current_user: User = Depends(get_current_user)):
    """获取伺服状态"""
    ros2 = get_ros2_client()
    if not ros2._initialized:
        await ros2.initialize()
    if ros2:
        status = ros2.get_servo_status()
        if status:
            return success_response(data=status, message="获取伺服状态成功")
    
    return success_response(
        data={
            "mode": "idle",
            "is_abs": True,
            "cycle_time_ns": 8000000,
            "publish_rate_hz": 125.0,
        },
        message="获取伺服状态成功"
    )


# ==================== 电源控制（低频配置） ====================

@router.post("/power_on", response_model=ApiResponse)
async def power_on(current_user: User = Depends(get_current_operator)):
    """上电 - 低频配置操作"""
    ros2 = get_ros2_client()
    if not ros2._initialized:
        await ros2.initialize()
    if ros2:
        result = await ros2.arm_power_on()
        if result.success:
            return success_response(message=result.message or "机械臂已上电")
        return error_response(message=result.message, code=ErrorCodes.ROS2_SERVICE_FAILED)
    return error_response(message="ROS2 未连接", code=ErrorCodes.ROS2_NOT_CONNECTED)


@router.post("/power_off", response_model=ApiResponse)
async def power_off(current_user: User = Depends(get_current_operator)):
    """下电 - 低频配置操作"""
    ros2 = get_ros2_client()
    if not ros2._initialized:
        await ros2.initialize()
    if ros2:
        result = await ros2.arm_power_off()
        if result.success:
            return success_response(message=result.message or "机械臂已下电")
        return error_response(message=result.message, code=ErrorCodes.ROS2_SERVICE_FAILED)
    return error_response(message="ROS2 未连接", code=ErrorCodes.ROS2_NOT_CONNECTED)


# ==================== 使能控制（低频配置） ====================

@router.post("/enable", response_model=ApiResponse)
async def enable_arm(current_user: User = Depends(get_current_operator)):
    """使能机械臂 - 低频配置操作"""
    ros2 = get_ros2_client()
    if not ros2._initialized:
        await ros2.initialize()
    if ros2:
        result = await ros2.arm_enable()
        if result.success:
            return success_response(message=result.message or "机械臂已使能")
        return error_response(message=result.message, code=ErrorCodes.ROS2_SERVICE_FAILED)
    return error_response(message="ROS2 未连接", code=ErrorCodes.ROS2_NOT_CONNECTED)


@router.post("/disable", response_model=ApiResponse)
async def disable_arm(current_user: User = Depends(get_current_operator)):
    """去使能机械臂 - 低频配置操作"""
    ros2 = get_ros2_client()
    if not ros2._initialized:
        await ros2.initialize()
    if ros2:
        result = await ros2.arm_disable()
        if result.success:
            return success_response(message=result.message or "机械臂已去使能")
        return error_response(message=result.message, code=ErrorCodes.ROS2_SERVICE_FAILED)
    return error_response(message="ROS2 未连接", code=ErrorCodes.ROS2_NOT_CONNECTED)


# ==================== 错误处理（低频配置） ====================

@router.post("/clear_error", response_model=ApiResponse)
async def clear_error(current_user: User = Depends(get_current_operator)):
    """清除错误 - 低频配置操作"""
    ros2 = get_ros2_client()
    if not ros2._initialized:
        await ros2.initialize()
    if ros2:
        result = await ros2.arm_clear_error()
        if result.success:
            return success_response(message=result.message or "错误已清除")
        return error_response(message=result.message, code=ErrorCodes.ROS2_SERVICE_FAILED)
    return error_response(message="ROS2 未连接", code=ErrorCodes.ROS2_NOT_CONNECTED)


@router.post("/motion_abort", response_model=ApiResponse)
async def motion_abort(current_user: User = Depends(get_current_operator)):
    """
    急停 (HTTP 备用通道)
    
    ⚠️ 主通道是 WebSocket 的 MSG_EMERGENCY_STOP
    HTTP 接口作为备用通道，用于 WebSocket 断开时
    """
    ros2 = get_ros2_client()
    if not ros2._initialized:
        await ros2.initialize()
    if ros2:
        result = await ros2.arm_motion_abort()
        if result.success:
            return success_response(message=result.message or "急停已执行")
        return error_response(message=result.message, code=ErrorCodes.ROS2_SERVICE_FAILED)
    return error_response(message="ROS2 未连接", code=ErrorCodes.ROS2_NOT_CONNECTED)


# ==================== 伺服模式控制（低频配置） ====================

@router.post("/servo/start", response_model=ApiResponse)
async def start_servo(current_user: User = Depends(get_current_operator)):
    """启动伺服模式 - 低频配置操作"""
    ros2 = get_ros2_client()
    if not ros2._initialized:
        await ros2.initialize()
    if ros2:
        result = await ros2.arm_servo_start()
        if result.success:
            return success_response(message=result.message or "伺服模式已启动")
        return error_response(message=result.message, code=ErrorCodes.ROS2_SERVICE_FAILED)
    return error_response(message="ROS2 未连接", code=ErrorCodes.ROS2_NOT_CONNECTED)


@router.post("/servo/stop", response_model=ApiResponse)
async def stop_servo(current_user: User = Depends(get_current_operator)):
    """停止伺服模式 - 低频配置操作"""
    ros2 = get_ros2_client()
    if not ros2._initialized:
        await ros2.initialize()
    if ros2:
        result = await ros2.arm_servo_stop()
        if result.success:
            return success_response(message=result.message or "伺服模式已停止")
        return error_response(message=result.message, code=ErrorCodes.ROS2_SERVICE_FAILED)
    return error_response(message="ROS2 未连接", code=ErrorCodes.ROS2_NOT_CONNECTED)


# ==================== 负载管理（低频配置） ====================

@router.get("/payload/gripper_config", response_model=ApiResponse)
async def get_gripper_payload_config(current_user: User = Depends(get_current_user)):
    """获取夹爪负载配置"""
    config = load_gripper_payload_config()
    return success_response(data=config, message="获取夹爪配置成功")


@router.post("/payload/gripper_config", response_model=ApiResponse)
async def set_gripper_payload_config(
    config: GripperPayloadConfig,
    current_user: User = Depends(get_current_operator)
):
    """设置夹爪负载配置"""
    if save_gripper_payload_config(config.left_gripper_mass, config.right_gripper_mass):
        return success_response(message="夹爪负载配置已保存")
    return error_response(message="保存夹爪负载配置失败", code=ErrorCodes.INTERNAL_ERROR)


@router.post("/payload/apply_gripper", response_model=ApiResponse)
async def apply_gripper_payload(
    robot_id: int = Query(0, ge=0, le=1, description="机器人ID: 0=左臂, 1=右臂"),
    current_user: User = Depends(get_current_operator)
):
    """应用夹爪负载到机械臂"""
    config = load_gripper_payload_config()
    mass = config["left_gripper_mass"] if robot_id == 0 else config["right_gripper_mass"]
    
    ros2 = get_ros2_client()
    if not ros2._initialized:
        await ros2.initialize()
    if ros2:
        result = await ros2.arm_set_payload(robot_id=robot_id, mass=mass)
        if result.success:
            key = "left" if robot_id == 0 else "right"
            _current_object_mass[key] = 0.0
            return success_response(message=f"夹爪负载已应用: {mass} kg")
        return error_response(message=result.message, code=ErrorCodes.ROS2_SERVICE_FAILED)
    
    return success_response(message=f"夹爪负载已应用: {mass} kg (模拟)")


@router.post("/payload/set_object", response_model=ApiResponse)
async def set_object_payload(
    robot_id: int = Query(0, ge=0, le=1),
    object_mass: float = Query(0.0, ge=0, le=5.0),
    current_user: User = Depends(get_current_operator)
):
    """设置夹取物品的重量"""
    config = load_gripper_payload_config()
    gripper_mass = config["left_gripper_mass"] if robot_id == 0 else config["right_gripper_mass"]
    total_mass = gripper_mass + object_mass
    
    ros2 = get_ros2_client()
    if not ros2._initialized:
        await ros2.initialize()
    if ros2:
        result = await ros2.arm_set_payload(robot_id=robot_id, mass=total_mass)
        if result.success:
            key = "left" if robot_id == 0 else "right"
            _current_object_mass[key] = object_mass
            return success_response(
                message=f"负载已更新: 夹爪={gripper_mass}kg + 物品={object_mass}kg = {total_mass}kg"
            )
        return error_response(message=result.message, code=ErrorCodes.ROS2_SERVICE_FAILED)
    
    key = "left" if robot_id == 0 else "right"
    _current_object_mass[key] = object_mass
    return success_response(
        message=f"负载已更新: {total_mass}kg (模拟)"
    )


@router.get("/payload/status", response_model=ApiResponse)
async def get_payload_status(current_user: User = Depends(get_current_user)):
    """获取负载状态"""
    config = load_gripper_payload_config()
    return success_response(
        data={
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
        },
        message="获取负载状态成功"
    )


# ==================== 点位管理 (CRUD - 预设管理) ====================

@router.get("/points", response_model=ApiResponse)
async def get_arm_points(current_user: User = Depends(get_current_user)):
    """获取机械臂点位列表"""
    from app.services.preset_manager import preset_manager
    from app.schemas.preset import PresetType
    
    items = preset_manager.list(PresetType.ARM_POSE, include_builtin=True)
    
    points = []
    for item in items:
        # 兼容性处理：PresetManager 将 data 展开到了 item 根层级
        # 但旧逻辑可能假设在 data 字段中
        src = item
        if "data" in item and isinstance(item["data"], dict) and item["data"]:
            src = item["data"]
            
        points.append({
            "id": item.get("id", ""),
            "name": item.get("name", ""),
            "description": item.get("description", ""),
            "is_builtin": item.get("is_builtin", False),
            "left_joints": src.get("left_joints", [0.0] * 7),
            "right_joints": src.get("right_joints", [0.0] * 7),
        })
    
    return success_response(
        data={"points": points, "total": len(points)},
        message="获取点位列表成功"
    )


@router.post("/points", response_model=ApiResponse)
async def create_arm_point(
    name: str = Query(..., description="点位名称"),
    description: str = Query("", description="点位描述"),
    side: str = Query("both", description="采集哪一侧: left, right, both"),
    current_user: User = Depends(get_current_operator)
):
    """采集新点位（保存当前机械臂位置）"""
    from app.services.preset_manager import preset_manager
    from app.schemas.preset import PresetType
    
    ros2 = get_ros2_client()
    if not ros2:
        return error_response(message="ROS2 未连接", code=ErrorCodes.ROS2_NOT_CONNECTED)
    
    state = ros2.get_joint_states()
    if not state:
        return error_response(message="无法获取关节数据", code=ErrorCodes.ROS2_SERVICE_FAILED)
    
    left_joints = state.get("left_arm", {}).get("joints", [0.0] * 7)
    right_joints = state.get("right_arm", {}).get("joints", [0.0] * 7)
    
    if side == "left":
        right_joints = [0.0] * 7
    elif side == "right":
        left_joints = [0.0] * 7
    
    try:
        preset = preset_manager.create(
            preset_type=PresetType.ARM_POSE,
            name=name,
            data={
                "side": side,
                "pose_type": "joint",
                "left_joints": left_joints,
                "right_joints": right_joints,
                "velocity": 0.5,
                "acceleration": 0.3,
            },
            description=description,
        )
        
        return success_response(
            data={"id": preset.get("id"), "name": preset.get("name")},
            message=f"点位 '{name}' 创建成功"
        )
    except ValueError as e:
        return error_response(ErrorCodes.INVALID_PARAMETER, str(e))


@router.put("/points/{point_id}", response_model=ApiResponse)
async def update_arm_point(
    point_id: str,
    name: Optional[str] = Query(None, description="新名称"),
    description: Optional[str] = Query(None, description="新描述"),
    update_joints: bool = Query(False, description="是否更新关节数据"),
    side: Optional[str] = Query(None, description="更新哪一侧: left, right, both"),
    current_user: User = Depends(get_current_operator)
):
    """更新点位"""
    from app.services.preset_manager import preset_manager
    from app.schemas.preset import PresetType
    
    items = preset_manager.list(PresetType.ARM_POSE, include_builtin=True)
    point = None
    for item in items:
        if item.get("id") == point_id:
            point = item
            break
    
    if not point:
        return error_response(ErrorCodes.RESOURCE_NOT_FOUND, f"点位 '{point_id}' 不存在")
    
    data = point.get("data", {})
    
    if update_joints:
        ros2 = get_ros2_client()
        if not ros2:
            return error_response(message="ROS2 未连接", code=ErrorCodes.ROS2_NOT_CONNECTED)
        
        state = ros2.get_joint_states()
        if not state:
            return error_response(message="无法获取关节数据", code=ErrorCodes.ROS2_SERVICE_FAILED)
        
        update_side = side or "both"
        if update_side in ["left", "both"]:
            data["left_joints"] = state.get("left_arm", {}).get("joints", [0.0] * 7)
        if update_side in ["right", "both"]:
            data["right_joints"] = state.get("right_arm", {}).get("joints", [0.0] * 7)
    
    try:
        updated = preset_manager.update(
            preset_type=PresetType.ARM_POSE,
            preset_id=point_id,
            name=name,
            description=description,
            data=data if update_joints else None,
        )
        
        if updated:
            return success_response(message=f"点位已更新")
        return error_response(message="更新失败", code=ErrorCodes.INTERNAL_ERROR)
    except ValueError as e:
        return error_response(ErrorCodes.INVALID_PARAMETER, str(e))


@router.delete("/points/{point_id}", response_model=ApiResponse)
async def delete_arm_point(
    point_id: str,
    current_user: User = Depends(get_current_operator)
):
    """删除点位"""
    from app.services.preset_manager import preset_manager
    from app.schemas.preset import PresetType
    
    try:
        success = preset_manager.delete(PresetType.ARM_POSE, point_id)
        if success:
            return success_response(message="点位已删除")
        return error_response(message="点位不存在", code=ErrorCodes.RESOURCE_NOT_FOUND)
    except ValueError as e:
        return error_response(ErrorCodes.INVALID_PARAMETER, str(e))


# ==================== 移动到点位 ====================
# ⚠️ 注意：这是一个"发起运动"的操作，严格来说应该走 WebSocket
# 但由于是"移动到预设点位"的配置操作，且不是高频连续控制，
# 暂时保留在 FastAPI 中。如果需要更严格遵循架构，应移至 WebSocket。

@router.post("/points/{point_id}/go", response_model=ApiResponse)
async def go_to_point(
    point_id: str,
    velocity: float = Query(0.5, gt=0, le=3.14),
    acceleration: float = Query(0.3, gt=0, le=10.0),
    side: str = Query("both", description="移动哪一侧: left, right, both"),
    current_user: User = Depends(get_current_operator)
):
    """
    移动到指定点位
    
    ⚠️ 这是一次性运动命令，不是高频控制。
    对于高频 Jog 点动，请使用 WebSocket 的 MSG_ARM_JOG
    """
    from app.services.preset_manager import preset_manager
    from app.schemas.preset import PresetType
    
    items = preset_manager.list(PresetType.ARM_POSE, include_builtin=True)
    point = None
    for item in items:
        if item.get("id") == point_id or item.get("name") == point_id:
            point = item
            break
    
    if not point:
        return error_response(ErrorCodes.RESOURCE_NOT_FOUND, f"点位 '{point_id}' 不存在")
    
    # 兼容性处理：数据通常在根层级，但也可能在 data 字段
    src = point
    if "data" in point and isinstance(point["data"], dict) and point["data"]:
        src = point["data"]
        
    left_joints = src.get("left_joints", [0.0] * 7)
    right_joints = src.get("right_joints", [0.0] * 7)
    
    ros2 = get_ros2_client()
    if not ros2:
        return error_response(message="ROS2 未连接", code=ErrorCodes.ROS2_NOT_CONNECTED)
    
    results = []
    
    if side in ["left", "both"]:
        result = await ros2.arm_move_j(
            robot_id=0,
            joint_positions=left_joints + [0.0] * 7,
            velocity=velocity,
            acceleration=acceleration,
            is_block=False
        )
        results.append({"left": {"success": result.success, "message": result.message}})
    
    if side in ["right", "both"]:
        result = await ros2.arm_move_j(
            robot_id=1,
            joint_positions=[0.0] * 7 + right_joints,
            velocity=velocity,
            acceleration=acceleration,
            is_block=False
        )
        results.append({"right": {"success": result.success, "message": result.message}})
    
    side_text = {"left": "左臂", "right": "右臂", "both": "双臂"}[side]
    return success_response(
        data={"point": point.get("name"), "results": results},
        message=f"正在移动{side_text}到 '{point.get('name')}'"
    )
