"""底盘手动控制增强模块

提供更精细的底盘手动控制功能，包括：
1. 连续速度控制（支持游戏手柄/摇杆）
2. 方向键控制（支持键盘）
3. 速度曲线平滑
4. 控制状态管理
"""
from fastapi import APIRouter, Depends, WebSocket, WebSocketDisconnect
from pydantic import BaseModel, Field
from typing import Optional, Literal
import asyncio
import time
import math

from app.ros2_bridge.bridge import ros2_bridge
from app.dependencies import get_current_operator
from app.safety.watchdog import watchdog
from app.schemas.response import ApiResponse, success_response, error_response, ErrorCodes

router = APIRouter()


# ==================== 数据模型 ====================

class JoystickInput(BaseModel):
    """摇杆输入（归一化为 -1.0 ~ 1.0）"""
    x: float = Field(0.0, ge=-1.0, le=1.0, description="左右 (-1=左, 1=右)")
    y: float = Field(0.0, ge=-1.0, le=1.0, description="前后 (-1=后, 1=前)")
    rotation: float = Field(0.0, ge=-1.0, le=1.0, description="旋转 (-1=左转, 1=右转)")
    speed_multiplier: float = Field(1.0, ge=0.1, le=1.0, description="速度倍率")


class KeyboardInput(BaseModel):
    """键盘输入"""
    forward: bool = False
    backward: bool = False
    left: bool = False
    right: bool = False
    rotate_left: bool = False
    rotate_right: bool = False
    speed_level: Literal["slow", "normal", "fast"] = "normal"


class ManualControlConfig(BaseModel):
    """手动控制配置"""
    max_linear_speed: float = Field(0.5, ge=0.1, le=1.5, description="最大线速度 (m/s)")
    max_angular_speed: float = Field(0.8, ge=0.1, le=2.0, description="最大角速度 (rad/s)")
    acceleration: float = Field(0.3, ge=0.1, le=1.0, description="加速度 (m/s²)")
    dead_zone: float = Field(0.1, ge=0.0, le=0.3, description="摇杆死区")
    use_exponential_curve: bool = True  # 使用指数曲线使低速更精细


class ManualControlStatus(BaseModel):
    """手动控制状态"""
    enabled: bool = False
    mode: Literal["velocity", "coil", "joystick"] = "velocity"
    current_linear_x: float = 0.0
    current_linear_y: float = 0.0
    current_angular_z: float = 0.0
    last_command_time: Optional[float] = None
    config: ManualControlConfig = ManualControlConfig()


# 全局状态
_manual_control_status = ManualControlStatus()
_last_velocity_time = 0.0
_velocity_timeout = 0.5  # 500ms 无指令则停止


# ==================== 辅助函数 ====================

def _apply_dead_zone(value: float, dead_zone: float) -> float:
    """应用死区"""
    if abs(value) < dead_zone:
        return 0.0
    # 重新映射，使死区外的值从 0 开始
    sign = 1 if value > 0 else -1
    return sign * (abs(value) - dead_zone) / (1.0 - dead_zone)


def _apply_exponential_curve(value: float, exponent: float = 2.0) -> float:
    """应用指数曲线（使低速更精细）"""
    sign = 1 if value >= 0 else -1
    return sign * (abs(value) ** exponent)


def _keyboard_to_velocity(
    input: KeyboardInput, 
    config: ManualControlConfig
) -> tuple[float, float, float]:
    """将键盘输入转换为速度"""
    
    # 速度倍率
    speed_multipliers = {
        "slow": 0.3,
        "normal": 0.6,
        "fast": 1.0
    }
    multiplier = speed_multipliers.get(input.speed_level, 0.6)
    
    linear_x = 0.0
    linear_y = 0.0
    angular_z = 0.0
    
    # 前后
    if input.forward:
        linear_x = config.max_linear_speed * multiplier
    elif input.backward:
        linear_x = -config.max_linear_speed * multiplier
    
    # 左右平移（如果底盘支持）
    if input.left:
        linear_y = config.max_linear_speed * multiplier * 0.5
    elif input.right:
        linear_y = -config.max_linear_speed * multiplier * 0.5
    
    # 旋转
    if input.rotate_left:
        angular_z = config.max_angular_speed * multiplier
    elif input.rotate_right:
        angular_z = -config.max_angular_speed * multiplier
    
    return linear_x, linear_y, angular_z


def _joystick_to_velocity(
    input: JoystickInput, 
    config: ManualControlConfig
) -> tuple[float, float, float]:
    """将摇杆输入转换为速度"""
    
    # 应用死区
    x = _apply_dead_zone(input.x, config.dead_zone)
    y = _apply_dead_zone(input.y, config.dead_zone)
    rotation = _apply_dead_zone(input.rotation, config.dead_zone)
    
    # 应用指数曲线
    if config.use_exponential_curve:
        x = _apply_exponential_curve(x)
        y = _apply_exponential_curve(y)
        rotation = _apply_exponential_curve(rotation)
    
    # 计算速度
    linear_x = y * config.max_linear_speed * input.speed_multiplier
    linear_y = -x * config.max_linear_speed * input.speed_multiplier * 0.5  # 平移减半
    angular_z = -rotation * config.max_angular_speed * input.speed_multiplier
    
    return linear_x, linear_y, angular_z


# ==================== API 端点 ====================

@router.get("/chassis/manual/status", response_model=ApiResponse)
async def get_manual_control_status(current_user=Depends(get_current_operator)):
    """获取手动控制状态"""
    watchdog.heartbeat()
    return success_response(data=_manual_control_status.model_dump())


@router.post("/chassis/manual/config", response_model=ApiResponse)
async def update_manual_config(
    config: ManualControlConfig,
    current_user=Depends(get_current_operator)
):
    """更新手动控制配置"""
    watchdog.heartbeat()
    global _manual_control_status
    _manual_control_status.config = config
    return success_response(data={"message": "配置已更新", "config": config.model_dump()})


@router.post("/chassis/manual/enable", response_model=ApiResponse)
async def enable_manual_control(
    mode: Literal["velocity", "coil", "joystick"] = "velocity",
    current_user=Depends(get_current_operator)
):
    """
    启用手动控制模式
    
    - velocity: 速度模式（推荐，支持摇杆）
    - coil: 线圈模式（简单方向控制）
    - joystick: 摇杆模式（与 velocity 类似，专为游戏手柄优化）
    """
    watchdog.heartbeat()
    global _manual_control_status
    
    result = await ros2_bridge.call_chassis_service("start_manual")
    
    if result is None:
        # Mock 模式
        _manual_control_status.enabled = True
        _manual_control_status.mode = mode
        return success_response(data={
            "message": f"手动{mode}模式已启动 (Mock)",
            "status": _manual_control_status.model_dump()
        })
    
    if result.get('success'):
        _manual_control_status.enabled = True
        _manual_control_status.mode = mode
        return success_response(data={
            "message": f"手动{mode}模式已启动",
            "status": _manual_control_status.model_dump()
        })
    else:
        return error_response(
            code=ErrorCodes.OPERATION_FAILED,
            message=result.get('message', '启动手动控制失败')
        )


@router.post("/chassis/manual/disable", response_model=ApiResponse)
async def disable_manual_control(current_user=Depends(get_current_operator)):
    """禁用手动控制模式"""
    watchdog.heartbeat()
    global _manual_control_status
    
    # 先发送停止速度
    await ros2_bridge.send_chassis_velocity(0, 0, 0)
    
    result = await ros2_bridge.call_chassis_service("stop_manual")
    
    _manual_control_status.enabled = False
    _manual_control_status.current_linear_x = 0
    _manual_control_status.current_linear_y = 0
    _manual_control_status.current_angular_z = 0
    
    if result is None:
        return success_response(data={"message": "手动控制已禁用 (Mock)"})
    
    return success_response(data={
        "message": "手动控制已禁用",
        "status": _manual_control_status.model_dump()
    })


@router.post("/chassis/manual/joystick", response_model=ApiResponse)
async def send_joystick_command(
    input: JoystickInput,
    current_user=Depends(get_current_operator)
):
    """
    发送摇杆控制命令
    
    用于游戏手柄或虚拟摇杆，输入值归一化为 -1.0 ~ 1.0
    需要持续发送（建议 20-50ms 间隔）
    """
    watchdog.heartbeat()
    global _manual_control_status, _last_velocity_time
    
    if not _manual_control_status.enabled:
        return error_response(
            code=ErrorCodes.OPERATION_FAILED,
            message="手动控制未启用，请先调用 /chassis/manual/enable"
        )
    
    linear_x, linear_y, angular_z = _joystick_to_velocity(
        input, _manual_control_status.config
    )
    
    result = await ros2_bridge.send_chassis_velocity(linear_x, linear_y, angular_z)
    
    _manual_control_status.current_linear_x = linear_x
    _manual_control_status.current_linear_y = linear_y
    _manual_control_status.current_angular_z = angular_z
    _manual_control_status.last_command_time = time.time()
    _last_velocity_time = time.time()
    
    if result is None:
        return success_response(data={
            "velocity": {"linear_x": linear_x, "linear_y": linear_y, "angular_z": angular_z}
        })
    
    return success_response(data={
        "velocity": {"linear_x": linear_x, "linear_y": linear_y, "angular_z": angular_z},
        "result": result
    })


@router.post("/chassis/manual/keyboard", response_model=ApiResponse)
async def send_keyboard_command(
    input: KeyboardInput,
    current_user=Depends(get_current_operator)
):
    """
    发送键盘控制命令
    
    用于方向键控制，支持同时按多个键
    需要持续发送（建议 100ms 间隔）
    """
    watchdog.heartbeat()
    global _manual_control_status, _last_velocity_time
    
    if not _manual_control_status.enabled:
        return error_response(
            code=ErrorCodes.OPERATION_FAILED,
            message="手动控制未启用，请先调用 /chassis/manual/enable"
        )
    
    linear_x, linear_y, angular_z = _keyboard_to_velocity(
        input, _manual_control_status.config
    )
    
    result = await ros2_bridge.send_chassis_velocity(linear_x, linear_y, angular_z)
    
    _manual_control_status.current_linear_x = linear_x
    _manual_control_status.current_linear_y = linear_y
    _manual_control_status.current_angular_z = angular_z
    _manual_control_status.last_command_time = time.time()
    _last_velocity_time = time.time()
    
    if result is None:
        return success_response(data={
            "velocity": {"linear_x": linear_x, "linear_y": linear_y, "angular_z": angular_z}
        })
    
    return success_response(data={
        "velocity": {"linear_x": linear_x, "linear_y": linear_y, "angular_z": angular_z},
        "result": result
    })


@router.post("/chassis/manual/stop", response_model=ApiResponse)
async def stop_manual_motion(current_user=Depends(get_current_operator)):
    """立即停止移动（不退出手动模式）"""
    watchdog.heartbeat()
    global _manual_control_status
    
    result = await ros2_bridge.send_chassis_velocity(0, 0, 0)
    
    _manual_control_status.current_linear_x = 0
    _manual_control_status.current_linear_y = 0
    _manual_control_status.current_angular_z = 0
    _manual_control_status.last_command_time = time.time()
    
    if result is None:
        return success_response(data={"message": "已停止 (Mock)"})
    
    return success_response(data={"message": "已停止", "result": result})


# ==================== WebSocket 实时控制 ====================

@router.websocket("/chassis/manual/ws")
async def websocket_manual_control(websocket: WebSocket):
    """
    WebSocket 实时手动控制
    
    发送 JSON:
    - {"type": "joystick", "x": 0.0, "y": 0.5, "rotation": 0.0, "speed_multiplier": 1.0}
    - {"type": "keyboard", "forward": true, "rotate_left": false, ...}
    - {"type": "stop"}
    - {"type": "enable", "mode": "velocity"}
    - {"type": "disable"}
    - {"type": "config", "max_linear_speed": 0.5, ...}
    
    接收 JSON:
    - {"type": "status", "enabled": true, "velocity": {...}, ...}
    - {"type": "error", "message": "..."}
    """
    await websocket.accept()
    
    global _manual_control_status
    
    try:
        while True:
            data = await websocket.receive_json()
            msg_type = data.get("type", "")
            
            try:
                if msg_type == "joystick":
                    input = JoystickInput(
                        x=data.get("x", 0),
                        y=data.get("y", 0),
                        rotation=data.get("rotation", 0),
                        speed_multiplier=data.get("speed_multiplier", 1.0)
                    )
                    
                    if _manual_control_status.enabled:
                        linear_x, linear_y, angular_z = _joystick_to_velocity(
                            input, _manual_control_status.config
                        )
                        await ros2_bridge.send_chassis_velocity(linear_x, linear_y, angular_z)
                        
                        _manual_control_status.current_linear_x = linear_x
                        _manual_control_status.current_linear_y = linear_y
                        _manual_control_status.current_angular_z = angular_z
                        _manual_control_status.last_command_time = time.time()
                        
                        await websocket.send_json({
                            "type": "status",
                            "velocity": {
                                "linear_x": linear_x,
                                "linear_y": linear_y,
                                "angular_z": angular_z
                            }
                        })
                    else:
                        await websocket.send_json({
                            "type": "error",
                            "message": "手动控制未启用"
                        })
                
                elif msg_type == "keyboard":
                    input = KeyboardInput(
                        forward=data.get("forward", False),
                        backward=data.get("backward", False),
                        left=data.get("left", False),
                        right=data.get("right", False),
                        rotate_left=data.get("rotate_left", False),
                        rotate_right=data.get("rotate_right", False),
                        speed_level=data.get("speed_level", "normal")
                    )
                    
                    if _manual_control_status.enabled:
                        linear_x, linear_y, angular_z = _keyboard_to_velocity(
                            input, _manual_control_status.config
                        )
                        await ros2_bridge.send_chassis_velocity(linear_x, linear_y, angular_z)
                        
                        _manual_control_status.current_linear_x = linear_x
                        _manual_control_status.current_linear_y = linear_y
                        _manual_control_status.current_angular_z = angular_z
                        _manual_control_status.last_command_time = time.time()
                        
                        await websocket.send_json({
                            "type": "status",
                            "velocity": {
                                "linear_x": linear_x,
                                "linear_y": linear_y,
                                "angular_z": angular_z
                            }
                        })
                    else:
                        await websocket.send_json({
                            "type": "error",
                            "message": "手动控制未启用"
                        })
                
                elif msg_type == "stop":
                    await ros2_bridge.send_chassis_velocity(0, 0, 0)
                    _manual_control_status.current_linear_x = 0
                    _manual_control_status.current_linear_y = 0
                    _manual_control_status.current_angular_z = 0
                    await websocket.send_json({"type": "stopped"})
                
                elif msg_type == "enable":
                    mode = data.get("mode", "velocity")
                    result = await ros2_bridge.call_chassis_service("start_manual")
                    if result is None or result.get('success'):
                        _manual_control_status.enabled = True
                        _manual_control_status.mode = mode
                        await websocket.send_json({
                            "type": "enabled",
                            "mode": mode
                        })
                    else:
                        await websocket.send_json({
                            "type": "error",
                            "message": result.get('message', '启动失败')
                        })
                
                elif msg_type == "disable":
                    await ros2_bridge.send_chassis_velocity(0, 0, 0)
                    await ros2_bridge.call_chassis_service("stop_manual")
                    _manual_control_status.enabled = False
                    _manual_control_status.current_linear_x = 0
                    _manual_control_status.current_linear_y = 0
                    _manual_control_status.current_angular_z = 0
                    await websocket.send_json({"type": "disabled"})
                
                elif msg_type == "config":
                    config = ManualControlConfig(
                        max_linear_speed=data.get("max_linear_speed", 0.5),
                        max_angular_speed=data.get("max_angular_speed", 0.8),
                        acceleration=data.get("acceleration", 0.3),
                        dead_zone=data.get("dead_zone", 0.1),
                        use_exponential_curve=data.get("use_exponential_curve", True)
                    )
                    _manual_control_status.config = config
                    await websocket.send_json({
                        "type": "config_updated",
                        "config": config.model_dump()
                    })
                
                elif msg_type == "get_status":
                    await websocket.send_json({
                        "type": "status",
                        **_manual_control_status.model_dump()
                    })
                
            except Exception as e:
                await websocket.send_json({
                    "type": "error",
                    "message": str(e)
                })
                
    except WebSocketDisconnect:
        # 断开时停止移动
        if _manual_control_status.enabled:
            await ros2_bridge.send_chassis_velocity(0, 0, 0)
