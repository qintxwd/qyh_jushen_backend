"""腰部电机控制 API"""
from fastapi import APIRouter, Depends
from pydantic import BaseModel
from typing import Optional
import asyncio

from app.dependencies import get_current_admin
from app.ros2_bridge.bridge import ros2_bridge
from app.safety.watchdog import watchdog
from app.schemas.response import (
    ApiResponse, success_response, error_response, ErrorCodes
)

router = APIRouter()


class WaistControlRequest(BaseModel):
    """腰部电机控制请求"""
    command: int  # 1=使能, 2=去使能, 3=设置速度, 4=去位置, 5=去角度, 6=前倾, 7=后仰, 8=复位, 9=停止, 10=回正
    value: float = 0.0  # 速度、位置或角度值
    hold: bool = False  # 用于手动前倾/后仰


class WaistControlResponse(BaseModel):
    """腰部电机控制响应"""
    success: bool
    message: str


class WaistState(BaseModel):
    """腰部电机状态"""
    connected: bool = False
    enabled: bool = False
    current_position: int = 230715  # 原始位置值
    current_angle: float = 0.0  # 当前角度 (0=竖直, 45=最大前倾)
    current_speed: int = 1000
    position_reached: bool = True
    alarm: bool = False


# 位置常量
POSITION_UPRIGHT = 230715  # 竖直位置
POSITION_MAX_LEAN = 163711  # 最大前倾位置 (45度)
MAX_ANGLE = 45.0


def angle_to_position(angle: float) -> int:
    """角度转位置"""
    angle = max(0.0, min(angle, MAX_ANGLE))
    ratio = angle / MAX_ANGLE
    return int(POSITION_UPRIGHT + ratio * (POSITION_MAX_LEAN - POSITION_UPRIGHT))


def position_to_angle(position: int) -> float:
    """位置转角度"""
    if POSITION_MAX_LEAN == POSITION_UPRIGHT:
        return 0.0
    ratio = (position - POSITION_UPRIGHT) / (POSITION_MAX_LEAN - POSITION_UPRIGHT)
    ratio = max(0.0, min(ratio, 1.0))
    return ratio * MAX_ANGLE


# 模拟状态（实际应从 ROS2 获取）
_mock_state = WaistState()


@router.get("/waist/state", response_model=WaistState)
async def get_waist_state(current_user=Depends(get_current_admin)):
    """获取腰部电机状态"""
    # 视为心跳活动
    watchdog.heartbeat()

    # 尝试从 ROS2 获取状态
    if ros2_bridge.is_connected():
        state = ros2_bridge.get_waist_state()
        if state:
            return WaistState(
                connected=state.get("connected", False),
                enabled=state.get("enabled", False),
                current_position=state.get("current_position", POSITION_UPRIGHT),
                current_angle=state.get("current_angle", 0.0),
                current_speed=state.get("current_speed", 1000),
                position_reached=state.get("position_reached", True),
                alarm=state.get("alarm", False)
            )
    
    # 返回模拟状态（Mock 模式）
    return _mock_state


@router.post("/waist/control", response_model=WaistControlResponse)
async def control_waist(
    request: WaistControlRequest,
    current_user=Depends(get_current_admin)
):
    """控制腰部电机"""
    command_names = {
        1: "使能",
        2: "去使能",
        3: "设置速度",
        4: "去目标位置",
        5: "去目标角度",
        6: "手动前倾",
        7: "手动后仰",
        8: "复位报警",
        9: "停止运动",
        10: "回到竖直"
    }
    
    cmd_name = command_names.get(request.command, f"未知命令({request.command})")
    
    # 尝试通过 ROS2 发送命令
    if ros2_bridge.is_connected():
        try:
            result = await ros2_bridge.call_waist_control(
                command=request.command,
                value=request.value,
                hold=request.hold
            )
            if result:
                return WaistControlResponse(
                    success=result.get("success", False),
                    message=result.get("message", cmd_name + " 完成")
                )
        except Exception as e:
            return WaistControlResponse(
                success=False,
                message=f"{cmd_name} 失败: {str(e)}"
            )
    
    # Mock 模式：模拟执行
    global _mock_state
    
    if request.command == 1:  # 使能
        _mock_state.enabled = True
        _mock_state.connected = True
        return WaistControlResponse(success=True, message="电机已使能")
    
    elif request.command == 2:  # 去使能
        _mock_state.enabled = False
        return WaistControlResponse(success=True, message="电机已去使能")
    
    elif request.command == 3:  # 设置速度
        _mock_state.current_speed = int(request.value)
        return WaistControlResponse(success=True, message=f"速度已设置为 {int(request.value)}")
    
    elif request.command == 4:  # 去目标位置
        if not _mock_state.enabled:
            return WaistControlResponse(success=False, message="电机未使能")
        _mock_state.position_reached = False
        _mock_state.current_position = int(request.value)
        _mock_state.current_angle = position_to_angle(int(request.value))
        _mock_state.position_reached = True
        return WaistControlResponse(success=True, message=f"移动到位置 {int(request.value)}")
    
    elif request.command == 5:  # 去目标角度
        if not _mock_state.enabled:
            return WaistControlResponse(success=False, message="电机未使能")
        angle = max(0.0, min(request.value, MAX_ANGLE))
        _mock_state.position_reached = False
        _mock_state.current_angle = angle
        _mock_state.current_position = angle_to_position(angle)
        _mock_state.position_reached = True
        return WaistControlResponse(success=True, message=f"移动到角度 {angle:.1f}°")
    
    elif request.command == 6:  # 手动前倾
        if not _mock_state.enabled:
            return WaistControlResponse(success=False, message="电机未使能")
        if request.hold:
            return WaistControlResponse(success=True, message="开始前倾")
        else:
            return WaistControlResponse(success=True, message="停止前倾")
    
    elif request.command == 7:  # 手动后仰
        if not _mock_state.enabled:
            return WaistControlResponse(success=False, message="电机未使能")
        if request.hold:
            return WaistControlResponse(success=True, message="开始后仰")
        else:
            return WaistControlResponse(success=True, message="停止后仰")
    
    elif request.command == 8:  # 复位报警
        _mock_state.alarm = False
        return WaistControlResponse(success=True, message="报警已复位")
    
    elif request.command == 9:  # 停止运动
        _mock_state.position_reached = True
        return WaistControlResponse(success=True, message="运动已停止")
    
    elif request.command == 10:  # 回到竖直
        if not _mock_state.enabled:
            return WaistControlResponse(success=False, message="电机未使能")
        _mock_state.position_reached = False
        _mock_state.current_position = POSITION_UPRIGHT
        _mock_state.current_angle = 0.0
        _mock_state.position_reached = True
        return WaistControlResponse(success=True, message="已回到竖直位置")
    
    return WaistControlResponse(success=False, message=f"未知命令: {request.command}")
