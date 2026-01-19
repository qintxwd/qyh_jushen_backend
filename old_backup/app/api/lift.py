"""升降电机控制 API"""
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


class LiftControlRequest(BaseModel):
    """升降电机控制请求"""
    command: int  # 1=使能, 2=去使能, 3=设置速度, 4=去位置, 5=上升, 6=下降, 7=复位, 8=停止, 9=电磁铁
    value: float = 0.0  # 速度或位置值
    hold: bool = False  # 用于手动上升/下降


class LiftControlResponse(BaseModel):
    """升降电机控制响应"""
    success: bool
    message: str


class ElectromagnetRequest(BaseModel):
    """电磁铁控制请求"""
    enable: bool


class LiftState(BaseModel):
    """升降电机状态"""
    connected: bool = False
    enabled: bool = False
    current_position: float = 0.0
    current_speed: float = 20.0
    position_reached: bool = True
    alarm: bool = False
    electromagnet_on: bool = False



# 模拟状态（实际应从 ROS2 获取）
_mock_state = LiftState()


@router.get("/lift/state", response_model=LiftState)
async def get_lift_state(current_user=Depends(get_current_admin)):
    """获取升降电机状态"""
    # 视为心跳活动
    watchdog.heartbeat()

    # 尝试从 ROS2 获取状态
    if ros2_bridge.is_connected():
        state = ros2_bridge.get_lift_state()
        if state:
            return LiftState(
                connected=state.get("connected", False),
                enabled=state.get("enabled", False),
                current_position=state.get("current_position", 0.0),
                current_speed=state.get("current_speed", 20.0),
                position_reached=state.get("position_reached", True),
                alarm=state.get("alarm", False),
                electromagnet_on=state.get("electromagnet_on", False),
)
    
    # 返回模拟状态（Mock 模式）
    return _mock_state


@router.post("/lift/control", response_model=LiftControlResponse)
async def control_lift(
    request: LiftControlRequest,
    current_user=Depends(get_current_admin)
):
    """控制升降电机"""
    command_names = {
        1: "使能",
        2: "去使能",
        3: "设置速度",
        4: "去目标位置",
        5: "手动上升",
        6: "手动下降",
        7: "复位报警",
        8: "停止运动",
        9: "电磁铁"
    }
    
    cmd_name = command_names.get(request.command, f"未知命令({request.command})")
    
    # 尝试通过 ROS2 发送命令
    if ros2_bridge.is_connected():
        try:
            result = await ros2_bridge.call_lift_control(
                command=request.command,
                value=request.value,
                hold=request.hold
            )
            if result:
                return LiftControlResponse(
                    success=result.get("success", False),
                    message=result.get("message", cmd_name + " 完成")
                )
        except Exception as e:
            return LiftControlResponse(
                success=False,
                message=f"{cmd_name} 失败: {str(e)}"
            )
    
    # Mock 模式：模拟执行
    global _mock_state
    
    if request.command == 1:  # 使能
        _mock_state.enabled = True
        _mock_state.connected = True
        return LiftControlResponse(success=True, message="电机已使能")
    
    elif request.command == 2:  # 去使能
        _mock_state.enabled = False
        return LiftControlResponse(success=True, message="电机已去使能")
    
    elif request.command == 3:  # 设置速度
        _mock_state.current_speed = request.value
        return LiftControlResponse(
            success=True,
            message=f"速度已设置为 {request.value}"
        )
    
    elif request.command == 4:  # 去目标位置
        if not _mock_state.enabled:
            return LiftControlResponse(success=False, message="电机未使能")
        _mock_state.position_reached = False
        # 模拟移动（实际由 ROS2 节点处理）
        _mock_state.current_position = request.value
        _mock_state.position_reached = True
        return LiftControlResponse(
            success=True,
            message=f"移动到位置 {request.value}"
        )
    
    elif request.command == 5:  # 手动上升
        if not _mock_state.enabled:
            return LiftControlResponse(success=False, message="电机未使能")
        if request.hold:
            return LiftControlResponse(success=True, message="开始上升")
        else:
            return LiftControlResponse(success=True, message="停止上升")
    
    elif request.command == 6:  # 手动下降
        if not _mock_state.enabled:
            return LiftControlResponse(success=False, message="电机未使能")
        if request.hold:
            return LiftControlResponse(success=True, message="开始下降")
        else:
            return LiftControlResponse(success=True, message="停止下降")
    
    elif request.command == 7:  # 复位报警
        _mock_state.alarm = False
        return LiftControlResponse(success=True, message="报警已复位")
    
    elif request.command == 8:  # 停止运动
        _mock_state.position_reached = True
        return LiftControlResponse(success=True, message="运动已停止")

    elif request.command == 9:  # 电磁铁开关
        return LiftControlResponse(
            success=True,
            message="电磁铁已{}".format("开启" if request.value >= 0.5 else "关闭")
        )
    

    return LiftControlResponse(
        success=False,
        message=f"未知命令: {request.command}"
    )


@router.post("/lift/electromagnet", response_model=LiftControlResponse)
async def control_electromagnet(
    request: ElectromagnetRequest,
    current_user=Depends(get_current_admin)
):
    """电磁铁开关控制"""
    watchdog.heartbeat()

    if ros2_bridge.is_connected():
        try:
            result = await ros2_bridge.call_lift_control(
                command=9,
                value=1.0 if request.enable else 0.0,
                hold=False
            )
            if result:
                return LiftControlResponse(
                    success=result.get("success", False),
                    message=result.get("message", "电磁铁控制完成")
                )
        except Exception as e:
            return LiftControlResponse(
                success=False,
                message=f"电磁铁控制失败: {str(e)}"
            )

    # Mock 模式
    return LiftControlResponse(
        success=True,
        message="电磁铁已{} (Mock)".format("开启" if request.enable else "关闭")
    )


@router.post("/lift/enable", response_model=LiftControlResponse)
async def enable_lift(current_user=Depends(get_current_admin)):
    """使能升降电机（简化接口）"""
    watchdog.heartbeat()
    
    # 调用 control 接口，command=1 表示使能
    if ros2_bridge.is_connected():
        try:
            result = await ros2_bridge.call_lift_control(
                command=1,
                value=0.0,
                hold=False
            )
            if result:
                return LiftControlResponse(
                    success=result.get("success", False),
                    message=result.get("message", "升降已使能")
                )
        except Exception as e:
            return LiftControlResponse(
                success=False,
                message=f"升降使能失败: {str(e)}"
            )
    
    # Mock 模式
    global _mock_state
    _mock_state.enabled = True
    _mock_state.connected = True
    return LiftControlResponse(success=True, message="升降已使能 (Mock)")


