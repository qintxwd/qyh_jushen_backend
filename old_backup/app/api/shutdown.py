"""系统关机控制 API - 独立模块"""
from fastapi import APIRouter, Depends
from pydantic import BaseModel
from typing import Optional

from app.dependencies import get_current_admin, get_current_user
from app.ros2_bridge.bridge import ros2_bridge
from app.safety.watchdog import watchdog
from app.schemas.response import (
    ApiResponse, success_response, error_response, ErrorCodes
)

router = APIRouter()


class ShutdownResponse(BaseModel):
    """关机响应"""
    success: bool
    message: str


class ShutdownState(BaseModel):
    """关机状态"""
    shutdown_in_progress: bool = False
    trigger_source: int = 0  # 0=无, 1=硬件按钮, 2=软件命令
    trigger_source_text: str = ""
    countdown_seconds: int = -1
    plc_connected: bool = False


@router.get("/shutdown/state", response_model=ShutdownState)
async def get_shutdown_state(current_user=Depends(get_current_user)):
    """获取关机状态
    
    用于前端轮询显示关机进度
    """
    if ros2_bridge.is_connected():
        state = ros2_bridge.get_shutdown_state()
        if state:
            source_text = {
                0: "",
                1: "硬件按钮触发",
                2: "软件命令触发"
            }.get(state.get("trigger_source", 0), "")
            
            return ShutdownState(
                shutdown_in_progress=state.get("shutdown_in_progress", False),
                trigger_source=state.get("trigger_source", 0),
                trigger_source_text=source_text,
                countdown_seconds=state.get("countdown_seconds", -1),
                plc_connected=state.get("plc_connected", False)
            )
    
    # Mock 或未连接
    return ShutdownState()


@router.post("/shutdown", response_model=ShutdownResponse)
async def system_shutdown(current_user=Depends(get_current_admin)):
    """系统关机（通过PLC）
    
    软件关机流程:
    1. Web前端调用此接口
    2. 通过ROS2调用qyh_shutdown服务
    3. 关机节点写PLC关机线圈(M100)
    4. 系统执行shutdown命令
    5. PLC检测到系统关机后10秒断电
    """
    watchdog.heartbeat()
    
    # 尝试通过 ROS2 调用专门的关机服务
    if ros2_bridge.is_connected():
        try:
            result = await ros2_bridge.call_shutdown()
            if result:
                return ShutdownResponse(
                    success=result.get("success", False),
                    message=result.get("message", "系统关机命令已发送")
                )
        except Exception as e:
            return ShutdownResponse(
                success=False,
                message=f"系统关机命令发送失败: {str(e)}"
            )
    
    # Mock 模式
    return ShutdownResponse(
        success=True,
        message="系统关机命令已发送 (Mock模式，未实际关机)"
    )
