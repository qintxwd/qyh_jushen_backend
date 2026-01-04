"""
头部舵机控制 API
控制 Pan (左右) 和 Tilt (上下) 两个舵机
"""

from fastapi import APIRouter, Depends, HTTPException
from pydantic import BaseModel, Field
from typing import Optional

from app.dependencies import get_current_admin
from app.ros2_bridge.bridge import ros2_bridge
from app.safety.watchdog import watchdog

router = APIRouter()


class HeadState(BaseModel):
    """头部状态"""
    connected: bool = False
    pan_position: float = 500  # 0-1000
    tilt_position: float = 500  # 0-1000
    pan_normalized: float = 0  # -1.0 到 1.0
    tilt_normalized: float = 0  # -1.0 到 1.0


class HeadControlRequest(BaseModel):
    """头部控制请求"""
    pan: Optional[float] = Field(
        None, ge=-1.0, le=1.0,
        description="Pan 归一化位置 (-1.0 到 1.0)"
    )
    tilt: Optional[float] = Field(
        None, ge=-1.0, le=1.0,
        description="Tilt 归一化位置 (-1.0 到 1.0)"
    )


class HeadControlResponse(BaseModel):
    """控制响应"""
    success: bool
    message: str


# Mock 状态
_mock_state = HeadState()


@router.get("/head/state", response_model=HeadState)
async def get_head_state(current_user=Depends(get_current_admin)):
    """获取头部状态"""
    watchdog.heartbeat()
    
    if ros2_bridge.is_connected():
        state = ros2_bridge.get_head_state()
        if state:
            return HeadState(**state)
    
    # Mock 或未连接时返回模拟状态
    return _mock_state


@router.post("/head/control", response_model=HeadControlResponse)
async def control_head(
    request: HeadControlRequest,
    current_user=Depends(get_current_admin)
):
    """控制头部位置"""
    watchdog.heartbeat()
    
    if ros2_bridge.is_connected():
        try:
            result = await ros2_bridge.send_head_command(
                pan=request.pan,
                tilt=request.tilt
            )
            if result:
                return HeadControlResponse(**result)
        except Exception as e:
            raise HTTPException(status_code=500, detail=str(e))
    
    # Mock 模式
    if request.pan is not None:
        _mock_state.pan_normalized = request.pan
        _mock_state.pan_position = 500 + request.pan * 400  # 映射到 100-900
    if request.tilt is not None:
        _mock_state.tilt_normalized = request.tilt
        _mock_state.tilt_position = 500 + request.tilt * 300  # 映射到 200-800
    
    return HeadControlResponse(success=True, message="头部控制已执行")


@router.post("/head/enable", response_model=HeadControlResponse)
async def enable_head(current_user=Depends(get_current_admin)):
    """使能头部（头部电机默认使能，此接口用于兼容）"""
    watchdog.heartbeat()
    
    # 头部舵机默认使能，直接返回成功
    _mock_state.connected = True
    return HeadControlResponse(success=True, message="头部已使能")


@router.post("/head/reset", response_model=HeadControlResponse)
async def reset_head(current_user=Depends(get_current_admin)):
    """头部回正"""
    watchdog.heartbeat()
    
    if ros2_bridge.is_connected():
        try:
            result = await ros2_bridge.send_head_command(pan=0.0, tilt=0.0)
            if result:
                return HeadControlResponse(**result)
        except Exception as e:
            raise HTTPException(status_code=500, detail=str(e))
    
    # Mock 模式
    _mock_state.pan_position = 500
    _mock_state.tilt_position = 500
    _mock_state.pan_normalized = 0
    _mock_state.tilt_normalized = 0
    
    return HeadControlResponse(success=True, message="头部已回正")
