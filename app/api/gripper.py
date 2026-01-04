"""夹爪控制 API"""
from fastapi import APIRouter, HTTPException, Depends
from pydantic import BaseModel
from typing import Optional

from app.dependencies import get_current_admin, get_current_operator
from app.ros2_bridge.bridge import ros2_bridge
from app.safety.watchdog import watchdog

router = APIRouter()


class GripperState(BaseModel):
    """夹爪状态"""
    is_activated: bool = False
    is_moving: bool = False
    object_status: int = 0  # 0:moving, 1:inner_detected, 2:outer_detected(gripped), 3:arrived
    object_status_text: str = "未知"
    target_position: int = 0
    current_position: int = 0
    target_speed: int = 255
    current_speed: int = 0
    target_force: int = 150
    current_force: int = 0
    fault_code: int = 0
    fault_message: str = ""
    communication_ok: bool = False


class DualGripperState(BaseModel):
    """双夹爪状态"""
    left: GripperState
    right: GripperState


class ActivateGripperRequest(BaseModel):
    """激活夹爪请求"""
    side: str  # 'left' or 'right'


class MoveGripperRequest(BaseModel):
    """移动夹爪请求"""
    side: str  # 'left' or 'right'
    position: int  # 0-255: 0=fully open, 255=fully closed
    speed: int = 255  # 0-255
    force: int = 150  # 0-255


class GripperControlResponse(BaseModel):
    """夹爪控制响应"""
    success: bool
    message: str


# 对象状态文本映射
OBJECT_STATUS_TEXT = {
    0: "运动中",
    1: "内撑检测到",
    2: "外夹抓到",
    3: "到达位置"
}


# 模拟状态（Mock 模式）
_mock_left_state = GripperState()
_mock_right_state = GripperState()


def _parse_gripper_state(state: Optional[dict]) -> GripperState:
    """解析夹爪状态"""
    if not state:
        return GripperState()
    
    object_status = state.get("object_status", 0)
    return GripperState(
        is_activated=state.get("is_activated", False),
        is_moving=state.get("is_moving", False),
        object_status=object_status,
        object_status_text=OBJECT_STATUS_TEXT.get(object_status, "未知"),
        target_position=state.get("target_position", 0),
        current_position=state.get("current_position", 0),
        target_speed=state.get("target_speed", 255),
        current_speed=state.get("current_speed", 0),
        target_force=state.get("target_force", 150),
        current_force=state.get("current_force", 0),
        fault_code=state.get("fault_code", 0),
        fault_message=state.get("fault_message", ""),
        communication_ok=state.get("communication_ok", False)
    )


@router.get("/gripper/state", response_model=DualGripperState)
async def get_gripper_state(current_user=Depends(get_current_operator)):
    """获取双夹爪状态"""
    # 视为心跳活动
    watchdog.heartbeat()

    # 尝试从 ROS2 获取状态
    if ros2_bridge.is_connected():
        left_state = ros2_bridge.get_left_gripper_state()
        right_state = ros2_bridge.get_right_gripper_state()
        
        return DualGripperState(
            left=_parse_gripper_state(left_state),
            right=_parse_gripper_state(right_state)
        )
    
    # 返回模拟状态（Mock 模式）
    return DualGripperState(
        left=_mock_left_state,
        right=_mock_right_state
    )


@router.get("/gripper/{side}/state", response_model=GripperState)
async def get_single_gripper_state(
    side: str,
    current_user=Depends(get_current_operator)
):
    """获取单个夹爪状态"""
    if side not in ["left", "right"]:
        raise HTTPException(status_code=400, detail="side 必须是 'left' 或 'right'")
    
    watchdog.heartbeat()

    if ros2_bridge.is_connected():
        if side == "left":
            state = ros2_bridge.get_left_gripper_state()
        else:
            state = ros2_bridge.get_right_gripper_state()
        
        if state:
            return _parse_gripper_state(state)
    
    # Mock 模式
    if side == "left":
        return _mock_left_state
    else:
        return _mock_right_state


@router.post("/gripper/{side}/enable", response_model=GripperControlResponse)
async def enable_gripper(
    side: str,
    current_user=Depends(get_current_operator)
):
    """使能夹爪（激活的别名）"""
    if side not in ["left", "right"]:
        raise HTTPException(status_code=400, detail="side 必须是 'left' 或 'right'")
    
    side_name = "左" if side == "left" else "右"
    
    if ros2_bridge.is_connected():
        try:
            result = await ros2_bridge.call_gripper_activate(side)
            if result:
                return GripperControlResponse(
                    success=result.get("success", False),
                    message=result.get("message", f"{side_name}夹爪使能完成")
                )
        except Exception as e:
            return GripperControlResponse(
                success=False,
                message=f"{side_name}夹爪使能失败: {str(e)}"
            )
    
    # Mock 模式
    global _mock_left_state, _mock_right_state
    
    if side == "left":
        _mock_left_state.is_activated = True
        _mock_left_state.communication_ok = True
    else:
        _mock_right_state.is_activated = True
        _mock_right_state.communication_ok = True
    
    return GripperControlResponse(
        success=True,
        message=f"{side_name}夹爪已使能 (Mock)"
    )


@router.post("/gripper/activate", response_model=GripperControlResponse)
async def activate_gripper(
    request: ActivateGripperRequest,
    current_user=Depends(get_current_operator)
):
    """激活夹爪"""
    if request.side not in ["left", "right"]:
        raise HTTPException(status_code=400, detail="side 必须是 'left' 或 'right'")
    
    side_name = "左" if request.side == "left" else "右"
    
    if ros2_bridge.is_connected():
        try:
            result = await ros2_bridge.call_gripper_activate(request.side)
            if result:
                return GripperControlResponse(
                    success=result.get("success", False),
                    message=result.get("message", f"{side_name}夹爪激活完成")
                )
        except Exception as e:
            return GripperControlResponse(
                success=False,
                message=f"{side_name}夹爪激活失败: {str(e)}"
            )
    
    # Mock 模式
    global _mock_left_state, _mock_right_state
    
    if request.side == "left":
        _mock_left_state.is_activated = True
        _mock_left_state.communication_ok = True
    else:
        _mock_right_state.is_activated = True
        _mock_right_state.communication_ok = True
    
    return GripperControlResponse(
        success=True,
        message=f"{side_name}夹爪已激活 (Mock)"
    )


@router.post("/gripper/move", response_model=GripperControlResponse)
async def move_gripper(
    request: MoveGripperRequest,
    current_user=Depends(get_current_operator)
):
    """移动夹爪"""
    if request.side not in ["left", "right"]:
        raise HTTPException(status_code=400, detail="side 必须是 'left' 或 'right'")
    
    # 验证参数范围
    if not (0 <= request.position <= 255):
        raise HTTPException(status_code=400, detail="position 必须在 0-255 范围内")
    if not (0 <= request.speed <= 255):
        raise HTTPException(status_code=400, detail="speed 必须在 0-255 范围内")
    if not (0 <= request.force <= 255):
        raise HTTPException(status_code=400, detail="force 必须在 0-255 范围内")
    
    side_name = "左" if request.side == "left" else "右"
    
    if ros2_bridge.is_connected():
        try:
            result = await ros2_bridge.call_gripper_move(
                side=request.side,
                position=request.position,
                speed=request.speed,
                force=request.force
            )
            if result:
                return GripperControlResponse(
                    success=result.get("success", False),
                    message=result.get("message", f"{side_name}夹爪移动命令已发送")
                )
        except Exception as e:
            return GripperControlResponse(
                success=False,
                message=f"{side_name}夹爪移动失败: {str(e)}"
            )
    
    # Mock 模式
    global _mock_left_state, _mock_right_state
    
    mock_state = _mock_left_state if request.side == "left" else _mock_right_state
    
    if not mock_state.is_activated:
        return GripperControlResponse(
            success=False,
            message=f"{side_name}夹爪未激活"
        )
    
    mock_state.target_position = request.position
    mock_state.current_position = request.position
    mock_state.target_speed = request.speed
    mock_state.target_force = request.force
    mock_state.object_status = 3  # 到达位置
    mock_state.object_status_text = "到达位置"
    
    return GripperControlResponse(
        success=True,
        message=f"{side_name}夹爪已移动到位置 {request.position} (Mock)"
    )


@router.post("/gripper/{side}/open", response_model=GripperControlResponse)
async def open_gripper(
    side: str,
    current_user=Depends(get_current_operator)
):
    """全开夹爪"""
    return await move_gripper(
        MoveGripperRequest(side=side, position=0, speed=255, force=255),
        current_user
    )


@router.post("/gripper/{side}/close", response_model=GripperControlResponse)
async def close_gripper(
    side: str,
    current_user=Depends(get_current_operator)
):
    """全闭夹爪"""
    return await move_gripper(
        MoveGripperRequest(side=side, position=255, speed=255, force=150),
        current_user
    )


@router.post("/gripper/{side}/half", response_model=GripperControlResponse)
async def half_open_gripper(
    side: str,
    current_user=Depends(get_current_operator)
):
    """半开夹爪"""
    return await move_gripper(
        MoveGripperRequest(side=side, position=128, speed=255, force=150),
        current_user
    )


@router.post("/gripper/{side}/soft", response_model=GripperControlResponse)
async def soft_grip(
    side: str,
    current_user=Depends(get_current_operator)
):
    """柔性抓取"""
    return await move_gripper(
        MoveGripperRequest(side=side, position=255, speed=100, force=50),
        current_user
    )
