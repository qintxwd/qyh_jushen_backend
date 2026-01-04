"""紧急控制 API"""
from fastapi import APIRouter, Depends
from app.dependencies import get_current_operator
from app.models.user import User
from app.ros2_bridge.bridge import ros2_bridge
from app.core.control_lock import control_lock

router = APIRouter()


@router.post("/stop")
async def emergency_stop(current_user: User = Depends(get_current_operator)):
    """紧急停止所有运动"""
    # 发送急停指令
    ros2_bridge.send_command({
        'type': 'emergency_stop',
        'params': {}
    })
    
    # 释放控制权
    control_lock.force_release()
    
    # TODO: 记录审计日志
    
    return {
        "success": True,
        "message": "所有关节已停止"
    }
