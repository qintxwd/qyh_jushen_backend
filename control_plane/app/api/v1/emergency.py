"""
QYH Jushen Control Plane - 紧急停止 API

提供紧急停止接口作为 Data Plane WebSocket 的后备方案
"""
from fastapi import APIRouter, Depends

from app.dependencies import get_current_operator
from app.models.user import User
from app.schemas.response import ApiResponse, success_response
from app.core.control_lock import control_lock
from app.services.ros2_client import get_ros2_client
from app.services.audit_service import get_audit_service

router = APIRouter()


@router.post("/stop", response_model=ApiResponse)
async def emergency_stop(current_user: User = Depends(get_current_operator)):
    """
    紧急停止所有运动
    
    这是一个后备接口，正常情况下应该通过 Data Plane 的 WebSocket
    发送 EMERGENCY_STOP 消息（延迟更低）。
    
    此接口用于：
    1. WebSocket 连接失败时的后备方案
    2. 调试和测试
    3. 非实时场景的急停触发
    
    Returns:
        ApiResponse: 统一响应格式
    """
    # 获取 ROS2 客户端
    ros2_client = get_ros2_client()
    
    # 发送急停命令到 ROS2（发布零速度）
    try:
        await ros2_client.publish_emergency_stop()
    except Exception as e:
        # 即使 ROS2 调用失败，也继续释放控制权
        pass
    
    # 强制释放控制权
    old_holder = control_lock.force_release(reason="emergency_stop")
    
    # 记录审计日志
    audit_service = get_audit_service()
    await audit_service.log(
        action="emergency_stop",
        user_id=current_user.id,
        username=current_user.username,
        details={
            "previous_holder": old_holder.get("username") if old_holder else None,
            "trigger": "http_api",
        }
    )
    
    return success_response(
        message="紧急停止已触发，所有运动已停止"
    )


@router.post("/release", response_model=ApiResponse)
async def release_emergency_stop(current_user: User = Depends(get_current_operator)):
    """
    解除紧急停止状态
    
    注意：仅当硬件安全确认后才应调用此接口
    """
    ros2_client = get_ros2_client()
    
    try:
        await ros2_client.release_emergency_stop()
    except Exception as e:
        return success_response(
            message=f"解除急停失败: {str(e)}"
        )
    
    # 记录审计日志
    audit_service = get_audit_service()
    await audit_service.log(
        action="release_emergency_stop",
        user_id=current_user.id,
        username=current_user.username,
        details={
            "trigger": "http_api",
        }
    )
    
    return success_response(
        message="紧急停止已解除"
    )
