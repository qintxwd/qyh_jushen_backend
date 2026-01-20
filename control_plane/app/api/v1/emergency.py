"""
QYH Jushen Control Plane - 紧急停止 API

⚠️ 安全关键功能 - 双通道冗余设计

主通道: Data Plane WebSocket (延迟 < 20ms)
备用通道: 本 HTTP 接口 (延迟 ~ 100ms)

当 WebSocket 连接不可用时，此接口作为最后防线。
符合工业机器人安全标准 (ISO 10218) 的冗余要求。
"""
import logging
from datetime import datetime

from fastapi import APIRouter, Depends

from app.dependencies import get_current_user
from app.models.user import User
from app.schemas.response import ApiResponse, success_response, error_response, ErrorCodes
from app.services.ros2_client import get_ros2_client, ROS2ServiceClient
from app.services.audit_service import AuditService
from app.database import get_async_db
from sqlalchemy.ext.asyncio import AsyncSession

logger = logging.getLogger(__name__)
router = APIRouter()


@router.post(
    "/stop",
    response_model=ApiResponse,
    summary="紧急停止",
    description="""
触发全局紧急停止，立即停止所有运动部件。

**⚠️ 安全关键操作**

此接口会：
1. 发送零速度命令到底盘 (/cmd_vel)
2. 停止所有机械臂运动
3. 停止升降/腰部/头部执行器
4. 记录审计日志

**推荐使用 WebSocket 急停**：
- 正常操作时，建议通过 Data Plane WebSocket 发送 MSG_EMERGENCY_STOP
- WebSocket 延迟更低 (< 20ms vs HTTP ~100ms)

**此接口适用场景**：
- WebSocket 连接断开时的备用方案
- 运维终端/监控系统的紧急干预
- 自动化测试和集成
""",
)
async def emergency_stop(
    current_user: User = Depends(get_current_user),
    db: AsyncSession = Depends(get_async_db),
) -> ApiResponse:
    """执行紧急停止"""
    try:
        ros2_client = get_ros2_client()
        
        # 1. 发送底盘零速度
        chassis_stopped = await ros2_client.publish_emergency_stop()
        
        # 2. 停止机械臂 (如果支持)
        arm_stopped = await ros2_client.stop_arm_motion()
        
        # 3. 停止其他执行器
        actuators_stopped = await ros2_client.stop_all_actuators()
        
        # 4. 记录审计日志
        await AuditService.log(
            db=db,
            action="emergency_stop",
            resource="system",
            resource_id="global",
            details={
                "chassis_stopped": chassis_stopped,
                "arm_stopped": arm_stopped,
                "actuators_stopped": actuators_stopped,
                "timestamp": datetime.utcnow().isoformat(),
                "source": "http_api",
            },
            user=current_user,
        )
        
        logger.warning(
            f"Emergency stop triggered by user {current_user.username} via HTTP API"
        )
        
        return success_response(
            data={
                "stopped": True,
                "chassis": chassis_stopped,
                "arm": arm_stopped,
                "actuators": actuators_stopped,
                "timestamp": datetime.utcnow().isoformat(),
            },
            message="紧急停止已执行，所有运动部件已停止",
        )
        
    except RuntimeError as e:
        # ROS2 未连接
        logger.error(f"Emergency stop failed - ROS2 not available: {e}")
        return error_response(
            code=ErrorCodes.SERVICE_UNAVAILABLE,
            message="ROS2 服务不可用，无法执行紧急停止",
            details={"error": str(e)},
        )
    except Exception as e:
        logger.error(f"Emergency stop failed: {e}")
        return error_response(
            code=ErrorCodes.INTERNAL_ERROR,
            message="紧急停止执行失败",
            details={"error": str(e)},
        )


@router.get(
    "/status",
    response_model=ApiResponse,
    summary="获取急停状态",
    description="查询当前系统是否处于急停状态",
)
async def get_emergency_status(
    current_user: User = Depends(get_current_user),
) -> ApiResponse:
    """获取急停状态"""
    try:
        ros2_client = get_ros2_client()
        
        # 检查各子系统状态
        robot_state = ros2_client.get_robot_state()
        
        # 判断是否处于急停状态
        # 通常通过检查机械臂的 in_estop 标志
        in_estop = False
        if robot_state:
            arm_state = robot_state.get("arm", {})
            in_estop = arm_state.get("in_estop", False)
        
        return success_response(
            data={
                "in_estop": in_estop,
                "can_resume": not in_estop,
            },
            message="急停状态查询成功",
        )
        
    except RuntimeError:
        return success_response(
            data={
                "in_estop": False,
                "can_resume": False,
                "ros2_available": False,
            },
            message="ROS2 未连接，无法获取准确状态",
        )
