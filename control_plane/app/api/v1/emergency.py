"""
QYH Jushen Control Plane - Emergency Stop API.
"""

import logging
from datetime import datetime

from fastapi import APIRouter, Depends
from sqlalchemy.ext.asyncio import AsyncSession

from app.database import get_async_db
from app.dependencies import get_current_user
from app.models.user import User
from app.schemas.response import ApiResponse, ErrorCodes, error_response, success_response
from app.services.audit_service import AuditService
from app.services.ros2_client import get_ros2_client

logger = logging.getLogger(__name__)
router = APIRouter()


@router.post(
    "/stop",
    response_model=ApiResponse,
    summary="Trigger emergency stop",
)
async def emergency_stop(
    current_user: User = Depends(get_current_user),
    db: AsyncSession = Depends(get_async_db),
) -> ApiResponse:
    """Execute global emergency stop through ROS2 fallback channel."""
    try:
        ros2_client = get_ros2_client()

        chassis_stopped = await ros2_client.publish_emergency_stop()
        arm_stopped = await ros2_client.stop_arm_motion()
        actuators_stopped = await ros2_client.stop_all_actuators()

        all_stopped = bool(chassis_stopped and arm_stopped and actuators_stopped)

        await AuditService.log(
            db=db,
            action="emergency_stop",
            resource="system",
            resource_id="global",
            details={
                "chassis_stopped": chassis_stopped,
                "arm_stopped": arm_stopped,
                "actuators_stopped": actuators_stopped,
                "all_stopped": all_stopped,
                "timestamp": datetime.utcnow().isoformat(),
                "source": "http_api",
            },
            user=current_user,
        )

        logger.warning(
            "Emergency stop triggered by user %s via HTTP API",
            current_user.username,
        )

        data = {
            "stopped": all_stopped,
            "chassis": chassis_stopped,
            "arm": arm_stopped,
            "actuators": actuators_stopped,
            "timestamp": datetime.utcnow().isoformat(),
        }

        if all_stopped:
            return success_response(
                data=data,
                message="Emergency stop executed",
            )

        return error_response(
            code=ErrorCodes.OPERATION_FAILED,
            message="Emergency stop partially failed",
            data=data,
        )

    except RuntimeError as exc:
        logger.error("Emergency stop failed - ROS2 unavailable: %s", exc)
        return error_response(
            code=ErrorCodes.ROS2_NOT_CONNECTED,
            message="ROS2 service unavailable",
            data={"error": str(exc)},
        )
    except Exception as exc:
        logger.exception("Emergency stop failed")
        return error_response(
            code=ErrorCodes.INTERNAL_ERROR,
            message="Emergency stop failed",
            data={"error": str(exc)},
        )


@router.get(
    "/status",
    response_model=ApiResponse,
    summary="Get emergency stop status",
)
async def get_emergency_status(
    current_user: User = Depends(get_current_user),
) -> ApiResponse:
    """Return emergency stop status from ROS2 state caches."""
    _ = current_user
    try:
        ros2_client = get_ros2_client()

        jaka_state = ros2_client.get_jaka_robot_state() or {}
        standard_status = ros2_client.get_standard_robot_status() or {}

        arm_in_estop = bool(jaka_state.get("in_estop", False))
        chassis_in_estop = bool(standard_status.get("is_emergency_stopped", False))
        in_estop = arm_in_estop or chassis_in_estop

        return success_response(
            data={
                "in_estop": in_estop,
                "can_resume": not in_estop,
                "arm_in_estop": arm_in_estop,
                "chassis_in_estop": chassis_in_estop,
                "ros2_available": True,
            },
            message="Emergency status fetched",
        )

    except RuntimeError:
        return success_response(
            data={
                "in_estop": False,
                "can_resume": False,
                "arm_in_estop": False,
                "chassis_in_estop": False,
                "ros2_available": False,
            },
            message="ROS2 unavailable",
        )
