"""统一状态查询 API - 提供机器人全局状态的一站式查询"""
from fastapi import APIRouter, Depends
from typing import Optional, Dict, Any
from pydantic import BaseModel

from app.ros2_bridge.bridge import ros2_bridge
from app.dependencies import get_current_operator
from app.safety.watchdog import watchdog
from app.schemas.response import ApiResponse, success_response, error_response, ErrorCodes

router = APIRouter(prefix="/status", tags=["status"])


class RobotOverviewResponse(BaseModel):
    """机器人概览响应"""
    ros_connected: bool
    timestamp: float
    
    # 各子系统连接状态
    subsystems: Dict[str, bool]
    
    # 关键指标
    battery_percentage: Optional[float] = None
    is_emergency_stopped: bool = False
    current_task: Optional[str] = None


@router.get("/overview", response_model=ApiResponse)
async def get_overview(current_user=Depends(get_current_operator)):
    """
    获取机器人状态概览
    
    返回所有子系统的连接状态和关键指标，用于仪表板显示
    """
    watchdog.heartbeat()
    
    import time
    
    ros_connected = ros2_bridge.is_connected()
    
    # 收集各子系统状态
    subsystems = {
        "ros2": ros_connected,
        "chassis": False,
        "arm": False,
        "head": False,
        "lift": False,
        "waist": False,
        "left_gripper": False,
        "right_gripper": False,
        "camera_head": False,
        "camera_left": False,
        "camera_right": False,
    }
    
    battery_percentage = None
    is_emergency_stopped = False
    current_task = None
    
    if ros_connected:
        # 底盘状态
        chassis_status = ros2_bridge.get_chassis_status()
        if chassis_status:
            subsystems["chassis"] = chassis_status.get("connected", False)
            battery_percentage = chassis_status.get("battery", {}).get("percentage")
            is_emergency_stopped = chassis_status.get("flags", {}).get("is_emergency_stopped", False)
        
        # 机械臂状态
        arm_state = ros2_bridge.get_arm_state()
        if arm_state:
            subsystems["arm"] = arm_state.get("connected", False)
        
        # 头部状态
        head_state = ros2_bridge.get_head_state()
        if head_state:
            subsystems["head"] = head_state.get("connected", False)
        
        # 升降柱状态
        lift_state = ros2_bridge.get_lift_state()
        if lift_state:
            subsystems["lift"] = lift_state.get("connected", False)
        
        # 腰部状态
        waist_state = ros2_bridge.get_waist_state()
        if waist_state:
            subsystems["waist"] = waist_state.get("connected", False)
        
        # 夹爪状态
        left_gripper = ros2_bridge.get_left_gripper_state()
        if left_gripper:
            subsystems["left_gripper"] = left_gripper.get("communication_ok", False)
        
        right_gripper = ros2_bridge.get_right_gripper_state()
        if right_gripper:
            subsystems["right_gripper"] = right_gripper.get("communication_ok", False)
        
        # 相机状态
        for cam_id, key in [("head", "camera_head"), ("left_hand", "camera_left"), ("right_hand", "camera_right")]:
            cam_status = ros2_bridge.get_camera_status(cam_id)
            if cam_status:
                subsystems[key] = cam_status.get("available", False)
        
        # 任务状态
        task_status = ros2_bridge.get_cached_task_status()
        if task_status and task_status.get("status") in ["running", "paused"]:
            current_task = task_status.get("task_name") or task_status.get("task_id")
    
    return success_response(data={
        "ros_connected": ros_connected,
        "timestamp": time.time(),
        "subsystems": subsystems,
        "battery_percentage": battery_percentage,
        "is_emergency_stopped": is_emergency_stopped,
        "current_task": current_task
    })


@router.get("/all", response_model=ApiResponse)
async def get_all_status(current_user=Depends(get_current_operator)):
    """
    获取所有子系统的详细状态
    
    返回完整的状态信息，用于诊断和调试
    """
    watchdog.heartbeat()
    
    import time
    
    ros_connected = ros2_bridge.is_connected()
    
    result = {
        "timestamp": time.time(),
        "ros_connected": ros_connected,
        "chassis": None,
        "navigation": None,
        "arm": None,
        "servo": None,
        "head": None,
        "lift": None,
        "waist": None,
        "grippers": {
            "left": None,
            "right": None
        },
        "cameras": {},
        "task": None,
        "shutdown": None
    }
    
    if ros_connected:
        result["chassis"] = ros2_bridge.get_chassis_status()
        result["navigation"] = ros2_bridge.get_navigation_status()
        result["arm"] = ros2_bridge.get_arm_state()
        result["servo"] = ros2_bridge.get_servo_status()
        result["head"] = ros2_bridge.get_head_state()
        result["lift"] = ros2_bridge.get_lift_state()
        result["waist"] = ros2_bridge.get_waist_state()
        result["grippers"]["left"] = ros2_bridge.get_left_gripper_state()
        result["grippers"]["right"] = ros2_bridge.get_right_gripper_state()
        result["task"] = ros2_bridge.get_cached_task_status()
        result["shutdown"] = ros2_bridge.get_shutdown_state()
        
        # 相机状态
        for cam_id in ["head", "left_hand", "right_hand"]:
            result["cameras"][cam_id] = ros2_bridge.get_camera_status(cam_id)
    
    return success_response(data=result)


@router.get("/chassis", response_model=ApiResponse)
async def get_chassis_status(current_user=Depends(get_current_operator)):
    """获取底盘详细状态"""
    watchdog.heartbeat()
    
    status = ros2_bridge.get_chassis_status()
    nav_status = ros2_bridge.get_navigation_status()
    
    if status is None:
        # 返回 Mock 数据
        return success_response(data={
            "connected": False,
            "message": "底盘未连接或 ROS2 未启动"
        })
    
    return success_response(data={
        "chassis": status,
        "navigation": nav_status
    })


@router.get("/arm", response_model=ApiResponse)
async def get_arm_status(current_user=Depends(get_current_operator)):
    """获取机械臂详细状态"""
    watchdog.heartbeat()
    
    arm_state = ros2_bridge.get_arm_state()
    servo_status = ros2_bridge.get_servo_status()
    joint_states = ros2_bridge.get_joint_states()
    
    return success_response(data={
        "arm": arm_state,
        "servo": servo_status,
        "joints": joint_states
    })


@router.get("/cameras", response_model=ApiResponse)
async def get_cameras_status(current_user=Depends(get_current_operator)):
    """获取所有相机状态"""
    watchdog.heartbeat()
    
    cameras = {}
    for cam_id in ["head", "left_hand", "right_hand"]:
        cameras[cam_id] = ros2_bridge.get_camera_status(cam_id)
    
    # 统计在线相机数量
    online_count = sum(1 for c in cameras.values() if c and c.get("available"))
    
    return success_response(data={
        "cameras": cameras,
        "online_count": online_count,
        "total_count": len(cameras)
    })


@router.get("/peripherals", response_model=ApiResponse)
async def get_peripherals_status(current_user=Depends(get_current_operator)):
    """获取外设状态（头部、升降柱、腰部、夹爪）"""
    watchdog.heartbeat()
    
    return success_response(data={
        "head": ros2_bridge.get_head_state(),
        "lift": ros2_bridge.get_lift_state(),
        "waist": ros2_bridge.get_waist_state(),
        "grippers": {
            "left": ros2_bridge.get_left_gripper_state(),
            "right": ros2_bridge.get_right_gripper_state()
        }
    })


@router.get("/task", response_model=ApiResponse)
async def get_task_status(current_user=Depends(get_current_operator)):
    """获取当前任务状态"""
    watchdog.heartbeat()
    
    task_status = ros2_bridge.get_cached_task_status()
    
    if task_status is None:
        return success_response(data={
            "has_active_task": False,
            "message": "无正在执行的任务"
        })
    
    return success_response(data={
        "has_active_task": True,
        **task_status
    })


@router.get("/system", response_model=ApiResponse)
async def get_system_status(current_user=Depends(get_current_operator)):
    """获取系统状态（关机状态、看门狗等）"""
    watchdog.heartbeat()
    
    shutdown_state = ros2_bridge.get_shutdown_state()
    
    return success_response(data={
        "shutdown": shutdown_state,
        "ros_connected": ros2_bridge.is_connected(),
        "watchdog_active": True  # 如果能调用此接口，说明看门狗正常
    })
