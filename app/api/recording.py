"""
录包控制 API

提供 rosbag 录制的控制接口
"""

from fastapi import APIRouter, HTTPException
from pydantic import BaseModel, Field
from typing import List
import logging

logger = logging.getLogger(__name__)

router = APIRouter(prefix="/recording", tags=["recording"])


class StartRecordingRequest(BaseModel):
    """开始录制请求"""
    action_name: str = Field(..., description="动作名称")
    user_name: str = Field(..., description="用户名")
    version: str = Field(..., description="版本号")
    topics: List[str] = Field(
        ..., 
        min_length=1,
        description="要录制的话题列表（必填）"
    )


class StartRecordingResponse(BaseModel):
    """开始录制响应"""
    success: bool
    message: str
    bag_path: str = ""


class StopRecordingResponse(BaseModel):
    """停止录制响应"""
    success: bool
    message: str
    duration_sec: float = 0.0
    bag_path: str = ""


class RecordingStatusResponse(BaseModel):
    """录制状态响应"""
    is_recording: bool
    action_name: str = ""
    duration_sec: float = 0.0
    bag_path: str = ""
    topics: List[str] = []


# ROS2 服务客户端（延迟初始化）
_ros_client = None


def get_ros_client():
    """获取 ROS2 客户端"""
    global _ros_client
    if _ros_client is None:
        try:
            from .ros_client import ROSRecordingClient
            _ros_client = ROSRecordingClient()
        except Exception as e:
            logger.warning(f"ROS2 客户端初始化失败: {e}")
            _ros_client = None
    return _ros_client


@router.post("/start", response_model=StartRecordingResponse)
async def start_recording(request: StartRecordingRequest):
    """
    开始录制
    
    - action_name: 动作 ID（如 pickup_cube，用于确定保存目录）
    - user_name: 用户名
    - version: 版本号
    - topics: 要录制的话题列表
    
    录制文件将保存到: ~/qyh-robot-system/model_actions/{action_name}/data/bags/qyh_{user_name}_{version}_{timestamp}/
    """
    logger.info(f"开始录制请求: action={request.action_name}, user={request.user_name}, version={request.version}")
    
    client = get_ros_client()
    if client is None:
        # 模拟模式
        logger.warning("ROS2 客户端不可用，使用模拟模式")
        return StartRecordingResponse(
            success=True,
            message="录制已开始 (模拟模式)",
            bag_path=f"/tmp/qyh_{request.user_name}_{request.version}_simulated"
        )
    
    try:
        result = await client.start_recording(
            action_name=request.action_name,
            user_name=request.user_name,
            version=request.version,
            topics=request.topics or []
        )
        return StartRecordingResponse(**result)
    except Exception as e:
        logger.error(f"开始录制失败: {e}")
        raise HTTPException(status_code=500, detail=str(e))


@router.post("/stop", response_model=StopRecordingResponse)
async def stop_recording():
    """
    停止录制
    
    返回录制时长和保存路径
    """
    logger.info("停止录制请求")
    
    client = get_ros_client()
    if client is None:
        # 模拟模式
        return StopRecordingResponse(
            success=True,
            message="录制已停止 (模拟模式)",
            duration_sec=10.0,
            bag_path="/tmp/simulated_bag"
        )
    
    try:
        result = await client.stop_recording()
        return StopRecordingResponse(**result)
    except Exception as e:
        logger.error(f"停止录制失败: {e}")
        raise HTTPException(status_code=500, detail=str(e))


@router.get("/status", response_model=RecordingStatusResponse)
async def get_recording_status():
    """
    获取录制状态
    """
    client = get_ros_client()
    if client is None:
        # 模拟模式
        return RecordingStatusResponse(
            is_recording=False,
            action_name="",
            duration_sec=0.0,
            bag_path="",
            topics=[]
        )
    
    try:
        result = await client.get_status()
        return RecordingStatusResponse(**result)
    except Exception as e:
        logger.error(f"获取录制状态失败: {e}")
        raise HTTPException(status_code=500, detail=str(e))


@router.post("/discard", response_model=StopRecordingResponse)
async def discard_recording():
    """
    丢弃当前录制
    
    停止录制并删除录制的文件
    """
    logger.info("丢弃录制请求")
    
    client = get_ros_client()
    if client is None:
        return StopRecordingResponse(
            success=True,
            message="录制已丢弃 (模拟模式)",
            duration_sec=0.0,
            bag_path=""
        )
    
    try:
        # 先停止录制
        result = await client.stop_recording()
        
        if result.get("success") and result.get("bag_path"):
            # 删除录制的文件
            import shutil
            import os
            bag_path = result["bag_path"]
            if os.path.exists(bag_path):
                shutil.rmtree(bag_path)
                logger.info(f"已删除录制文件: {bag_path}")
                result["message"] = "录制已丢弃"
                result["bag_path"] = ""
        
        return StopRecordingResponse(**result)
    except Exception as e:
        logger.error(f"丢弃录制失败: {e}")
        raise HTTPException(status_code=500, detail=str(e))


@router.get("/topics")
async def get_available_topics():
    """
    获取可用于录制的话题列表
    """
    client = get_ros_client()
    if client is None:
        # 返回默认话题列表
        return {
            "topics": [
                "/joint_states",
                "/tf",
                "/tf_static",
                "/left_arm/joint_states",
                "/right_arm/joint_states",
                "/left_gripper/state",
                "/right_gripper/state",
                "/head/joint_states",
                "/lift/state",
                "/chassis/odom"
            ]
        }
    
    try:
        topics = await client.get_available_topics()
        return {"topics": topics}
    except Exception as e:
        logger.error(f"获取话题列表失败: {e}")
        return {"topics": []}
