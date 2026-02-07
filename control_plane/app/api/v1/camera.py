"""
QYH Jushen Control Plane - 摄像头配置 API

提供摄像头列表查询，用于前端选择 WebRTC 视频流源。

注意：实际视频流通过 Media Plane (WebRTC) 传输，
      本接口仅提供摄像头元数据查询。
"""
import json
import os
from pathlib import Path
from typing import List, Optional

from fastapi import APIRouter, Depends, HTTPException
from pydantic import BaseModel, Field

from app.dependencies import get_current_user
from app.models.user import User
from app.schemas.response import ApiResponse, success_response, error_response, ErrorCodes

router = APIRouter()


# ==================== 数据模型 ====================

class CameraInfo(BaseModel):
    """摄像头信息"""
    id: str = Field(..., description="摄像头唯一标识")
    name: str = Field(..., description="摄像头名称 (显示用)")
    type: str = Field(
        default="rgb",
        description="摄像头类型: rgb, depth, ir, fisheye"
    )
    topic: str = Field(..., description="ROS2 图像话题")
    width: int = Field(default=640, description="图像宽度")
    height: int = Field(default=480, description="图像高度")
    fps: int = Field(default=30, description="帧率")
    encoding: str = Field(
        default="bgr8",
        description="图像编码格式: bgr8, rgb8, mono8, 16UC1"
    )
    webrtc_track_id: Optional[str] = Field(
        default=None,
        description="WebRTC track ID (由 Media Plane 分配)"
    )


class CameraListResponse(BaseModel):
    """摄像头列表响应"""
    cameras: List[CameraInfo]
    current_streaming: Optional[str] = Field(
        default=None,
        description="当前正在推流的摄像头 ID"
    )


# ==================== 配置管理 ====================

def _get_camera_config_file() -> Path:
    """获取摄像头配置文件路径"""
    workspace_root = Path(os.environ.get('QYH_WORKSPACE_ROOT', Path.home() / 'qyh-robot-system'))
    config_dir = workspace_root / "persistent" / "web"
    config_dir.mkdir(parents=True, exist_ok=True)
    return config_dir / "camera_config.json"


def _load_camera_config() -> dict:
    """加载摄像头配置"""
    config_file = _get_camera_config_file()
    if config_file.exists():
        try:
            with open(config_file, 'r', encoding='utf-8') as f:
                return json.load(f)
        except Exception:
            pass
    
    # 默认配置 - 基于实际机器人配置
    return {
        "cameras": [
            {
                "id": "head_rgb",
                "name": "头部 RGB 摄像头",
                "type": "rgb",
                "topic": "/head_camera/color/image_raw",
                "width": 1280,
                "height": 720,
                "fps": 30,
                "encoding": "bgr8"
            },
            {
                "id": "head_depth",
                "name": "头部深度摄像头",
                "type": "depth",
                "topic": "/head_camera/depth/image_rect_raw",
                "width": 640,
                "height": 480,
                "fps": 30,
                "encoding": "16UC1"
            },
            {
                "id": "left_wrist",
                "name": "左手腕 RGB 摄像头",
                "type": "rgb",
                "topic": "/left_wrist_camera/color/image_raw",
                "width": 640,
                "height": 480,
                "fps": 30,
                "encoding": "bgr8"
            },
            {
                "id": "right_wrist",
                "name": "右手腕 RGB 摄像头",
                "type": "rgb",
                "topic": "/right_wrist_camera/color/image_raw",
                "width": 640,
                "height": 480,
                "fps": 30,
                "encoding": "bgr8"
            },
            {
                "id": "chassis_front",
                "name": "底盘前方摄像头",
                "type": "rgb",
                "topic": "/chassis_camera/front/image_raw",
                "width": 640,
                "height": 480,
                "fps": 15,
                "encoding": "bgr8"
            }
        ],
        "current_streaming": None
    }


def _save_camera_config(config: dict):
    """保存摄像头配置"""
    config_file = _get_camera_config_file()
    with open(config_file, 'w', encoding='utf-8') as f:
        json.dump(config, f, indent=2, ensure_ascii=False)


# ==================== API 端点 ====================

@router.get("/list", response_model=ApiResponse)
async def get_camera_list(
    current_user: User = Depends(get_current_user),
):
    """
    获取可用摄像头列表
    
    返回所有已配置的摄像头信息，包括：
    - 摄像头 ID 和名称
    - ROS2 话题
    - 分辨率和帧率
    - WebRTC track ID (如果正在推流)
    
    前端可根据此列表选择要订阅的视频流。
    """
    config = _load_camera_config()
    
    cameras = []
    for cam_data in config.get("cameras", []):
        cameras.append(CameraInfo(**cam_data))
    
    return success_response(
        data={
            "cameras": [c.model_dump() for c in cameras],
            "current_streaming": config.get("current_streaming"),
        },
        message="获取摄像头列表成功"
    )


@router.get("/{camera_id}", response_model=ApiResponse)
async def get_camera_info(
    camera_id: str,
    current_user: User = Depends(get_current_user),
):
    """
    获取指定摄像头信息
    """
    config = _load_camera_config()
    
    for cam_data in config.get("cameras", []):
        if cam_data.get("id") == camera_id:
            camera = CameraInfo(**cam_data)
            return success_response(
                data=camera.model_dump(),
                message="获取摄像头信息成功"
            )
    
    return error_response(
        code=ErrorCodes.RESOURCE_NOT_FOUND,
        message=f"未找到摄像头: {camera_id}"
    )


@router.get("/{camera_id}/webrtc", response_model=ApiResponse)
async def get_camera_webrtc_info(
    camera_id: str,
    current_user: User = Depends(get_current_user),
):
    """
    获取摄像头的 WebRTC 连接信息
    
    返回用于建立 WebRTC 连接的信息，包括：
    - Media Plane signaling URL
    - 可用的 track IDs
    - 推荐的编解码器
    
    ⚠️ 实际 WebRTC 连接应直接与 Media Plane 建立。
    """
    # TODO: 从 Media Plane 获取实际的 WebRTC 信息
    
    # 从配置获取 Media Plane 地址
    config = _load_camera_config()
    camera = None
    for cam_data in config.get("cameras", []):
        if cam_data.get("id") == camera_id:
            camera = CameraInfo(**cam_data)
            break

    if camera is None:
        return error_response(
            code=ErrorCodes.RESOURCE_NOT_FOUND,
            message=f"未找到摄像头: {camera_id}"
        )

    from urllib.parse import urlparse
    from app.config import settings
    
    parsed = urlparse(settings.WEBRTC_SIGNALING_URL)
    netloc = parsed.netloc or parsed.path
    scheme = parsed.scheme.lower()
    if scheme in ("https", "wss"):
        ws_scheme = "wss"
    else:
        ws_scheme = "ws"
    signaling_url = f"{ws_scheme}://{netloc}/webrtc"
    
    return success_response(
        data={
            "camera_id": camera.id,
            "source": camera.id,
            "topic": camera.topic,
            "signaling_url": signaling_url,
            "ice_servers": [
                {"urls": "stun:stun.l.google.com:19302"}
            ],
            "codec_preference": ["H264", "VP8"],
        },
        message="获取 WebRTC 信息成功"
    )
