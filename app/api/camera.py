"""相机视频流 API - 代理 ROS web_video_server 的视频流"""
import httpx
from fastapi import APIRouter, Response, Query
from fastapi.responses import StreamingResponse
from typing import Optional
import asyncio

from app.schemas.response import (
    ApiResponse, success_response, error_response, ErrorCodes
)

router = APIRouter(prefix="/camera", tags=["camera"])

# web_video_server 配置
WEB_VIDEO_SERVER_HOST = "localhost"
WEB_VIDEO_SERVER_PORT = 8080

# 相机话题映射
CAMERA_TOPICS = {
    "head": "/head_camera/color/image_raw",
    "left_hand": "/left_camera/color/image_raw",
    "right_hand": "/right_camera/color/image_raw",
}


async def proxy_video_stream(url: str):
    """代理视频流的异步生成器"""
    async with httpx.AsyncClient(timeout=None) as client:
        try:
            async with client.stream("GET", url) as response:
                if response.status_code != 200:
                    return
                async for chunk in response.aiter_bytes(chunk_size=4096):
                    yield chunk
        except httpx.RequestError:
            return


@router.get("/stream/{camera_id}")
async def get_camera_stream(
    camera_id: str,
    quality: int = Query(default=50, ge=1, le=100, description="JPEG 质量 (1-100)"),
    width: Optional[int] = Query(default=None, description="输出宽度"),
    height: Optional[int] = Query(default=None, description="输出高度"),
):
    """
    获取相机 MJPEG 视频流
    
    - camera_id: 相机标识 (head, left_hand, right_hand)
    - quality: JPEG 质量 (1-100)
    - width/height: 可选的输出尺寸
    
    返回 multipart/x-mixed-replace 格式的 MJPEG 流
    """
    if camera_id not in CAMERA_TOPICS:
        # 视频流端点返回错误时需要用 Response
        return Response(
            content=f"Unknown camera: {camera_id}. Available: {list(CAMERA_TOPICS.keys())}",
            status_code=404,
            media_type="text/plain"
        )
    
    topic = CAMERA_TOPICS[camera_id]
    
    # 构建 web_video_server URL
    params = [
        f"topic={topic}",
        "type=mjpeg",
        f"quality={quality}",
    ]
    if width:
        params.append(f"width={width}")
    if height:
        params.append(f"height={height}")
    
    url = f"http://{WEB_VIDEO_SERVER_HOST}:{WEB_VIDEO_SERVER_PORT}/stream?{'&'.join(params)}"
    
    return StreamingResponse(
        proxy_video_stream(url),
        media_type="multipart/x-mixed-replace; boundary=boundarydonotcross",
        headers={
            "Cache-Control": "no-cache, no-store, must-revalidate",
            "Pragma": "no-cache",
            "Expires": "0",
        }
    )


@router.get("/snapshot/{camera_id}")
async def get_camera_snapshot(
    camera_id: str,
    quality: int = Query(default=80, ge=1, le=100, description="JPEG 质量 (1-100)"),
    width: Optional[int] = Query(default=None, description="输出宽度"),
    height: Optional[int] = Query(default=None, description="输出高度"),
):
    """
    获取相机单帧快照
    
    - camera_id: 相机标识 (head, left_hand, right_hand)
    - quality: JPEG 质量 (1-100)
    - width/height: 可选的输出尺寸
    
    返回 JPEG 图片
    """
    if camera_id not in CAMERA_TOPICS:
        return Response(
            content=f"Unknown camera: {camera_id}. Available: {list(CAMERA_TOPICS.keys())}",
            status_code=404,
            media_type="text/plain"
        )
    
    topic = CAMERA_TOPICS[camera_id]
    
    # 构建 web_video_server snapshot URL
    params = [
        f"topic={topic}",
        f"quality={quality}",
    ]
    if width:
        params.append(f"width={width}")
    if height:
        params.append(f"height={height}")
    
    url = f"http://{WEB_VIDEO_SERVER_HOST}:{WEB_VIDEO_SERVER_PORT}/snapshot?{'&'.join(params)}"
    
    async with httpx.AsyncClient(timeout=5.0) as client:
        try:
            response = await client.get(url)
            if response.status_code != 200:
                return Response(
                    content="Camera not available or web_video_server not running",
                    status_code=503,
                    media_type="text/plain"
                )
            return Response(
                content=response.content,
                media_type="image/jpeg",
                headers={
                    "Cache-Control": "no-cache, no-store, must-revalidate",
                }
            )
        except httpx.RequestError as e:
            return Response(
                content=f"Cannot connect to web_video_server: {str(e)}",
                status_code=503,
                media_type="text/plain"
            )


@router.get("/status")
async def get_camera_status():
    """
    获取相机服务状态
    
    返回:
    - web_video_server_available: web_video_server 是否可用
    - cameras: 各相机的可用状态
    """
    status = {
        "web_video_server_available": False,
        "web_video_server_url": f"http://{WEB_VIDEO_SERVER_HOST}:{WEB_VIDEO_SERVER_PORT}",
        "cameras": {}
    }
    
    # 检查 web_video_server 是否在运行
    try:
        async with httpx.AsyncClient(timeout=2.0) as client:
            response = await client.get(
                f"http://{WEB_VIDEO_SERVER_HOST}:{WEB_VIDEO_SERVER_PORT}/"
            )
            status["web_video_server_available"] = response.status_code == 200
    except (httpx.RequestError, httpx.TimeoutException, Exception):
        status["web_video_server_available"] = False
    
    # 检查各相机话题状态（通过尝试获取快照）
    if status["web_video_server_available"]:
        for camera_id, topic in CAMERA_TOPICS.items():
            try:
                async with httpx.AsyncClient(timeout=2.0) as client:
                    url = f"http://{WEB_VIDEO_SERVER_HOST}:{WEB_VIDEO_SERVER_PORT}/snapshot?topic={topic}"
                    response = await client.get(url)
                    status["cameras"][camera_id] = {
                        "available": response.status_code == 200,
                        "topic": topic
                    }
            except (httpx.RequestError, httpx.TimeoutException, Exception):
                status["cameras"][camera_id] = {
                    "available": False,
                    "topic": topic
                }
    else:
        for camera_id, topic in CAMERA_TOPICS.items():
            status["cameras"][camera_id] = {
                "available": False,
                "topic": topic
            }
    
    return status


@router.get("/topics")
async def get_available_topics():
    """
    获取可用的相机话题列表
    
    返回所有配置的相机 ID 和对应的 ROS 话题
    """
    return {
        "cameras": [
            {"id": cam_id, "topic": topic, "name": cam_id.replace("_", " ").title()}
            for cam_id, topic in CAMERA_TOPICS.items()
        ]
    }


@router.get("/topic_status")
async def get_camera_topic_status():
    """
    基于 ROS2 topic 检查相机状态（更准确）
    
    直接检查 ROS2 话题是否在发布数据，而不依赖 web_video_server
    返回各相机话题的发布状态
    """
    status = {"cameras": {}}
    
    async def check_single_topic(camera_id: str, topic: str):
        """异步检查单个话题状态"""
        try:
            # 使用 ros2 topic info 检查话题是否存在且有发布者
            process = await asyncio.create_subprocess_exec(
                "ros2", "topic", "info", topic,
                stdout=asyncio.subprocess.PIPE,
                stderr=asyncio.subprocess.PIPE
            )
            
            try:
                stdout, stderr = await asyncio.wait_for(
                    process.communicate(), timeout=1.5
                )
                output = stdout.decode()
                
                # 检查是否有发布者
                has_publisher = (
                    "Publisher count:" in output
                    and "Publisher count: 0" not in output
                )
                
                return camera_id, {
                    "available": has_publisher,
                    "topic": topic,
                    "method": "ros2_topic_info"
                }
            except asyncio.TimeoutError:
                process.kill()
                return camera_id, {
                    "available": False,
                    "topic": topic,
                    "error": "timeout"
                }
        except Exception as e:
            return camera_id, {
                "available": False,
                "topic": topic,
                "error": str(e)
            }
    
    # 并发检查所有话题，提高响应速度
    tasks = [
        check_single_topic(cam_id, topic)
        for cam_id, topic in CAMERA_TOPICS.items()
    ]
    results = await asyncio.gather(*tasks)
    
    for camera_id, camera_status in results:
        status["cameras"][camera_id] = camera_status
    
    return status
