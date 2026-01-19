"""相机视频流 API - 代理 ROS web_video_server 的视频流

优化内容：
1. 统一 HTTP 客户端连接池，减少连接开销
2. 添加 FPS 和延迟统计
3. 更详细的相机状态信息
4. 支持批量查询相机状态
"""
import httpx
import time
from fastapi import APIRouter, Response, Query
from fastapi.responses import StreamingResponse
from typing import Optional, Dict, Any
import asyncio

from app.schemas.response import ApiResponse, success_response
from app.ros2_bridge.bridge import ros2_bridge

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

# 相机友好名称
CAMERA_NAMES = {
    "head": "头部相机",
    "left_hand": "左手相机",
    "right_hand": "右手相机",
}

# 全局 HTTP 客户端（连接池复用）
_http_client: Optional[httpx.AsyncClient] = None


async def get_http_client() -> httpx.AsyncClient:
    """获取复用的 HTTP 客户端"""
    global _http_client
    if _http_client is None or _http_client.is_closed:
        _http_client = httpx.AsyncClient(
            timeout=httpx.Timeout(10.0, connect=5.0),
            limits=httpx.Limits(max_keepalive_connections=10, max_connections=20),
        )
    return _http_client


async def close_http_client():
    """关闭 HTTP 客户端"""
    global _http_client
    if _http_client is not None:
        await _http_client.aclose()
        _http_client = None


async def proxy_video_stream(url: str):
    """代理视频流的异步生成器，使用连接池"""
    client = await get_http_client()
    try:
        async with client.stream("GET", url, timeout=None) as response:
            if response.status_code != 200:
                return
            async for chunk in response.aiter_bytes(chunk_size=8192):
                yield chunk
    except (httpx.RequestError, Exception):
        return


async def proxy_video_stream_with_stats(url: str, camera_id: str):
    """代理视频流并统计帧率"""
    client = await get_http_client()
    frame_count = 0
    start_time = time.time()
    
    try:
        async with client.stream("GET", url, timeout=None) as response:
            if response.status_code != 200:
                return
            
            buffer = b""
            async for chunk in response.aiter_bytes(chunk_size=8192):
                buffer += chunk
                # 统计 MJPEG 帧数（通过边界标记）
                while b"--boundarydonotcross" in buffer:
                    frame_count += 1
                    idx = buffer.index(b"--boundarydonotcross")
                    yield buffer[:idx + len(b"--boundarydonotcross")]
                    buffer = buffer[idx + len(b"--boundarydonotcross"):]
                    
                    # 每 30 帧更新一次 FPS 统计
                    if frame_count % 30 == 0:
                        elapsed = time.time() - start_time
                        if elapsed > 0:
                            fps = frame_count / elapsed
                            # 可以在这里记录 FPS 到某个全局状态
                
                if buffer:
                    yield buffer
                    buffer = b""
                    
    except (httpx.RequestError, Exception):
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
    
    client = await get_http_client()
    try:
        response = await client.get(url, timeout=5.0)
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
                "X-Camera-Id": camera_id,
                "X-Camera-Topic": topic,
            }
        )
    except httpx.RequestError as e:
        return Response(
            content=f"Cannot connect to web_video_server: {str(e)}",
            status_code=503,
            media_type="text/plain"
        )


@router.get("/status", response_model=ApiResponse)
async def get_camera_status():
    """
    获取相机服务状态（增强版）
    
    返回:
    - web_video_server_available: web_video_server 是否可用
    - cameras: 各相机的可用状态（包含 ROS 话题和视频服务两个来源）
    - summary: 摘要统计信息
    """
    result = {
        "web_video_server": {
            "available": False,
            "url": f"http://{WEB_VIDEO_SERVER_HOST}:{WEB_VIDEO_SERVER_PORT}",
        },
        "ros_bridge_connected": ros2_bridge.is_connected(),
        "cameras": {},
        "summary": {
            "total": len(CAMERA_TOPICS),
            "online": 0,
            "offline": 0,
        }
    }
    
    client = await get_http_client()
    
    # 检查 web_video_server 是否在运行
    try:
        response = await client.get(
            f"http://{WEB_VIDEO_SERVER_HOST}:{WEB_VIDEO_SERVER_PORT}/",
            timeout=2.0
        )
        result["web_video_server"]["available"] = response.status_code == 200
    except (httpx.RequestError, httpx.TimeoutException, Exception):
        result["web_video_server"]["available"] = False
    
    # 并行检查所有相机
    async def check_camera(camera_id: str, topic: str) -> Dict[str, Any]:
        cam_info = {
            "id": camera_id,
            "name": CAMERA_NAMES.get(camera_id, camera_id),
            "topic": topic,
            "ros_available": False,
            "stream_available": False,
            "last_frame_time": None,
            "fps": None,
        }
        
        # 从 ROS bridge 获取状态
        if ros2_bridge.is_connected():
            ros_status = ros2_bridge.get_camera_status(camera_id)
            if ros_status:
                cam_info["ros_available"] = ros_status.get("available", False)
                cam_info["last_frame_time"] = ros_status.get("last_seen_sec")
                cam_info["fps"] = ros_status.get("fps")
        
        # 检查视频流是否可用
        if result["web_video_server"]["available"]:
            try:
                url = f"http://{WEB_VIDEO_SERVER_HOST}:{WEB_VIDEO_SERVER_PORT}/snapshot?topic={topic}"
                response = await client.get(url, timeout=2.0)
                cam_info["stream_available"] = response.status_code == 200
            except (httpx.RequestError, httpx.TimeoutException, Exception):
                cam_info["stream_available"] = False
        
        # 综合判断
        cam_info["available"] = cam_info["ros_available"] or cam_info["stream_available"]
        
        return cam_info
    
    # 并行检查所有相机
    tasks = [check_camera(cam_id, topic) for cam_id, topic in CAMERA_TOPICS.items()]
    camera_results = await asyncio.gather(*tasks)
    
    for cam_info in camera_results:
        result["cameras"][cam_info["id"]] = cam_info
        if cam_info["available"]:
            result["summary"]["online"] += 1
        else:
            result["summary"]["offline"] += 1
    
    return success_response(data=result)


@router.get("/topics", response_model=ApiResponse)
async def get_available_topics():
    """
    获取可用的相机话题列表
    
    返回所有配置的相机 ID 和对应的 ROS 话题
    """
    cameras = [
        {
            "id": cam_id, 
            "topic": topic, 
            "name": CAMERA_NAMES.get(cam_id, cam_id.replace("_", " ").title()),
            "stream_url": f"/api/v1/camera/stream/{cam_id}",
            "snapshot_url": f"/api/v1/camera/snapshot/{cam_id}",
        }
        for cam_id, topic in CAMERA_TOPICS.items()
    ]
    return success_response(data={"cameras": cameras})


@router.get("/topic_status", response_model=ApiResponse)
async def get_camera_topic_status():
    """
    检查相机 ROS 话题状态
    
    通过 ROS2 topic 是否有数据来检测（与视频流解耦）
    """
    cameras = {}

    if not ros2_bridge.is_connected():
        for camera_id, topic in CAMERA_TOPICS.items():
            cameras[camera_id] = {
                "available": False,
                "topic": topic,
                "name": CAMERA_NAMES.get(camera_id, camera_id),
                "error": "ros2_bridge_not_connected"
            }
        return success_response(data={
            "cameras": cameras, 
            "source": "ros_topic",
            "ros_connected": False
        })

    for camera_id, topic in CAMERA_TOPICS.items():
        info = ros2_bridge.get_camera_status(camera_id)
        if info is None:
            cameras[camera_id] = {
                "available": False,
                "topic": topic,
                "name": CAMERA_NAMES.get(camera_id, camera_id),
                "error": "camera_id_not_registered"
            }
            continue

        cameras[camera_id] = {
            "available": info.get("available", False),
            "topic": topic,
            "name": CAMERA_NAMES.get(camera_id, camera_id),
            "last_seen_sec": info.get("last_seen_sec"),
            "timeout_sec": info.get("timeout_sec"),
            "fps": info.get("fps"),
        }

    return success_response(data={
        "cameras": cameras,
        "source": "ros_topic",
        "ros_connected": True
    })
