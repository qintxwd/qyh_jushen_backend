"""
QYH Jushen Control Plane - 健康检查服务

检测 Data Plane / Media Plane / ROS2 的连接状态
"""
import asyncio
import time
import logging
from typing import Optional, Dict, Any
from dataclasses import dataclass
from enum import Enum

import aiohttp

from app.config import settings

logger = logging.getLogger(__name__)


class ServiceStatus(str, Enum):
    """服务状态"""
    HEALTHY = "healthy"
    UNHEALTHY = "unhealthy"
    UNKNOWN = "unknown"
    DEGRADED = "degraded"


@dataclass
class ServiceHealthResult:
    """服务健康检查结果"""
    name: str
    status: ServiceStatus
    latency_ms: Optional[float] = None
    message: Optional[str] = None
    details: Optional[Dict[str, Any]] = None


class HealthChecker:
    """
    健康检查服务
    
    定期检查各个子服务的健康状态
    """
    
    def __init__(self):
        self._cache: Dict[str, ServiceHealthResult] = {}
        self._cache_ttl = 5.0  # 缓存 5 秒
        self._last_check: Dict[str, float] = {}
    
    async def check_data_plane(self) -> ServiceHealthResult:
        """
        检查 Data Plane (WebSocket 服务) 健康状态
        
        Data Plane 通常在 ws://host:8765
        尝试 HTTP 健康检查端点（如果有），或尝试 WebSocket 连接
        """
        name = "data_plane"
        
        # 检查缓存
        cached = self._get_cached(name)
        if cached:
            return cached
        
        # 从 WebSocket URL 提取 HTTP URL
        ws_url = settings.WEBSOCKET_SERVER_URL
        # ws://127.0.0.1:8765 -> http://127.0.0.1:8765
        http_url = ws_url.replace("ws://", "http://").replace("wss://", "https://")
        health_url = f"{http_url}/health"
        
        start_time = time.time()
        
        try:
            async with aiohttp.ClientSession(timeout=aiohttp.ClientTimeout(total=3)) as session:
                # 先尝试 HTTP 健康检查
                try:
                    async with session.get(health_url) as resp:
                        latency_ms = (time.time() - start_time) * 1000
                        if resp.status == 200:
                            result = ServiceHealthResult(
                                name=name,
                                status=ServiceStatus.HEALTHY,
                                latency_ms=round(latency_ms, 2),
                                message="Data Plane 连接正常"
                            )
                        else:
                            result = ServiceHealthResult(
                                name=name,
                                status=ServiceStatus.DEGRADED,
                                latency_ms=round(latency_ms, 2),
                                message=f"Data Plane 返回状态码 {resp.status}"
                            )
                except aiohttp.ClientError:
                    # HTTP 失败，尝试简单的 TCP 连接
                    try:
                        # 解析主机和端口
                        host = ws_url.split("://")[1].split("/")[0]
                        if ":" in host:
                            hostname, port = host.split(":")
                            port = int(port)
                        else:
                            hostname = host
                            port = 80
                        
                        # 尝试 TCP 连接
                        _, writer = await asyncio.wait_for(
                            asyncio.open_connection(hostname, port),
                            timeout=2.0
                        )
                        writer.close()
                        await writer.wait_closed()
                        
                        latency_ms = (time.time() - start_time) * 1000
                        result = ServiceHealthResult(
                            name=name,
                            status=ServiceStatus.HEALTHY,
                            latency_ms=round(latency_ms, 2),
                            message="Data Plane 端口可达（无 HTTP 健康端点）"
                        )
                    except Exception as e:
                        latency_ms = (time.time() - start_time) * 1000
                        result = ServiceHealthResult(
                            name=name,
                            status=ServiceStatus.UNHEALTHY,
                            latency_ms=round(latency_ms, 2),
                            message=f"Data Plane 不可达: {str(e)}"
                        )
        except Exception as e:
            latency_ms = (time.time() - start_time) * 1000
            result = ServiceHealthResult(
                name=name,
                status=ServiceStatus.UNHEALTHY,
                latency_ms=round(latency_ms, 2),
                message=f"连接失败: {str(e)}"
            )
        
        self._set_cache(name, result)
        return result
    
    async def check_media_plane(self) -> ServiceHealthResult:
        """
        检查 Media Plane (WebRTC 信令服务) 健康状态
        
        Media Plane 通常在 http://host:8888
        """
        name = "media_plane"
        
        # 检查缓存
        cached = self._get_cached(name)
        if cached:
            return cached
        
        signaling_url = settings.WEBRTC_SIGNALING_URL
        health_url = f"{signaling_url}/health"
        
        start_time = time.time()
        
        try:
            async with aiohttp.ClientSession(timeout=aiohttp.ClientTimeout(total=3)) as session:
                try:
                    async with session.get(health_url) as resp:
                        latency_ms = (time.time() - start_time) * 1000
                        if resp.status == 200:
                            result = ServiceHealthResult(
                                name=name,
                                status=ServiceStatus.HEALTHY,
                                latency_ms=round(latency_ms, 2),
                                message="Media Plane 连接正常"
                            )
                        else:
                            result = ServiceHealthResult(
                                name=name,
                                status=ServiceStatus.DEGRADED,
                                latency_ms=round(latency_ms, 2),
                                message=f"Media Plane 返回状态码 {resp.status}"
                            )
                except aiohttp.ClientError:
                    # HTTP 失败，尝试 TCP 连接
                    try:
                        host = signaling_url.split("://")[1].split("/")[0]
                        if ":" in host:
                            hostname, port = host.split(":")
                            port = int(port)
                        else:
                            hostname = host
                            port = 80
                        
                        _, writer = await asyncio.wait_for(
                            asyncio.open_connection(hostname, port),
                            timeout=2.0
                        )
                        writer.close()
                        await writer.wait_closed()
                        
                        latency_ms = (time.time() - start_time) * 1000
                        result = ServiceHealthResult(
                            name=name,
                            status=ServiceStatus.HEALTHY,
                            latency_ms=round(latency_ms, 2),
                            message="Media Plane 端口可达（无 HTTP 健康端点）"
                        )
                    except Exception as e:
                        latency_ms = (time.time() - start_time) * 1000
                        result = ServiceHealthResult(
                            name=name,
                            status=ServiceStatus.UNHEALTHY,
                            latency_ms=round(latency_ms, 2),
                            message=f"Media Plane 不可达: {str(e)}"
                        )
        except Exception as e:
            latency_ms = (time.time() - start_time) * 1000
            result = ServiceHealthResult(
                name=name,
                status=ServiceStatus.UNHEALTHY,
                latency_ms=round(latency_ms, 2),
                message=f"连接失败: {str(e)}"
            )
        
        self._set_cache(name, result)
        return result
    
    async def check_ros2(self) -> ServiceHealthResult:
        """
        检查 ROS2 连接状态
        
        通过 ROS2ServiceClient 检查 ROS2 是否可用
        """
        name = "ros2"
        
        # 检查缓存
        cached = self._get_cached(name)
        if cached:
            return cached
        
        try:
            from app.services.ros2_client import get_ros2_client
            
            ros2_client = get_ros2_client()
            
            if ros2_client.is_mock_mode:
                result = ServiceHealthResult(
                    name=name,
                    status=ServiceStatus.UNHEALTHY,
                    message="ROS2 不可用（使用 Mock 模式）",
                    details={"mock_mode": True}
                )
            else:
                # ROS2 可用，进一步检查节点状态
                # TODO: 可以调用一个简单的服务来验证节点活跃
                result = ServiceHealthResult(
                    name=name,
                    status=ServiceStatus.HEALTHY,
                    message="ROS2 连接正常",
                    details={"mock_mode": False}
                )
        except Exception as e:
            result = ServiceHealthResult(
                name=name,
                status=ServiceStatus.UNHEALTHY,
                message=f"ROS2 检查失败: {str(e)}"
            )
        
        self._set_cache(name, result)
        return result
    
    async def check_all(self) -> Dict[str, ServiceHealthResult]:
        """
        检查所有服务的健康状态
        
        并行执行所有检查
        """
        results = await asyncio.gather(
            self.check_data_plane(),
            self.check_media_plane(),
            self.check_ros2(),
            return_exceptions=True
        )
        
        services = {}
        service_names = ["data_plane", "media_plane", "ros2"]
        
        for name, result in zip(service_names, results):
            if isinstance(result, Exception):
                services[name] = ServiceHealthResult(
                    name=name,
                    status=ServiceStatus.UNKNOWN,
                    message=f"检查异常: {str(result)}"
                )
            else:
                services[name] = result
        
        return services
    
    def _get_cached(self, name: str) -> Optional[ServiceHealthResult]:
        """获取缓存的检查结果"""
        if name in self._cache and name in self._last_check:
            if time.time() - self._last_check[name] < self._cache_ttl:
                return self._cache[name]
        return None
    
    def _set_cache(self, name: str, result: ServiceHealthResult):
        """设置缓存"""
        self._cache[name] = result
        self._last_check[name] = time.time()
    
    def clear_cache(self):
        """清除所有缓存"""
        self._cache.clear()
        self._last_check.clear()


# 全局单例
health_checker = HealthChecker()


def get_health_checker() -> HealthChecker:
    """获取健康检查器单例"""
    return health_checker
