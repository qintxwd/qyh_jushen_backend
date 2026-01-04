"""
ROS2 录制客户端

用于调用 qyh_bag_recorder 节点的服务
"""

import asyncio
import logging
from typing import List, Dict, Any, Optional

logger = logging.getLogger(__name__)

# 尝试导入 ROS2
try:
    import rclpy
    from rclpy.node import Node
    from qyh_bag_recorder.srv import StartRecording, StopRecording, GetRecordingStatus
    ROS2_AVAILABLE = True
except ImportError:
    ROS2_AVAILABLE = False
    logger.warning("ROS2 不可用，录制功能将使用模拟模式")


class ROSRecordingClient:
    """ROS2 录制服务客户端"""
    
    def __init__(self, node_name: str = "recording_client"):
        self._node: Optional[Node] = None
        self._node_name = node_name
        self._initialized = False
        
        if ROS2_AVAILABLE:
            self._initialize_ros()
    
    def _initialize_ros(self):
        """初始化 ROS2"""
        try:
            if not rclpy.ok():
                rclpy.init()
            
            self._node = rclpy.create_node(self._node_name)
            
            # 创建服务客户端
            self._start_client = self._node.create_client(
                StartRecording, 
                '/bag_recorder/start_recording'
            )
            self._stop_client = self._node.create_client(
                StopRecording,
                '/bag_recorder/stop_recording'
            )
            self._status_client = self._node.create_client(
                GetRecordingStatus,
                '/bag_recorder/get_status'
            )
            
            self._initialized = True
            logger.info("ROS2 录制客户端初始化成功")
            
        except Exception as e:
            logger.error(f"ROS2 初始化失败: {e}")
            self._initialized = False
    
    async def start_recording(
        self, 
        action_name: str,
        user_name: str, 
        version: str,
        topics: List[str] = None
    ) -> Dict[str, Any]:
        """
        开始录制
        
        Args:
            action_name: 动作名称
            user_name: 用户名
            version: 版本号
            topics: 要录制的话题列表
            
        Returns:
            包含 success, message, bag_path 的字典
        """
        if not self._initialized:
            return {
                "success": False,
                "message": "ROS2 未初始化",
                "bag_path": ""
            }
        
        try:
            request = StartRecording.Request()
            request.action_name = action_name
            request.user_name = user_name
            request.version = version
            request.topics = topics or []
            
            # 等待服务可用
            if not self._start_client.wait_for_service(timeout_sec=2.0):
                return {
                    "success": False,
                    "message": "录制服务不可用",
                    "bag_path": ""
                }
            
            # 调用服务
            future = self._start_client.call_async(request)
            
            # 使用 asyncio 等待
            response = await asyncio.get_event_loop().run_in_executor(
                None,
                lambda: self._spin_until_future_complete(future)
            )
            
            if response is None:
                return {
                    "success": False,
                    "message": "服务调用超时",
                    "bag_path": ""
                }
            
            return {
                "success": response.success,
                "message": response.message,
                "bag_path": response.bag_path
            }
            
        except Exception as e:
            logger.error(f"调用开始录制服务失败: {e}")
            return {
                "success": False,
                "message": str(e),
                "bag_path": ""
            }
    
    async def stop_recording(self) -> Dict[str, Any]:
        """
        停止录制
        
        Returns:
            包含 success, message, duration_sec, bag_path 的字典
        """
        if not self._initialized:
            return {
                "success": False,
                "message": "ROS2 未初始化",
                "duration_sec": 0.0,
                "bag_path": ""
            }
        
        try:
            request = StopRecording.Request()
            
            if not self._stop_client.wait_for_service(timeout_sec=2.0):
                return {
                    "success": False,
                    "message": "录制服务不可用",
                    "duration_sec": 0.0,
                    "bag_path": ""
                }
            
            future = self._stop_client.call_async(request)
            
            response = await asyncio.get_event_loop().run_in_executor(
                None,
                lambda: self._spin_until_future_complete(future)
            )
            
            if response is None:
                return {
                    "success": False,
                    "message": "服务调用超时",
                    "duration_sec": 0.0,
                    "bag_path": ""
                }
            
            return {
                "success": response.success,
                "message": response.message,
                "duration_sec": response.duration_sec,
                "bag_path": response.bag_path
            }
            
        except Exception as e:
            logger.error(f"调用停止录制服务失败: {e}")
            return {
                "success": False,
                "message": str(e),
                "duration_sec": 0.0,
                "bag_path": ""
            }
    
    async def get_status(self) -> Dict[str, Any]:
        """
        获取录制状态
        
        Returns:
            包含 is_recording, action_name, duration_sec, bag_path, topics 的字典
        """
        if not self._initialized:
            return {
                "is_recording": False,
                "action_name": "",
                "duration_sec": 0.0,
                "bag_path": "",
                "topics": []
            }
        
        try:
            request = GetRecordingStatus.Request()
            
            if not self._status_client.wait_for_service(timeout_sec=2.0):
                return {
                    "is_recording": False,
                    "action_name": "",
                    "duration_sec": 0.0,
                    "bag_path": "",
                    "topics": []
                }
            
            future = self._status_client.call_async(request)
            
            response = await asyncio.get_event_loop().run_in_executor(
                None,
                lambda: self._spin_until_future_complete(future)
            )
            
            if response is None:
                return {
                    "is_recording": False,
                    "action_name": "",
                    "duration_sec": 0.0,
                    "bag_path": "",
                    "topics": []
                }
            
            return {
                "is_recording": response.is_recording,
                "action_name": response.action_name,
                "duration_sec": response.duration_sec,
                "bag_path": response.bag_path,
                "topics": list(response.topics)
            }
            
        except Exception as e:
            logger.error(f"调用获取状态服务失败: {e}")
            return {
                "is_recording": False,
                "action_name": "",
                "duration_sec": 0.0,
                "bag_path": "",
                "topics": []
            }
    
    async def get_available_topics(self) -> List[str]:
        """
        获取可用的话题列表
        
        Returns:
            话题名称列表
        """
        if not self._initialized:
            return []
        
        try:
            # 获取所有话题
            topic_list = self._node.get_topic_names_and_types()
            return [name for name, _ in topic_list]
        except Exception as e:
            logger.error(f"获取话题列表失败: {e}")
            return []
    
    def _spin_until_future_complete(self, future, timeout_sec: float = 5.0):
        """同步等待 future 完成"""
        import time
        start_time = time.time()
        
        while not future.done():
            rclpy.spin_once(self._node, timeout_sec=0.1)
            if time.time() - start_time > timeout_sec:
                return None
        
        return future.result()
    
    def __del__(self):
        """清理资源"""
        if self._node is not None:
            self._node.destroy_node()
