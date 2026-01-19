"""相机组件 - 管理相机状态检测和视频流"""
import time
from typing import Optional, Dict, Any, List
from .base import BridgeComponent


class CameraComponent(BridgeComponent):
    """相机控制组件
    
    功能:
    - 相机话题订阅和状态检测
    - 基于 ROS2 消息时间戳判断相机是否在线
    - 提供相机列表和状态查询
    """
    
    # 默认相机配置
    DEFAULT_CAMERA_TOPICS = {
        "head": "/head_camera/color/image_raw",
        "left_hand": "/left_camera/color/image_raw",
        "right_hand": "/right_camera/color/image_raw"
    }
    
    def __init__(self, camera_topics: Optional[Dict[str, str]] = None):
        super().__init__()
        # 相机话题配置
        self.camera_topics = camera_topics or self.DEFAULT_CAMERA_TOPICS.copy()
        # 最后消息时间缓存
        self.camera_last_msg_time: Dict[str, float] = {
            cam_id: 0.0 for cam_id in self.camera_topics
        }
        # 超时时间（秒）
        self.timeout_seconds: float = 3.0
        # 帧率统计
        self.camera_frame_count: Dict[str, int] = {
            cam_id: 0 for cam_id in self.camera_topics
        }
        self.camera_fps: Dict[str, float] = {
            cam_id: 0.0 for cam_id in self.camera_topics
        }
        self._last_fps_calc_time: float = 0.0
    
    def setup_subscribers(self):
        """设置相机话题订阅器"""
        try:
            from sensor_msgs.msg import Image
            
            for camera_id, topic in self.camera_topics.items():
                # 使用闭包捕获 camera_id
                def make_callback(cid: str):
                    def callback(msg: Image):
                        now = time.time()
                        self.camera_last_msg_time[cid] = now
                        self.camera_frame_count[cid] += 1
                        
                        # 每秒计算一次帧率
                        if now - self._last_fps_calc_time >= 1.0:
                            self._calculate_fps()
                            self._last_fps_calc_time = now
                    return callback
                
                self.node.create_subscription(
                    Image,
                    topic,
                    make_callback(camera_id),
                    1  # 队列深度为1，只关心最新消息
                )
                print(f"✅ 相机订阅器创建成功: {camera_id} -> {topic}")
        except Exception as e:
            print(f"⚠️  相机订阅器创建失败: {e}")
    
    def setup_publishers(self):
        """相机组件不需要发布器"""
        pass
    
    def _calculate_fps(self):
        """计算帧率"""
        for cam_id in self.camera_topics:
            self.camera_fps[cam_id] = float(self.camera_frame_count[cam_id])
            self.camera_frame_count[cam_id] = 0
    
    # ==================== 状态获取 ====================
    
    def get_camera_status(self, camera_id: str) -> Optional[Dict[str, Any]]:
        """获取单个相机状态
        
        Args:
            camera_id: 相机标识
            
        Returns:
            相机状态字典，包含:
            - available: 是否在线
            - topic: ROS 话题名
            - last_seen_sec: 距离上次收到消息的秒数
            - timeout_sec: 超时阈值
            - fps: 帧率（仅当在线时有效）
        """
        if camera_id not in self.camera_topics:
            return None
        
        if self.mock_mode:
            return {
                "available": True,
                "topic": self.camera_topics[camera_id],
                "last_seen_sec": 0.0,
                "timeout_sec": self.timeout_seconds,
                "fps": 30.0
            }
        
        try:
            last_time = self.camera_last_msg_time.get(camera_id, 0.0)
            now = time.time()
            last_seen_sec = now - last_time if last_time > 0.0 else None
            available = last_seen_sec is not None and last_seen_sec <= self.timeout_seconds
            
            return {
                "available": available,
                "topic": self.camera_topics[camera_id],
                "last_seen_sec": last_seen_sec,
                "timeout_sec": self.timeout_seconds,
                "fps": self.camera_fps.get(camera_id, 0.0) if available else 0.0
            }
        except Exception:
            return None
    
    def get_all_camera_status(self) -> Dict[str, Dict[str, Any]]:
        """获取所有相机状态
        
        Returns:
            相机状态字典，key 为 camera_id
        """
        result = {}
        for camera_id in self.camera_topics:
            status = self.get_camera_status(camera_id)
            if status:
                result[camera_id] = status
        return result
    
    def get_available_cameras(self) -> List[str]:
        """获取当前在线的相机列表
        
        Returns:
            在线相机的 ID 列表
        """
        available = []
        for camera_id in self.camera_topics:
            status = self.get_camera_status(camera_id)
            if status and status.get("available"):
                available.append(camera_id)
        return available
    
    def get_camera_topics(self) -> Dict[str, str]:
        """获取相机话题配置
        
        Returns:
            相机 ID 到话题名的映射
        """
        return self.camera_topics.copy()
    
    def add_camera(self, camera_id: str, topic: str):
        """动态添加相机
        
        Args:
            camera_id: 相机标识
            topic: ROS 话题名
        """
        if camera_id in self.camera_topics:
            print(f"⚠️  相机 {camera_id} 已存在，将更新话题")
        
        self.camera_topics[camera_id] = topic
        self.camera_last_msg_time[camera_id] = 0.0
        self.camera_frame_count[camera_id] = 0
        self.camera_fps[camera_id] = 0.0
        
        # 如果节点已初始化，创建新的订阅器
        if self.node:
            try:
                from sensor_msgs.msg import Image
                
                def callback(msg: Image):
                    now = time.time()
                    self.camera_last_msg_time[camera_id] = now
                    self.camera_frame_count[camera_id] += 1
                
                self.node.create_subscription(Image, topic, callback, 1)
                print(f"✅ 动态添加相机订阅器: {camera_id} -> {topic}")
            except Exception as e:
                print(f"⚠️  动态添加相机订阅器失败: {e}")
    
    def remove_camera(self, camera_id: str):
        """移除相机（注意：ROS2 不支持动态销毁订阅器，只是从状态中移除）
        
        Args:
            camera_id: 相机标识
        """
        if camera_id in self.camera_topics:
            del self.camera_topics[camera_id]
            del self.camera_last_msg_time[camera_id]
            del self.camera_frame_count[camera_id]
            del self.camera_fps[camera_id]
            print(f"✅ 相机 {camera_id} 已从状态中移除")
    
    def set_timeout(self, seconds: float):
        """设置超时时间
        
        Args:
            seconds: 超时秒数
        """
        self.timeout_seconds = max(0.5, seconds)  # 最小 0.5 秒
    
    def is_camera_available(self, camera_id: str) -> bool:
        """快速检查相机是否在线
        
        Args:
            camera_id: 相机标识
            
        Returns:
            是否在线
        """
        status = self.get_camera_status(camera_id)
        return status is not None and status.get("available", False)
