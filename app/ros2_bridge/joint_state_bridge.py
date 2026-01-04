"""
ROS2 Joint State Bridge - 将 /joint_states 话题转发到 WebSocket
"""
import asyncio
import json
from typing import Optional, Set
from fastapi import WebSocket
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class JointStateBridge(Node):
    """ROS2 Joint State 桥接节点"""
    
    def __init__(self):
        super().__init__('joint_state_bridge')
        
        # WebSocket 连接集合
        self.websocket_clients: Set[WebSocket] = set()
        
        # 订阅 /joint_states 话题
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        
        # 最新的 joint state
        self.latest_joint_state: Optional[dict] = None
        
        self.get_logger().info('Joint State Bridge initialized')
    
    def joint_state_callback(self, msg: JointState):
        """接收 joint state 消息并转换为字典"""
        try:
            # 转换为字典
            joint_state_dict = {
                'topic': '/joint_states',
                'msg': {
                    'header': {
                        'stamp': {
                            'sec': msg.header.stamp.sec,
                            'nanosec': msg.header.stamp.nanosec
                        },
                        'frame_id': msg.header.frame_id
                    },
                    'name': list(msg.name),
                    'position': list(msg.position),
                    'velocity': list(msg.velocity) if msg.velocity else [],
                    'effort': list(msg.effort) if msg.effort else []
                }
            }
            
            self.latest_joint_state = joint_state_dict
            
            # 异步发送到所有 WebSocket 客户端
            if self.websocket_clients:
                asyncio.create_task(self.broadcast_to_clients(joint_state_dict))
                
        except Exception as e:
            self.get_logger().error(f'Error processing joint state: {e}')
    
    async def broadcast_to_clients(self, data: dict):
        """广播数据到所有 WebSocket 客户端"""
        disconnected_clients = set()
        
        for client in self.websocket_clients:
            try:
                await client.send_json(data)
            except Exception as e:
                self.get_logger().warning(f'Failed to send to client: {e}')
                disconnected_clients.add(client)
        
        # 移除断开的客户端
        self.websocket_clients -= disconnected_clients
    
    def add_client(self, websocket: WebSocket):
        """添加 WebSocket 客户端"""
        self.websocket_clients.add(websocket)
        self.get_logger().info(f'Client added. Total clients: {len(self.websocket_clients)}')
        
        # 如果有最新数据，立即发送
        if self.latest_joint_state:
            asyncio.create_task(websocket.send_json(self.latest_joint_state))
    
    def remove_client(self, websocket: WebSocket):
        """移除 WebSocket 客户端"""
        if websocket in self.websocket_clients:
            self.websocket_clients.remove(websocket)
            self.get_logger().info(f'Client removed. Total clients: {len(self.websocket_clients)}')


# 全局桥接实例
_bridge_instance: Optional[JointStateBridge] = None
_bridge_thread: Optional[asyncio.Task] = None


def get_bridge() -> JointStateBridge:
    """获取桥接实例（单例）"""
    global _bridge_instance
    
    if _bridge_instance is None:
        # 初始化 rclpy (如果未初始化)
        if not rclpy.ok():
            rclpy.init()
        
        # 创建桥接节点
        _bridge_instance = JointStateBridge()
        
        # 启动 ROS2 spin 线程
        import threading
        def spin_node():
            try:
                rclpy.spin(_bridge_instance)
            except Exception as e:
                print(f'ROS2 spin error: {e}')
        
        thread = threading.Thread(target=spin_node, daemon=True)
        thread.start()
    
    return _bridge_instance


def cleanup_bridge():
    """清理桥接资源"""
    global _bridge_instance, _bridge_thread
    
    if _bridge_instance:
        _bridge_instance.destroy_node()
        _bridge_instance = None
    
    if _bridge_thread:
        _bridge_thread.cancel()
        _bridge_thread = None
    
    if rclpy.ok():
        rclpy.shutdown()
