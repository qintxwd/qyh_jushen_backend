"""ROS2 Bridge 组件基类"""
from abc import ABC, abstractmethod
from typing import Optional, Dict, Any, Callable
from queue import Queue
import asyncio


class BridgeComponent(ABC):
    """ROS2 Bridge 组件基类
    
    每个组件负责:
    1. 管理自己的状态缓存
    2. 设置相关的订阅器
    3. 设置相关的发布器/服务客户端
    4. 处理相关的命令
    """
    
    def __init__(self):
        self.node = None
        self.command_queue: Optional[Queue] = None
        self.mock_mode: bool = False
    
    def initialize(self, node, command_queue: Queue, mock_mode: bool = False):
        """初始化组件
        
        Args:
            node: ROS2 节点
            command_queue: 命令队列
            mock_mode: 是否为 Mock 模式
        """
        self.node = node
        self.command_queue = command_queue
        self.mock_mode = mock_mode
    
    @abstractmethod
    def setup_subscribers(self):
        """设置 ROS2 订阅器"""
        pass
    
    @abstractmethod
    def setup_publishers(self):
        """设置 ROS2 发布器和服务客户端"""
        pass
    
    def handle_command(self, cmd: Dict[str, Any]) -> bool:
        """处理命令
        
        Args:
            cmd: 命令字典
            
        Returns:
            是否处理了该命令（用于命令分发）
        """
        return False
    
    def _send_result(self, cmd: Dict[str, Any], result: Dict[str, Any]):
        """发送命令执行结果
        
        Args:
            cmd: 原始命令
            result: 执行结果
        """
        if 'loop' in cmd and 'future' in cmd:
            cmd['loop'].call_soon_threadsafe(
                cmd['future'].set_result, result
            )
    
    def _create_async_command(
        self,
        cmd_type: str,
        params: Dict[str, Any]
    ) -> asyncio.Future:
        """创建异步命令
        
        Args:
            cmd_type: 命令类型
            params: 命令参数
            
        Returns:
            Future 对象
        """
        loop = asyncio.get_running_loop()
        future = loop.create_future()
        
        cmd = {
            'type': cmd_type,
            'params': params,
            'future': future,
            'loop': loop
        }
        self.command_queue.put(cmd)
        return future
