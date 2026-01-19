"""升降柱控制组件"""
from typing import Optional, Dict, Any
from .base import BridgeComponent


class LiftComponent(BridgeComponent):
    """升降柱控制组件
    
    功能:
    - 升降柱状态订阅
    - 升降柱控制服务调用
    - 硬件关机请求检测
    """
    
    def __init__(self):
        super().__init__()
        self.lift_state: Optional[Dict[str, Any]] = None
        self.lift_client = None
        self.shutdown_requested: bool = False
    
    def setup_subscribers(self):
        """设置升降柱状态订阅器"""
        try:
            from qyh_lift_msgs.msg import LiftState
            
            def callback(msg):
                self.lift_state = {
                    "connected": msg.connected,
                    "enabled": msg.enabled,
                    "current_position": msg.current_position,
                    "current_speed": msg.current_speed,
                    "position_reached": msg.position_reached,
                    "alarm": msg.alarm,
                    "electromagnet_on": msg.electromagnet_on,
                    "shutdown_requested": msg.shutdown_requested
                }
                
                if msg.shutdown_requested:
                    print("⚠️ 检测到硬件关机请求！")
                    self.shutdown_requested = True
            
            self.node.create_subscription(LiftState, '/lift/state', callback, 10)
            print("✅ 升降机订阅器创建成功: /lift/state")
        except Exception as e:
            print(f"⚠️  升降机订阅器创建失败: {e}")
    
    def setup_publishers(self):
        """设置升降柱服务客户端"""
        try:
            from qyh_lift_msgs.srv import LiftControl
            self.lift_client = self.node.create_client(LiftControl, '/lift/control')
            print("✅ 升降机控制客户端创建成功")
        except Exception as e:
            print(f"⚠️  升降机客户端创建失败: {e}")
    
    def handle_command(self, cmd: Dict[str, Any]) -> bool:
        """处理升降柱控制命令"""
        if cmd.get('type') != 'lift_control':
            return False
        
        try:
            from qyh_lift_msgs.srv import LiftControl
            params = cmd['params']
            
            if not self.lift_client:
                self._send_result(cmd, {'success': False, 'message': '升降机客户端未初始化'})
                return True
            
            if not self.lift_client.wait_for_service(timeout_sec=1.0):
                self._send_result(cmd, {'success': False, 'message': '升降机服务不可用'})
                return True
            
            req = LiftControl.Request()
            req.command = params['command']
            req.value = float(params['value'])
            req.hold = params['hold']
            
            future = self.lift_client.call_async(req)
            
            def done_callback(f):
                try:
                    resp = f.result()
                    result = {'success': resp.success, 'message': resp.message}
                except Exception as e:
                    result = {'success': False, 'message': str(e)}
                self._send_result(cmd, result)
            
            future.add_done_callback(done_callback)
        except Exception as e:
            self._send_result(cmd, {'success': False, 'message': str(e)})
        
        return True
    
    def get_state(self) -> Optional[Dict[str, Any]]:
        """获取升降柱状态"""
        if self.mock_mode:
            return None
        return self.lift_state
    
    def is_shutdown_requested(self) -> bool:
        """检查是否有硬件关机请求"""
        return self.shutdown_requested
    
    def clear_shutdown_request(self):
        """清除关机请求标志"""
        self.shutdown_requested = False
    
    async def control(
        self,
        command: int,
        value: float = 0.0,
        hold: bool = False
    ) -> Optional[Dict[str, Any]]:
        """调用升降柱控制服务"""
        if self.mock_mode:
            return None
        
        return await self._create_async_command('lift_control', {
            'command': command,
            'value': value,
            'hold': hold
        })
