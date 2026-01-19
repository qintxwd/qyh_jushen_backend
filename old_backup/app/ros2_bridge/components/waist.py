"""腰部控制组件"""
from typing import Optional, Dict, Any
from .base import BridgeComponent


class WaistComponent(BridgeComponent):
    """腰部控制组件
    
    功能:
    - 腰部状态订阅
    - 腰部控制服务调用
    """
    
    def __init__(self):
        super().__init__()
        self.waist_state: Optional[Dict[str, Any]] = None
        self.waist_client = None
    
    def setup_subscribers(self):
        """设置腰部状态订阅器"""
        try:
            from qyh_waist_msgs.msg import WaistState
            
            def callback(msg):
                self.waist_state = {
                    "connected": msg.connected,
                    "enabled": msg.enabled,
                    "current_position": msg.current_position,
                    "current_angle": msg.current_angle,
                    "current_speed": msg.current_speed,
                    "position_reached": msg.position_reached,
                    "alarm": msg.alarm
                }
            
            self.node.create_subscription(WaistState, '/waist/state', callback, 10)
            print("✅ 腰部状态订阅器创建成功: /waist/state")
        except Exception as e:
            print(f"⚠️  腰部状态订阅器创建失败: {e}")
    
    def setup_publishers(self):
        """设置腰部服务客户端"""
        try:
            from qyh_waist_msgs.srv import WaistControl
            self.waist_client = self.node.create_client(WaistControl, '/waist/control')
            print("✅ 腰部服务客户端创建成功: /waist/control")
        except Exception as e:
            print(f"⚠️  腰部服务客户端创建失败: {e}")
    
    def handle_command(self, cmd: Dict[str, Any]) -> bool:
        """处理腰部控制命令"""
        if cmd.get('type') != 'waist_control':
            return False
        
        try:
            from qyh_waist_msgs.srv import WaistControl
            params = cmd['params']
            
            if not self.waist_client:
                self._send_result(cmd, {'success': False, 'message': '腰部控制客户端未初始化'})
                return True
            
            if not self.waist_client.wait_for_service(timeout_sec=1.0):
                self._send_result(cmd, {'success': False, 'message': '腰部控制服务不可用'})
                return True
            
            req = WaistControl.Request()
            req.command = params['command']
            req.value = float(params['value'])
            req.hold = params['hold']
            
            future = self.waist_client.call_async(req)
            
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
        """获取腰部状态"""
        if self.mock_mode:
            return None
        return self.waist_state
    
    async def control(
        self,
        command: int,
        value: float = 0.0,
        hold: bool = False
    ) -> Optional[Dict[str, Any]]:
        """调用腰部控制服务"""
        if self.mock_mode:
            return None
        
        return await self._create_async_command('waist_control', {
            'command': command,
            'value': value,
            'hold': hold
        })
