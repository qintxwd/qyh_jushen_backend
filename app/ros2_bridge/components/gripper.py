"""夹爪控制组件"""
from typing import Optional, Dict, Any
from .base import BridgeComponent


class GripperComponent(BridgeComponent):
    """夹爪控制组件
    
    功能:
    - 左右夹爪状态订阅
    - 夹爪激活和移动控制
    - LED 灯带控制（共用 485 总线）
    """
    
    def __init__(self):
        super().__init__()
        self.left_gripper_state: Optional[Dict[str, Any]] = None
        self.right_gripper_state: Optional[Dict[str, Any]] = None
        
        # 服务客户端
        self.left_activate_client = None
        self.left_move_client = None
        self.right_activate_client = None
        self.right_move_client = None
        
        # LED 发布器
        self.led_color_pub = None
        self.led_blink_pub = None
    
    def setup_subscribers(self):
        """设置夹爪状态订阅器"""
        self._setup_left_gripper_subscriber()
        self._setup_right_gripper_subscriber()
    
    def _setup_left_gripper_subscriber(self):
        """设置左夹爪订阅器"""
        try:
            from qyh_gripper_msgs.msg import GripperState
            
            def callback(msg):
                self.left_gripper_state = {
                    "is_activated": msg.is_activated,
                    "is_moving": msg.is_moving,
                    "object_status": msg.object_status,
                    "target_position": msg.target_position,
                    "current_position": msg.current_position,
                    "target_speed": msg.target_speed,
                    "current_speed": msg.current_speed,
                    "target_force": msg.target_force,
                    "current_force": msg.current_force,
                    "fault_code": msg.fault_code,
                    "fault_message": msg.fault_message,
                    "communication_ok": msg.communication_ok
                }
            
            self.node.create_subscription(GripperState, '/left/gripper_state', callback, 10)
            print("✅ 左夹爪订阅器创建成功: /left/gripper_state")
        except Exception as e:
            print(f"⚠️  左夹爪订阅器创建失败: {e}")
    
    def _setup_right_gripper_subscriber(self):
        """设置右夹爪订阅器"""
        try:
            from qyh_gripper_msgs.msg import GripperState
            
            def callback(msg):
                self.right_gripper_state = {
                    "is_activated": msg.is_activated,
                    "is_moving": msg.is_moving,
                    "object_status": msg.object_status,
                    "target_position": msg.target_position,
                    "current_position": msg.current_position,
                    "target_speed": msg.target_speed,
                    "current_speed": msg.current_speed,
                    "target_force": msg.target_force,
                    "current_force": msg.current_force,
                    "fault_code": msg.fault_code,
                    "fault_message": msg.fault_message,
                    "communication_ok": msg.communication_ok
                }
            
            self.node.create_subscription(GripperState, '/right/gripper_state', callback, 10)
            print("✅ 右夹爪订阅器创建成功: /right/gripper_state")
        except Exception as e:
            print(f"⚠️  右夹爪订阅器创建失败: {e}")
    
    def setup_publishers(self):
        """设置夹爪服务客户端和 LED 发布器"""
        self._setup_gripper_clients()
        self._setup_led_publishers()
    
    def _setup_gripper_clients(self):
        """设置夹爪服务客户端"""
        try:
            from qyh_gripper_msgs.srv import ActivateGripper, MoveGripper
            
            self.left_activate_client = self.node.create_client(
                ActivateGripper, '/left/activate_gripper')
            self.left_move_client = self.node.create_client(
                MoveGripper, '/left/move_gripper')
            self.right_activate_client = self.node.create_client(
                ActivateGripper, '/right/activate_gripper')
            self.right_move_client = self.node.create_client(
                MoveGripper, '/right/move_gripper')
            
            print("✅ 夹爪服务客户端创建成功")
        except Exception as e:
            print(f"⚠️  夹爪服务客户端创建失败: {e}")
    
    def _setup_led_publishers(self):
        """设置 LED 发布器"""
        try:
            from std_msgs.msg import ColorRGBA, String
            
            self.led_color_pub = self.node.create_publisher(
                ColorRGBA, '/robot_led/set_color', 10)
            self.led_blink_pub = self.node.create_publisher(
                String, '/robot_led/blink', 10)
            print("✅ LED发布器创建成功")
        except Exception as e:
            print(f"⚠️  LED发布器创建失败: {e}")
    
    def handle_command(self, cmd: Dict[str, Any]) -> bool:
        """处理夹爪相关命令"""
        cmd_type = cmd.get('type', '')
        
        if cmd_type == 'gripper_activate':
            self._handle_activate(cmd)
            return True
        elif cmd_type == 'gripper_move':
            self._handle_move(cmd)
            return True
        
        return False
    
    def _handle_activate(self, cmd: Dict[str, Any]):
        """处理夹爪激活"""
        side = cmd['params']['side']
        client = self.left_activate_client if side == 'left' else self.right_activate_client
        
        if not client:
            self._send_result(cmd, {'success': False, 'message': '夹爪激活客户端未初始化'})
            return
        
        if not client.wait_for_service(timeout_sec=1.0):
            self._send_result(cmd, {'success': False, 'message': '夹爪激活服务不可用'})
            return
        
        from qyh_gripper_msgs.srv import ActivateGripper
        req = ActivateGripper.Request()
        future = client.call_async(req)
        
        def done_callback(f):
            try:
                resp = f.result()
                result = {'success': resp.success, 'message': resp.message}
            except Exception as e:
                result = {'success': False, 'message': str(e)}
            self._send_result(cmd, result)
        
        future.add_done_callback(done_callback)
    
    def _handle_move(self, cmd: Dict[str, Any]):
        """处理夹爪移动"""
        params = cmd['params']
        side = params['side']
        client = self.left_move_client if side == 'left' else self.right_move_client
        
        if not client:
            self._send_result(cmd, {'success': False, 'message': '夹爪移动客户端未初始化'})
            return
        
        if not client.wait_for_service(timeout_sec=1.0):
            self._send_result(cmd, {'success': False, 'message': '夹爪移动服务不可用'})
            return
        
        from qyh_gripper_msgs.srv import MoveGripper
        req = MoveGripper.Request()
        req.position = int(params['position'])
        req.speed = int(params['speed'])
        req.force = int(params['force'])
        
        future = client.call_async(req)
        
        def done_callback(f):
            try:
                resp = f.result()
                result = {'success': resp.success, 'message': resp.message}
            except Exception as e:
                result = {'success': False, 'message': str(e)}
            self._send_result(cmd, result)
        
        future.add_done_callback(done_callback)
    
    # ==================== 状态获取 ====================
    
    def get_left_state(self) -> Optional[Dict[str, Any]]:
        """获取左夹爪状态"""
        if self.mock_mode:
            return None
        return self.left_gripper_state
    
    def get_right_state(self) -> Optional[Dict[str, Any]]:
        """获取右夹爪状态"""
        if self.mock_mode:
            return None
        return self.right_gripper_state
    
    # ==================== LED 控制 ====================
    
    def set_led_color(self, r: int, g: int, b: int, w: int = 0) -> bool:
        """设置 LED 纯色"""
        if self.mock_mode:
            print(f"[Mock] LED color: R={r}, G={g}, B={b}, W={w}")
            return True
        
        if not self.led_color_pub:
            print("⚠️  LED发布器未初始化")
            return False
        
        try:
            from std_msgs.msg import ColorRGBA
            msg = ColorRGBA()
            msg.r = r / 255.0
            msg.g = g / 255.0
            msg.b = b / 255.0
            msg.a = w / 255.0
            self.led_color_pub.publish(msg)
            return True
        except Exception as e:
            print(f"❌ LED颜色设置失败: {e}")
            return False
    
    def set_led_blink(self, command: str) -> bool:
        """设置 LED 闪烁模式"""
        if self.mock_mode:
            print(f"[Mock] LED blink: {command}")
            return True
        
        if not self.led_blink_pub:
            print("⚠️  LED闪烁发布器未初始化")
            return False
        
        try:
            from std_msgs.msg import String
            msg = String()
            msg.data = command
            self.led_blink_pub.publish(msg)
            return True
        except Exception as e:
            print(f"❌ LED闪烁命令发送失败: {e}")
            return False
    
    # ==================== 异步接口 ====================
    
    async def activate(self, side: str) -> Optional[Dict[str, Any]]:
        """激活夹爪"""
        if self.mock_mode:
            return None
        
        return await self._create_async_command('gripper_activate', {'side': side})
    
    async def move(
        self,
        side: str,
        position: int,
        speed: int,
        force: int
    ) -> Optional[Dict[str, Any]]:
        """移动夹爪"""
        if self.mock_mode:
            return None
        
        return await self._create_async_command('gripper_move', {
            'side': side,
            'position': position,
            'speed': speed,
            'force': force
        })
