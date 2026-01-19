"""头部控制组件"""
from typing import Optional, Dict, Any
from .base import BridgeComponent


class HeadComponent(BridgeComponent):
    """头部控制组件
    
    功能:
    - 头部状态订阅（pan/tilt 位置）
    - 头部控制命令发送
    """
    
    def __init__(self):
        super().__init__()
        self.head_state: Optional[Dict[str, Any]] = None
        self.head_cmd_pub = None
    
    def setup_subscribers(self):
        """设置头部状态订阅器"""
        try:
            from sensor_msgs.msg import JointState
            
            def callback(msg):
                if len(msg.position) >= 2 and len(msg.name) >= 2:
                    # 根据 joint name 查找正确的索引
                    pan_idx, tilt_idx = -1, -1
                    for i, name in enumerate(msg.name):
                        if 'pan' in name.lower():
                            pan_idx = i
                        elif 'tilt' in name.lower():
                            tilt_idx = i
                    
                    if pan_idx < 0:
                        pan_idx = 0
                    if tilt_idx < 0:
                        tilt_idx = 1
                    
                    pan_rad = msg.position[pan_idx]
                    tilt_rad = msg.position[tilt_idx]
                    
                    # 归一化到 -1 到 1
                    pan_norm = pan_rad / 1.5708
                    tilt_norm = tilt_rad / 0.7854
                    
                    # 转舵机位置
                    pan_pos = 500 + pan_norm * 400
                    tilt_pos = 500 + tilt_norm * 300
                    
                    self.head_state = {
                        "connected": True,
                        "pan_position": pan_pos,
                        "tilt_position": tilt_pos,
                        "pan_normalized": pan_norm,
                        "tilt_normalized": tilt_norm
                    }
            
            self.node.create_subscription(JointState, '/head/joint_states', callback, 10)
            print("✅ 头部订阅器创建成功: /head/joint_states")
        except Exception as e:
            print(f"⚠️  头部订阅器创建失败: {e}")
    
    def setup_publishers(self):
        """设置头部控制发布器"""
        try:
            from std_msgs.msg import Float64MultiArray
            self.head_cmd_pub = self.node.create_publisher(
                Float64MultiArray, '/head_motor_node/cmd_position', 10
            )
            print("✅ 头部发布器创建成功")
        except Exception as e:
            print(f"⚠️  头部发布器创建失败: {e}")
    
    def handle_command(self, cmd: Dict[str, Any]) -> bool:
        """处理头部控制命令"""
        if cmd.get('type') != 'head_control':
            return False
        
        try:
            from std_msgs.msg import Float64MultiArray
            params = cmd['params']
            pan = params.get('pan')
            tilt = params.get('tilt')
            speed = params.get('speed')
            
            if (pan is not None or tilt is not None) and self.head_cmd_pub:
                # 如果只提供了一个值，使用当前状态填充另一个
                if pan is None:
                    pan = self.head_state.get('pan_normalized', 0.0) if self.head_state else 0.0
                if tilt is None:
                    tilt = self.head_state.get('tilt_normalized', 0.0) if self.head_state else 0.0
                
                msg = Float64MultiArray()
                if speed is not None:
                    speed = max(0.0, min(100.0, float(speed)))
                    duration_ms = 100.0 + (100.0 - speed) * 19.0
                    msg.data = [float(tilt), float(pan), float(duration_ms)]
                else:
                    msg.data = [float(tilt), float(pan)]
                
                self.head_cmd_pub.publish(msg)
            
            self._send_result(cmd, {'success': True, 'message': '头部控制已发送'})
        except Exception as e:
            self._send_result(cmd, {'success': False, 'message': str(e)})
        
        return True
    
    def get_state(self) -> Optional[Dict[str, Any]]:
        """获取头部状态"""
        if self.mock_mode:
            return None
        return self.head_state
    
    async def send_command(
        self,
        pan: Optional[float] = None,
        tilt: Optional[float] = None,
        speed: Optional[float] = None
    ) -> Optional[Dict[str, Any]]:
        """发送头部控制命令"""
        if self.mock_mode:
            return None
        
        return await self._create_async_command('head_control', {
            'pan': pan,
            'tilt': tilt,
            'speed': speed
        })
