"""VR 控制组件"""
from typing import Optional, Dict, Any, List
from .base import BridgeComponent

class VRComponent(BridgeComponent):
    """VR 控制组件
    
    功能:
    - 订阅VR头部位姿
    - 订阅手柄状态（位姿、按键、摇杆）
    """
    
    def __init__(self):
        super().__init__()
        # VR 状态
        self.vr_state = {
            "connected": False,
            "head_position": [0.0, 0.0, 0.0],
            "head_orientation": [0.0, 0.0, 0.0, 1.0],
            "left_hand_active": False,
            "left_position": [0.0, 0.0, 0.0],
            "left_orientation": [0.0, 0.0, 0.0, 1.0],
            "left_joystick": [0.0, 0.0],
            "left_trigger": 0.0,
            "left_grip_value": 0.0,
            "left_clutch_engaged": False,
            "left_buttons": [0, 0, 0, 0, 0, 0],
            "right_hand_active": False,
            "right_position": [0.0, 0.0, 0.0],
            "right_orientation": [0.0, 0.0, 0.0, 1.0],
            "right_joystick": [0.0, 0.0],
            "right_trigger": 0.0,
            "right_grip_value": 0.0,
            "right_clutch_engaged": False,
            "right_buttons": [0, 0, 0, 0, 0, 0]
        }

    def setup_subscribers(self):
        """设置VR遥操作相关订阅器"""
        if self.mock_mode:
            return
            
        try:
            from std_msgs.msg import Bool
            from sensor_msgs.msg import Joy
            from geometry_msgs.msg import PoseStamped
        except ImportError:
            print("⚠️ 缺少消息定义，跳过 VR 订阅器设置")
            return
            
        # ===== 头部位姿订阅 =====
        try:
            def head_pose_callback(msg: PoseStamped):
                self.vr_state['head_position'] = [
                    msg.pose.position.x,
                    msg.pose.position.y,
                    msg.pose.position.z
                ]
                self.vr_state['head_orientation'] = [
                    msg.pose.orientation.x,
                    msg.pose.orientation.y,
                    msg.pose.orientation.z,
                    msg.pose.orientation.w
                ]
                self.vr_state['connected'] = True
            
            self.node.create_subscription(
                PoseStamped,
                '/vr/head/pose',
                head_pose_callback,
                10
            )
            print("✅ VR头部位姿订阅器创建成功: /vr/head/pose")
        except Exception as e:
            print(f"⚠️  VR头部位姿订阅器创建失败: {e}")
        
        # ===== 左手控制器 =====
        # 左手活跃状态
        try:
            def left_active_callback(msg: Bool):
                self.vr_state['left_hand_active'] = msg.data
                self.vr_state['connected'] = True
            
            self.node.create_subscription(
                Bool,
                '/vr/left_controller/active',
                left_active_callback,
                10
            )
            print("✅ 左手活跃状态订阅器创建成功: /vr/left_controller/active")
        except Exception as e:
            print(f"⚠️  左手活跃状态订阅器创建失败: {e}")
        
        # 左手位姿
        try:
            def left_pose_callback(msg: PoseStamped):
                self.vr_state['left_position'] = [
                    msg.pose.position.x,
                    msg.pose.position.y,
                    msg.pose.position.z
                ]
                self.vr_state['left_orientation'] = [
                    msg.pose.orientation.x,
                    msg.pose.orientation.y,
                    msg.pose.orientation.z,
                    msg.pose.orientation.w
                ]
                self.vr_state['connected'] = True
            
            self.node.create_subscription(
                PoseStamped,
                '/vr/left_controller/pose',
                left_pose_callback,
                10
            )
            print("✅ 左手位姿订阅器创建成功: /vr/left_controller/pose")
        except Exception as e:
            print(f"⚠️  左手位姿订阅器创建失败: {e}")
        
        # 左手Joy
        try:
            def left_joy_callback(msg: Joy):
                if len(msg.axes) >= 2:
                    self.vr_state['left_joystick'] = [msg.axes[0], msg.axes[1]]
                if len(msg.axes) >= 3:
                    self.vr_state['left_trigger'] = msg.axes[2]
                if len(msg.axes) >= 4:
                    self.vr_state['left_grip_value'] = msg.axes[3]
                    self.vr_state['left_clutch_engaged'] = msg.axes[3] > 0.8
                if len(msg.buttons) >= 6:
                    self.vr_state['left_buttons'] = list(msg.buttons[:6])
                self.vr_state['connected'] = True
            
            self.node.create_subscription(
                Joy,
                '/vr/left_controller/joy',
                left_joy_callback,
                10
            )
            print("✅ 左手Joy订阅器创建成功: /vr/left_controller/joy")
        except Exception as e:
            print(f"⚠️  左手Joy订阅器创建失败: {e}")
        
        # ===== 右手控制器 =====
        # 右手活跃状态
        try:
            def right_active_callback(msg: Bool):
                self.vr_state['right_hand_active'] = msg.data
                self.vr_state['connected'] = True
            
            self.node.create_subscription(
                Bool,
                '/vr/right_controller/active',
                right_active_callback,
                10
            )
            print("✅ 右手活跃状态订阅器创建成功: /vr/right_controller/active")
        except Exception as e:
            print(f"⚠️  右手活跃状态订阅器创建失败: {e}")
        
        # 右手位姿
        try:
            def right_pose_callback(msg: PoseStamped):
                self.vr_state['right_position'] = [
                    msg.pose.position.x,
                    msg.pose.position.y,
                    msg.pose.position.z
                ]
                self.vr_state['right_orientation'] = [
                    msg.pose.orientation.x,
                    msg.pose.orientation.y,
                    msg.pose.orientation.z,
                    msg.pose.orientation.w
                ]
                self.vr_state['connected'] = True
            
            self.node.create_subscription(
                PoseStamped,
                '/vr/right_controller/pose',
                right_pose_callback,
                10
            )
            print("✅ 右手位姿订阅器创建成功: /vr/right_controller/pose")
        except Exception as e:
            print(f"⚠️  右手位姿订阅器创建失败: {e}")
        
        # 右手Joy
        try:
            def right_joy_callback(msg: Joy):
                if len(msg.axes) >= 2:
                    self.vr_state['right_joystick'] = [msg.axes[0], msg.axes[1]]
                if len(msg.axes) >= 3:
                    self.vr_state['right_trigger'] = msg.axes[2]
                if len(msg.axes) >= 4:
                    self.vr_state['right_grip_value'] = msg.axes[3]
                    self.vr_state['right_clutch_engaged'] = msg.axes[3] > 0.8
                if len(msg.buttons) >= 6:
                    self.vr_state['right_buttons'] = list(msg.buttons[:6])
                self.vr_state['connected'] = True
            
            self.node.create_subscription(
                Joy,
                '/vr/right_controller/joy',
                right_joy_callback,
                10
            )
            print("✅ 右手Joy订阅器创建成功: /vr/right_controller/joy")
        except Exception as e:
            print(f"⚠️  右手Joy订阅器创建失败: {e}")

    def setup_publishers(self):
        pass

    def get_state(self) -> Dict[str, Any]:
        """获取 VR 状态"""
        return self.vr_state
