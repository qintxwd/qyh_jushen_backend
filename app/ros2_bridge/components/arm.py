"""机械臂组件 - 管理双臂状态和控制"""
import time
from typing import Optional, Dict, Any, List
from .base import BridgeComponent


class ArmComponent(BridgeComponent):
    """机械臂控制组件
    
    功能:
    - 机械臂状态订阅和缓存
    - 伺服状态订阅
    - 关节状态管理
    - 运动控制（MoveJ, MoveL, Jog）
    - 负载管理
    """
    
    def __init__(self):
        super().__init__()
        # 状态缓存
        self.arm_state: Optional[Dict[str, Any]] = None
        self.servo_status: Optional[Dict[str, Any]] = None
        self.joint_positions: List[float] = [0.0] * 14  # 14 个关节
        
        # 3D 场景用的关节状态
        self.joint_states_for_3d: Dict[str, Any] = {
            "timestamp": 0.0,
            "left": [0.0] * 7,
            "right": [0.0] * 7,
            "names": {
                "left": [f"left_joint{i+1}" for i in range(7)],
                "right": [f"right_joint{i+1}" for i in range(7)]
            }
        }
        
        # URDF 描述
        self.robot_description: Optional[str] = None
        
        # 服务客户端
        self.power_on_client = None
        self.power_off_client = None
        self.enable_client = None
        self.disable_client = None
        self.clear_error_client = None
        self.motion_abort_client = None
        self.start_servo_client = None
        self.stop_servo_client = None
        self.move_j_client = None
        self.move_l_client = None
        self.jog_client = None
        self.jog_stop_client = None
        self.set_payload_client = None
        self.get_payload_client = None
        
        # 发布器
        self.arm_command_pub = None
    
    def setup_subscribers(self):
        """设置机械臂相关订阅器"""
        self._setup_robot_description_subscriber()
        self._setup_robot_state_subscriber()
        self._setup_servo_status_subscriber()
    
    def _setup_robot_description_subscriber(self):
        """设置机器人描述订阅器"""
        try:
            from std_msgs.msg import String
            
            def callback(msg):
                self.robot_description = msg.data
                print("✅ 收到 robot_description")
            
            self.node.create_subscription(String, '/robot_description', callback, 10)
            print("✅ robot_description 订阅器创建成功")
        except Exception as e:
            print(f"⚠️  robot_description 订阅器创建失败: {e}")
    
    def _setup_robot_state_subscriber(self):
        """设置机械臂状态订阅器"""
        try:
            from qyh_jaka_control_msgs.msg import RobotState
            
            def callback(msg):
                left_joints = list(msg.left_joint_positions) if len(msg.left_joint_positions) == 7 else [0.0] * 7
                right_joints = list(msg.right_joint_positions) if len(msg.right_joint_positions) == 7 else [0.0] * 7
                
                self.arm_state = {
                    "connected": msg.connected,
                    "robot_ip": msg.robot_ip,
                    "powered_on": msg.powered_on,
                    "enabled": msg.enabled,
                    "in_estop": msg.in_estop,
                    "in_error": msg.in_error,
                    "servo_mode_enabled": msg.servo_mode_enabled,
                    "error_message": msg.error_message,
                    "left_in_position": msg.left_in_position,
                    "right_in_position": msg.right_in_position,
                    "left_joint_positions": left_joints,
                    "right_joint_positions": right_joints
                }
                
                # 更新关节位置
                self.joint_positions = left_joints + right_joints
                
                # 更新 3D 场景用的关节数据
                self.joint_states_for_3d = {
                    "timestamp": time.time(),
                    "left": left_joints,
                    "right": right_joints,
                    "names": {
                        "left": [f"left_joint{i+1}" for i in range(7)],
                        "right": [f"right_joint{i+1}" for i in range(7)]
                    }
                }
            
            self.node.create_subscription(RobotState, '/jaka/robot_state', callback, 10)
            print("✅ 机械臂状态订阅器创建成功: /jaka/robot_state")
        except Exception as e:
            print(f"⚠️  机械臂状态订阅器创建失败: {e}")
    
    def _setup_servo_status_subscriber(self):
        """设置伺服状态订阅器"""
        try:
            from qyh_jaka_control_msgs.msg import JakaServoStatus
            
            def callback(msg):
                self.servo_status = {
                    "mode": msg.mode,
                    "is_abs": msg.is_abs,
                    "cycle_time_ns": msg.cycle_time_ns,
                    "publish_rate_hz": msg.publish_rate_hz,
                    "latency_ms": msg.latency_ms,
                    "packet_loss_rate": msg.packet_loss_rate,
                    "error_code": msg.error_code
                }
            
            self.node.create_subscription(JakaServoStatus, '/jaka/servo/status', callback, 10)
            print("✅ 伺服状态订阅器创建成功: /jaka/servo/status")
        except Exception as e:
            print(f"⚠️  伺服状态订阅器创建失败: {e}")
    
    def setup_publishers(self):
        """设置机械臂发布器和服务客户端"""
        self._setup_arm_publisher()
        self._setup_service_clients()
    
    def _setup_arm_publisher(self):
        """设置机械臂命令发布器"""
        try:
            from std_msgs.msg import Float64MultiArray
            self.arm_command_pub = self.node.create_publisher(
                Float64MultiArray, '/arm/joint_command', 10
            )
            print("✅ 机械臂命令发布器创建成功")
        except Exception as e:
            print(f"⚠️  机械臂命令发布器创建失败: {e}")
    
    def _setup_service_clients(self):
        """设置服务客户端"""
        try:
            from std_srvs.srv import Trigger
            from qyh_jaka_control_msgs.srv import (
                StartServo, StopServo, MoveJ, MoveL, Jog, JogStop, SetPayload, GetPayload
            )
            
            # 基础控制服务
            self.power_on_client = self.node.create_client(Trigger, '/jaka/robot/power_on')
            self.power_off_client = self.node.create_client(Trigger, '/jaka/robot/power_off')
            self.enable_client = self.node.create_client(Trigger, '/jaka/robot/enable')
            self.disable_client = self.node.create_client(Trigger, '/jaka/robot/disable')
            self.clear_error_client = self.node.create_client(Trigger, '/jaka/robot/clear_error')
            self.motion_abort_client = self.node.create_client(Trigger, '/jaka/robot/motion_abort')
            
            # 伺服控制服务
            self.start_servo_client = self.node.create_client(StartServo, '/jaka/servo/start')
            self.stop_servo_client = self.node.create_client(StopServo, '/jaka/servo/stop')
            
            # 运动控制服务
            self.move_j_client = self.node.create_client(MoveJ, '/jaka/move_j')
            self.move_l_client = self.node.create_client(MoveL, '/jaka/move_l')
            self.jog_client = self.node.create_client(Jog, '/jaka/jog')
            self.jog_stop_client = self.node.create_client(JogStop, '/jaka/jog_stop')
            
            # 负载管理服务
            self.set_payload_client = self.node.create_client(SetPayload, '/jaka/set_payload')
            self.get_payload_client = self.node.create_client(GetPayload, '/jaka/get_payload')
            
            print("✅ 机械臂服务客户端创建成功")
        except Exception as e:
            print(f"⚠️  机械臂服务客户端创建失败: {e}")
    
    # ==================== 命令处理 ====================
    
    def handle_command(self, cmd: Dict[str, Any]) -> bool:
        """处理机械臂相关命令"""
        cmd_type = cmd.get('type', '')
        
        handlers = {
            'arm_trigger_service': self._handle_trigger_service,
            'arm_move_j': self._handle_move_j,
            'arm_move_l': self._handle_move_l,
            'arm_jog': self._handle_jog,
            'arm_jog_stop': self._handle_jog_stop,
            'arm_set_payload': self._handle_set_payload,
            'arm_get_payload': self._handle_get_payload,
        }
        
        handler = handlers.get(cmd_type)
        if handler:
            handler(cmd)
            return True
        return False
    
    def _handle_trigger_service(self, cmd: Dict[str, Any]):
        """处理 Trigger 类型服务"""
        service_name = cmd['params']['service_name']
        
        # 获取对应客户端
        client_map = {
            'power_on': self.power_on_client,
            'power_off': self.power_off_client,
            'enable': self.enable_client,
            'disable': self.disable_client,
            'clear_error': self.clear_error_client,
            'motion_abort': self.motion_abort_client,
        }
        
        client = client_map.get(service_name)
        
        # 伺服服务需要特殊处理
        if service_name == 'start_servo':
            self._handle_start_servo(cmd)
            return
        elif service_name == 'stop_servo':
            self._handle_stop_servo(cmd)
            return
        
        if not client:
            self._send_result(cmd, {'success': False, 'message': f'未知服务: {service_name}'})
            return
        
        if not client.wait_for_service(timeout_sec=1.0):
            self._send_result(cmd, {'success': False, 'message': f'服务不可用: {service_name}'})
            return
        
        from std_srvs.srv import Trigger
        req = Trigger.Request()
        future = client.call_async(req)
        
        def done_callback(f):
            try:
                resp = f.result()
                result = {'success': resp.success, 'message': resp.message}
            except Exception as e:
                result = {'success': False, 'message': str(e)}
            self._send_result(cmd, result)
        
        future.add_done_callback(done_callback)
    
    def _handle_start_servo(self, cmd: Dict[str, Any]):
        """处理启动伺服"""
        if not self.start_servo_client:
            self._send_result(cmd, {'success': False, 'message': '伺服启动客户端未初始化'})
            return
        
        if not self.start_servo_client.wait_for_service(timeout_sec=1.0):
            self._send_result(cmd, {'success': False, 'message': '伺服启动服务不可用'})
            return
        
        from qyh_jaka_control_msgs.srv import StartServo
        req = StartServo.Request()
        future = self.start_servo_client.call_async(req)
        future.add_done_callback(lambda f: self._handle_service_response(cmd, f))
    
    def _handle_stop_servo(self, cmd: Dict[str, Any]):
        """处理停止伺服"""
        if not self.stop_servo_client:
            self._send_result(cmd, {'success': False, 'message': '伺服停止客户端未初始化'})
            return
        
        if not self.stop_servo_client.wait_for_service(timeout_sec=1.0):
            self._send_result(cmd, {'success': False, 'message': '伺服停止服务不可用'})
            return
        
        from qyh_jaka_control_msgs.srv import StopServo
        req = StopServo.Request()
        future = self.stop_servo_client.call_async(req)
        future.add_done_callback(lambda f: self._handle_service_response(cmd, f))
    
    def _handle_move_j(self, cmd: Dict[str, Any]):
        """处理 MoveJ"""
        if not self.move_j_client:
            self._send_result(cmd, {'success': False, 'message': 'MoveJ 客户端未初始化'})
            return
        
        if not self.move_j_client.wait_for_service(timeout_sec=1.0):
            self._send_result(cmd, {'success': False, 'message': 'MoveJ 服务不可用'})
            return
        
        from qyh_jaka_control_msgs.srv import MoveJ
        params = cmd['params']
        req = MoveJ.Request()
        req.robot_id = params['robot_id']
        req.joint_positions = params['joint_positions']
        req.velocity = params['velocity']
        req.acceleration = params['acceleration']
        req.is_block = params['is_block']
        
        future = self.move_j_client.call_async(req)
        future.add_done_callback(lambda f: self._handle_service_response(cmd, f))
    
    def _handle_move_l(self, cmd: Dict[str, Any]):
        """处理 MoveL"""
        if not self.move_l_client:
            self._send_result(cmd, {'success': False, 'message': 'MoveL 客户端未初始化'})
            return
        
        if not self.move_l_client.wait_for_service(timeout_sec=1.0):
            self._send_result(cmd, {'success': False, 'message': 'MoveL 服务不可用'})
            return
        
        from qyh_jaka_control_msgs.srv import MoveL
        from geometry_msgs.msg import Pose
        params = cmd['params']
        
        req = MoveL.Request()
        req.robot_id = params['robot_id']
        req.target_pose = Pose()
        req.target_pose.position.x = params['x']
        req.target_pose.position.y = params['y']
        req.target_pose.position.z = params['z']
        req.target_pose.orientation.x = params['rx']
        req.target_pose.orientation.y = params['ry']
        req.target_pose.orientation.z = params['rz']
        req.target_pose.orientation.w = 1.0
        req.velocity = params['velocity']
        req.acceleration = params['acceleration']
        req.is_block = params['is_block']
        
        future = self.move_l_client.call_async(req)
        future.add_done_callback(lambda f: self._handle_service_response(cmd, f))
    
    def _handle_jog(self, cmd: Dict[str, Any]):
        """处理 Jog"""
        if not self.jog_client:
            self._send_result(cmd, {'success': False, 'message': 'Jog 客户端未初始化'})
            return
        
        if not self.jog_client.wait_for_service(timeout_sec=1.0):
            self._send_result(cmd, {'success': False, 'message': 'Jog 服务不可用'})
            return
        
        from qyh_jaka_control_msgs.srv import Jog
        params = cmd['params']
        
        req = Jog.Request()
        req.robot_id = params['robot_id']
        req.axis_num = params['axis_num']
        req.move_mode = params['move_mode']
        req.coord_type = params['coord_type']
        req.velocity = params['velocity']
        req.position = params['position']
        
        future = self.jog_client.call_async(req)
        future.add_done_callback(lambda f: self._handle_service_response(cmd, f))
    
    def _handle_jog_stop(self, cmd: Dict[str, Any]):
        """处理 JogStop"""
        if not self.jog_stop_client:
            self._send_result(cmd, {'success': False, 'message': 'JogStop 客户端未初始化'})
            return
        
        if not self.jog_stop_client.wait_for_service(timeout_sec=1.0):
            self._send_result(cmd, {'success': False, 'message': 'JogStop 服务不可用'})
            return
        
        from qyh_jaka_control_msgs.srv import JogStop
        params = cmd['params']
        
        req = JogStop.Request()
        req.robot_id = params['robot_id']
        req.axis_num = params['axis_num']
        
        future = self.jog_stop_client.call_async(req)
        future.add_done_callback(lambda f: self._handle_service_response(cmd, f))
    
    def _handle_set_payload(self, cmd: Dict[str, Any]):
        """处理设置负载"""
        if not self.set_payload_client:
            self._send_result(cmd, {'success': False, 'message': 'SetPayload 客户端未初始化'})
            return
        
        if not self.set_payload_client.wait_for_service(timeout_sec=1.0):
            self._send_result(cmd, {'success': False, 'message': 'SetPayload 服务不可用'})
            return
        
        from qyh_jaka_control_msgs.srv import SetPayload
        params = cmd['params']
        
        req = SetPayload.Request()
        req.robot_id = params['robot_id']
        req.mass = params['mass']
        
        future = self.set_payload_client.call_async(req)
        future.add_done_callback(lambda f: self._handle_service_response(cmd, f))
    
    def _handle_get_payload(self, cmd: Dict[str, Any]):
        """处理获取负载"""
        if not self.get_payload_client:
            self._send_result(cmd, {'success': False, 'message': 'GetPayload 客户端未初始化'})
            return
        
        if not self.get_payload_client.wait_for_service(timeout_sec=1.0):
            self._send_result(cmd, {'success': False, 'message': 'GetPayload 服务不可用'})
            return
        
        from qyh_jaka_control_msgs.srv import GetPayload
        params = cmd['params']
        
        req = GetPayload.Request()
        req.robot_id = params['robot_id']
        
        future = self.get_payload_client.call_async(req)
        
        def done_callback(f):
            try:
                resp = f.result()
                result = {
                    'success': resp.success,
                    'message': resp.message,
                    'mass': resp.mass,
                    'centroid_x': resp.centroid_x,
                    'centroid_y': resp.centroid_y,
                    'centroid_z': resp.centroid_z
                }
            except Exception as e:
                result = {'success': False, 'message': str(e)}
            self._send_result(cmd, result)
        
        future.add_done_callback(done_callback)
    
    def _handle_service_response(self, cmd: Dict[str, Any], future):
        """处理服务响应"""
        try:
            resp = future.result()
            result = {
                'success': getattr(resp, 'success', True),
                'message': getattr(resp, 'message', '操作成功')
            }
        except Exception as e:
            result = {'success': False, 'message': str(e)}
        self._send_result(cmd, result)
    
    # ==================== 状态获取 ====================
    
    def get_state(self) -> Dict[str, Any]:
        """获取机械臂状态"""
        state = {}
        if self.arm_state:
            state = self.arm_state.copy()
        else:
            state = {
                "connected": False,
                "robot_ip": "",
                "powered_on": False,
                "enabled": False,
                "in_estop": False,
                "in_error": False,
                "servo_mode_enabled": False,
                "error_message": "ROS2未连接" if self.mock_mode else "",
                "left_in_position": True,
                "right_in_position": True
            }
        
        # 确保关节位置字段始终存在
        if len(self.joint_positions) >= 14:
            state['left_joint_positions'] = list(self.joint_positions[:7])
            state['right_joint_positions'] = list(self.joint_positions[7:14])
        else:
            state.setdefault('left_joint_positions', [0.0] * 7)
            state.setdefault('right_joint_positions', [0.0] * 7)
        
        return state
    
    def get_servo_status(self) -> Dict[str, Any]:
        """获取伺服状态"""
        if self.servo_status:
            return self.servo_status
        
        return {
            "mode": "idle",
            "is_abs": True,
            "cycle_time_ns": 8000000,
            "publish_rate_hz": 125.0,
            "latency_ms": 0.0,
            "packet_loss_rate": 0.0,
            "error_code": 0
        }
    
    def get_joint_states(self) -> Dict[str, Any]:
        """获取 3D 场景用的关节状态"""
        return self.joint_states_for_3d
    
    def get_robot_description(self) -> Optional[str]:
        """获取机器人 URDF 描述"""
        return self.robot_description
    
    # ==================== 异步接口 ====================
    
    async def call_service(self, service_name: str) -> Optional[Dict[str, Any]]:
        """调用机械臂基础服务"""
        if self.mock_mode:
            return None
        
        return await self._create_async_command('arm_trigger_service', {'service_name': service_name})
    
    async def move_j(
        self,
        robot_id: int,
        joint_positions: list,
        velocity: float,
        acceleration: float,
        is_block: bool
    ) -> Optional[Dict[str, Any]]:
        """MoveJ 运动"""
        if self.mock_mode:
            return None
        
        return await self._create_async_command('arm_move_j', {
            'robot_id': robot_id,
            'joint_positions': joint_positions,
            'velocity': velocity,
            'acceleration': acceleration,
            'is_block': is_block
        })
    
    async def move_l(
        self,
        robot_id: int,
        x: float, y: float, z: float,
        rx: float, ry: float, rz: float,
        velocity: float,
        acceleration: float,
        is_block: bool
    ) -> Optional[Dict[str, Any]]:
        """MoveL 运动"""
        if self.mock_mode:
            return None
        
        return await self._create_async_command('arm_move_l', {
            'robot_id': robot_id,
            'x': x, 'y': y, 'z': z,
            'rx': rx, 'ry': ry, 'rz': rz,
            'velocity': velocity,
            'acceleration': acceleration,
            'is_block': is_block
        })
    
    async def jog(
        self,
        robot_id: int,
        axis_num: int,
        move_mode: int,
        coord_type: int,
        velocity: float,
        position: float
    ) -> Optional[Dict[str, Any]]:
        """Jog 点动控制"""
        if self.mock_mode:
            return None
        
        return await self._create_async_command('arm_jog', {
            'robot_id': robot_id,
            'axis_num': axis_num,
            'move_mode': move_mode,
            'coord_type': coord_type,
            'velocity': velocity,
            'position': position
        })
    
    async def jog_stop(self, robot_id: int, axis_num: int = 0) -> Optional[Dict[str, Any]]:
        """停止 Jog"""
        if self.mock_mode:
            return None
        
        return await self._create_async_command('arm_jog_stop', {
            'robot_id': robot_id,
            'axis_num': axis_num
        })
    
    async def set_payload(self, robot_id: int, mass: float) -> Optional[Dict[str, Any]]:
        """设置负载"""
        if self.mock_mode:
            return None
        
        return await self._create_async_command('arm_set_payload', {
            'robot_id': robot_id,
            'mass': mass
        })
    
    async def get_payload(self, robot_id: int) -> Optional[Dict[str, Any]]:
        """获取负载"""
        if self.mock_mode:
            return None
        
        return await self._create_async_command('arm_get_payload', {'robot_id': robot_id})
