"""ROS2 Bridge - 多线程隔离的 ROS2 集成"""
import threading
from queue import Queue
from typing import Optional, Dict, Any, List
from app.config import settings


class ROS2Bridge:
    """ROS2 桥接器（多线程隔离）"""
    
    def __init__(self):
        self.node = None
        self.executor = None
        self.thread: Optional[threading.Thread] = None
        self.running = False
        
        # 线程安全的数据队列
        self.state_queue = Queue(maxsize=10)
        self.command_queue = Queue()
        
        # 升降机状态缓存
        self.lift_state = None
        self.lift_client = None
        # 关机服务客户端 (软件触发关机)
        # self._shutdown_client 在node创建后初始化
        self._shutdown_client = None
        # 关机状态
        self._shutdown_state = {
            "shutdown_in_progress": False,
            "trigger_source": 0,
            "trigger_source_text": "",
            "countdown_seconds": -1,
            "plc_connected": False
        }
        
        # 头部状态缓存
        self.head_state = None
        self.head_cmd_pub = None
        
        # 机械臂状态缓存
        self.arm_state = None
        self.servo_status = None
        self.joint_positions = [0.0] * 14  # 14 个关节
        
        # 3D 场景用的关节状态 (分离左右臂)
        self.joint_states_for_3d = {
            "timestamp": 0.0,
            "left": [0.0] * 7,
            "right": [0.0] * 7,
            "names": {
                "left": [f"left_joint{i+1}" for i in range(7)],
                "right": [f"right_joint{i+1}" for i in range(7)]
            }
        }
        
        # 机器人描述 (URDF)
        self.robot_description = None
        
        # 机械臂服务客户端
        self.arm_power_on_client = None
        self.arm_power_off_client = None
        self.arm_enable_client = None
        self.arm_disable_client = None
        self.arm_clear_error_client = None
        self.arm_motion_abort_client = None
        self.arm_start_servo_client = None
        self.arm_stop_servo_client = None
        self.arm_move_j_client = None
        self.arm_move_l_client = None
        self.arm_jog_client = None
        self.arm_jog_stop_client = None
        self.arm_set_payload_client = None
        self.arm_get_payload_client = None
        
        # 任务引擎服务客户端
        self.task_execute_client = None
        self.task_pause_client = None
        self.task_resume_client = None
        self.task_cancel_client = None
        self.task_status_client = None
        
        # 任务状态缓存
        self.task_status = None
        
        # ==================== 底盘相关 ====================
        # 底盘状态缓存
        self.chassis_status = None
        self.chassis_last_update_time = None
        self.chassis_timeout_seconds = 2.0  # 2秒未收到数据视为断连
        self.navigation_status = None
        
        # 底盘控制发布器
        self.chassis_cmd_vel_pub = None
        self.chassis_manual_pub = None
        self.chassis_velocity_pub = None  # 速度模式发布器
        
        # 底盘控制服务客户端
        self.chassis_clients = {}
        
        # 底盘 Go 系列服务客户端
        self.chassis_go_nav_coord_client = None
        self.chassis_go_nav_site_client = None
        self.chassis_go_force_loc_client = None
        self.chassis_go_set_speed_client = None
        self.chassis_go_set_obstacle_client = None
        self.chassis_go_set_site_client = None
        self.chassis_go_set_volume_client = None
        self.chassis_go_set_map_client = None
        
        # ==================== 夹爪相关 ====================
        # 夹爪状态缓存
        self.left_gripper_state = None
        self.right_gripper_state = None
        
        # ==================== 腰部相关 ====================
        # 腰部状态缓存
        self.waist_state = None
        # 腰部控制服务客户端
        self.waist_control_client = None
        
        # ==================== VR遥操作相关 ====================
        # VR状态缓存
        self.vr_state = {
            "connected": False,
            "left_hand_active": False,
            "right_hand_active": False,
            "left_clutch_engaged": False,
            "right_clutch_engaged": False,
            "left_grip_value": 0.0,
            "right_grip_value": 0.0
        }
        
        # 夹爪服务客户端
        self.left_gripper_activate_client = None
        self.left_gripper_move_client = None
        self.right_gripper_activate_client = None
        self.right_gripper_move_client = None
        
        # 是否为 Mock 模式
        self.mock_mode = settings.MOCK_MODE
    
    def start(self):
        """启动 ROS2 Bridge（在子线程运行）"""
        if self.running:
            return
        
        if self.mock_mode:
            print("🤖 ROS2 Bridge 运行在 Mock 模式")
            self.running = True
            return
        
        self.running = True
        self.thread = threading.Thread(target=self._run_ros2, daemon=True)
        self.thread.start()
        print("✅ ROS2 Bridge 已启动")
    
    def _run_ros2(self):
        """ROS2 事件循环（在独立线程中运行）"""
        try:
            import rclpy
            from rclpy.executors import MultiThreadedExecutor
            from rclpy.node import Node
            
            rclpy.init()
            self.node = Node('web_bridge')
            self.executor = MultiThreadedExecutor()
            self.executor.add_node(self.node)
            
            # 创建订阅器和发布器
            self._setup_subscribers()
            self._setup_publishers()
            
            # 启动命令处理定时器
            self.node.create_timer(0.05, self._process_commands)
            
            # 阻塞式运行
            self.executor.spin()
        
        except Exception as e:
            print(f"❌ ROS2 Bridge 错误: {e}")
            self.running = False
        
        finally:
            if self.node:
                self.node.destroy_node()
            try:
                import rclpy
                rclpy.shutdown()
            except:
                pass
    
    def _setup_subscribers(self):
        """设置 ROS2 订阅器"""
        # /robot_description 订阅 (用于 3D 场景)
        try:
            from std_msgs.msg import String
            
            def robot_description_callback(msg):
                self.robot_description = msg.data
                print("✅ 收到 robot_description")
            
            self.node.create_subscription(
                String,
                '/robot_description',
                robot_description_callback,
                10
            )
            print("✅ robot_description 订阅器创建成功")
        except Exception as e:
            print(f"⚠️  robot_description 订阅器创建失败: {e}")

        # /joint_states 订阅 (用于 3D 场景)
        try:
            from sensor_msgs.msg import JointState
            import time
            import re
            
            def joint_state_callback(msg: JointState):
                # 更新 3D 场景用的关节状态
                left_joints = [0.0] * 7
                right_joints = [0.0] * 7
                
                # 打印接收到的关节名称用于调试
                print(f"📥 收到关节状态: {len(msg.name)} 个关节, 名称: {msg.name[:5] if len(msg.name) > 5 else msg.name}")
                
                for i, name in enumerate(msg.name):
                    if i < len(msg.position):
                        angle = msg.position[i]
                        # 解析关节名称，支持多种格式：
                        # - l-j1, l-j2, ... (URDF格式)
                        # - left_joint1, left_joint_1 (其他格式)
                        left_match = re.match(r'l-j(\d)', name) or re.match(r'left_joint_?(\d)', name)
                        right_match = re.match(r'r-j(\d)', name) or re.match(r'right_joint_?(\d)', name)
                        
                        if left_match:
                            try:
                                idx = int(left_match.group(1)) - 1
                                if 0 <= idx < 7:
                                    left_joints[idx] = angle
                            except ValueError:
                                pass
                        elif right_match:
                            try:
                                idx = int(right_match.group(1)) - 1
                                if 0 <= idx < 7:
                                    right_joints[idx] = angle
                            except ValueError:
                                pass
                
                self.joint_states_for_3d = {
                    "timestamp": time.time(),
                    "left": left_joints,
                    "right": right_joints,
                    "names": {
                        "left": [f"left_joint{i+1}" for i in range(7)],
                        "right": [f"right_joint{i+1}" for i in range(7)]
                    }
                }
                
                # 打印更新后的数据（每秒一次，避免过多日志）
                if not hasattr(self, '_last_joint_log_time'):
                    self._last_joint_log_time = 0
                if time.time() - self._last_joint_log_time > 1.0:
                    print(f"🔄 更新3D关节状态 - Left[0]={left_joints[0]:.3f}, Right[0]={right_joints[0]:.3f}")
                    self._last_joint_log_time = time.time()
                
                # 同时更新 arm 的 joint_positions
                self.joint_positions = left_joints + right_joints
                
                # 旧的队列更新 (兼容)
                state_data = {
                    'joints': {
                        'left_arm': left_joints,
                        'right_arm': right_joints
                    },
                    'timestamp': time.time()
                }
                
                if not self.state_queue.full():
                    self.state_queue.put(state_data)
            
            self.node.create_subscription(
                JointState,
                '/joint_states',
                joint_state_callback,
                10
            )
            print("✅ joint_states 订阅器创建成功")
        except Exception as e:
            print(f"⚠️  joint_states 订阅器创建失败: {e}")

        # 升降机状态订阅
        try:
            from qyh_lift_msgs.msg import LiftState
            
            def lift_state_callback(msg):
                print(f"📥 收到升降机状态: pos={msg.current_position}")
                self.lift_state = {
                    "connected": msg.connected,
                    "enabled": msg.enabled,
                    "current_position": msg.current_position,
                    "current_speed": msg.current_speed,
                    "position_reached": msg.position_reached,
                    "alarm": msg.alarm,
                    "shutdown_requested": msg.shutdown_requested
                }
                
                # 检测硬件关机请求
                if msg.shutdown_requested:
                    print("⚠️ 检测到硬件关机请求！")
                    self.shutdown_requested = True
            
            self.node.create_subscription(
                LiftState,
                '/lift/state',
                lift_state_callback,
                10
            )
            print("✅ 升降机订阅器创建成功: /lift/state")
        except Exception as e:
            print(f"⚠️  升降机订阅器创建失败: {e}")
        # 关机状态订阅
        try:
            from qyh_shutdown_msgs.msg import ShutdownState
            
            def shutdown_state_callback(msg):
                trigger_text = ""
                if msg.trigger_source == 1:
                    trigger_text = "硬件按钮触发"
                elif msg.trigger_source == 2:
                    trigger_text = "软件命令触发"
                
                self._shutdown_state = {
                    "shutdown_in_progress": msg.shutdown_in_progress,
                    "trigger_source": msg.trigger_source,
                    "trigger_source_text": trigger_text,
                    "countdown_seconds": msg.countdown_seconds,
                    "plc_connected": msg.plc_connected
                }
                
                if msg.shutdown_in_progress:
                    print(f"⚠️ 系统正在关机！来源={trigger_text}, 倒计时={msg.countdown_seconds}秒")
            
            self.node.create_subscription(
                ShutdownState,
                'shutdown_state',
                shutdown_state_callback,
                10
            )
            print("✅ 关机状态订阅器创建成功: /shutdown_state")
        except Exception as e:
            print(f"⚠️  关机状态订阅器创建失败: {e}")


        # 头部状态订阅
        try:
            from sensor_msgs.msg import JointState as HeadJointState
            
            def head_state_callback(msg):
                if len(msg.position) >= 2:
                    # pan 和 tilt 弧度转换为归一化值
                    pan_rad = msg.position[0]
                    tilt_rad = msg.position[1]
                    # 弧度范围约 -1.57 到 1.57，归一化到 -1 到 1
                    pan_norm = pan_rad / 1.5708
                    tilt_norm = tilt_rad / 1.5708
                    # 归一化转舵机位置 (100-900 / 200-800)
                    pan_pos = 500 + pan_norm * 400
                    tilt_pos = 500 + tilt_norm * 300
                    
                    self.head_state = {
                        "connected": True,
                        "pan_position": pan_pos,
                        "tilt_position": tilt_pos,
                        "pan_normalized": pan_norm,
                        "tilt_normalized": tilt_norm
                    }
            
            self.node.create_subscription(
                HeadJointState,
                '/head/joint_states',
                head_state_callback,
                10
            )
            print("✅ 头部订阅器创建成功: /head/joint_states")
        except Exception as e:
            print(f"⚠️  头部订阅器创建失败: {e}")

        # 机械臂状态订阅
        try:
            from qyh_jaka_control_msgs.msg import RobotState
            
            def robot_state_callback(msg):
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
                    "right_in_position": msg.right_in_position
                }
            
            self.node.create_subscription(
                RobotState,
                '/jaka/robot_state',
                robot_state_callback,
                10
            )
            print("✅ 机械臂状态订阅器创建成功: /jaka/robot_state")
        except Exception as e:
            print(f"⚠️  机械臂状态订阅器创建失败: {e}")

        # 伺服状态订阅
        try:
            from qyh_jaka_control_msgs.msg import JakaServoStatus
            
            def servo_status_callback(msg):
                self.servo_status = {
                    "mode": msg.mode,
                    "is_abs": msg.is_abs,
                    "cycle_time_ns": msg.cycle_time_ns,
                    "publish_rate_hz": msg.publish_rate_hz,
                    "latency_ms": msg.latency_ms,
                    "packet_loss_rate": msg.packet_loss_rate,
                    "error_code": msg.error_code
                }
            
            self.node.create_subscription(
                JakaServoStatus,
                '/jaka/servo/status',
                servo_status_callback,
                10
            )
            print("✅ 伺服状态订阅器创建成功: /jaka/servo/status")
        except Exception as e:
            print(f"⚠️  伺服状态订阅器创建失败: {e}")

        # 关节状态订阅 (用于更新关节位置)
        try:
            from sensor_msgs.msg import JointState
            
            def jaka_joint_state_callback(msg):
                if len(msg.position) >= 14:
                    self.joint_positions = list(msg.position[:14])
                    # 更新 arm_state 中的关节位置
                    if self.arm_state:
                        self.arm_state['left_joint_positions'] = list(
                            msg.position[:7]
                        )
                        self.arm_state['right_joint_positions'] = list(
                            msg.position[7:14]
                        )
            
            self.node.create_subscription(
                JointState,
                '/joint_states',
                jaka_joint_state_callback,
                10
            )
            print("✅ JAKA关节状态订阅器创建成功: /joint_states")
        except Exception as e:
            print(f"⚠️  JAKA关节状态订阅器创建失败: {e}")

        # ==================== 底盘状态订阅 ====================
        self._setup_chassis_subscribers()
        
        # ==================== 夹爪状态订阅 ====================
        self._setup_gripper_subscribers()
        
        # ==================== 腰部状态订阅 ====================
        self._setup_waist_subscribers()
        
        # ==================== VR遥操作状态订阅 ====================
        self._setup_vr_subscribers()
    
    def _setup_vr_subscribers(self):
        """设置VR遥操作相关订阅器"""
        # 左手Clutch状态订阅
        try:
            from std_msgs.msg import Bool
            
            def left_clutch_callback(msg: Bool):
                self.vr_state['left_clutch_engaged'] = msg.data
                self.vr_state['connected'] = True
            
            self.node.create_subscription(
                Bool,
                '/vr/left_clutch_engaged',
                left_clutch_callback,
                10
            )
            print("✅ 左手Clutch状态订阅器创建成功: /vr/left_clutch_engaged")
        except Exception as e:
            print(f"⚠️  左手Clutch状态订阅器创建失败: {e}")

        # 右手Clutch状态订阅
        try:
            from std_msgs.msg import Bool
            
            def right_clutch_callback(msg: Bool):
                self.vr_state['right_clutch_engaged'] = msg.data
                self.vr_state['connected'] = True
            
            self.node.create_subscription(
                Bool,
                '/vr/right_clutch_engaged',
                right_clutch_callback,
                10
            )
            print("✅ 右手Clutch状态订阅器创建成功: /vr/right_clutch_engaged")
        except Exception as e:
            print(f"⚠️  右手Clutch状态订阅器创建失败: {e}")

        # 左手VR活跃状态订阅
        try:
            from std_msgs.msg import Bool
            
            def left_active_callback(msg: Bool):
                self.vr_state['left_hand_active'] = msg.data
                self.vr_state['connected'] = True  # 收到数据说明VR已连接
            
            self.node.create_subscription(
                Bool,
                '/vr/left_hand/active',
                left_active_callback,
                10
            )
            print("✅ 左手活跃状态订阅器创建成功: /vr/left_hand/active")
        except Exception as e:
            print(f"⚠️  左手活跃状态订阅器创建失败: {e}")

        # 右手VR活跃状态订阅
        try:
            from std_msgs.msg import Bool
            
            def right_active_callback(msg: Bool):
                self.vr_state['right_hand_active'] = msg.data
                self.vr_state['connected'] = True  # 收到数据说明VR已连接
            
            self.node.create_subscription(
                Bool,
                '/vr/right_hand/active',
                right_active_callback,
                10
            )
            print("✅ 右手活跃状态订阅器创建成功: /vr/right_hand/active")
        except Exception as e:
            print(f"⚠️  右手活跃状态订阅器创建失败: {e}")

        # 左手Grip值订阅（从Joy消息中提取）
        try:
            from sensor_msgs.msg import Joy
            
            def left_joy_callback(msg: Joy):
                if len(msg.axes) >= 4:
                    self.vr_state['left_grip_value'] = msg.axes[3]
                    self.vr_state['connected'] = True
            
            self.node.create_subscription(
                Joy,
                '/vr/left_hand/joy',
                left_joy_callback,
                10
            )
            print("✅ 左手Joy订阅器创建成功: /vr/left_hand/joy")
        except Exception as e:
            print(f"⚠️  左手Joy订阅器创建失败: {e}")

        # 右手Grip值订阅（从Joy消息中提取）
        try:
            from sensor_msgs.msg import Joy
            
            def right_joy_callback(msg: Joy):
                if len(msg.axes) >= 4:
                    self.vr_state['right_grip_value'] = msg.axes[3]
                    self.vr_state['connected'] = True
            
            self.node.create_subscription(
                Joy,
                '/vr/right_hand/joy',
                right_joy_callback,
                10
            )
            print("✅ 右手Joy订阅器创建成功: /vr/right_hand/joy")
        except Exception as e:
            print(f"⚠️  右手Joy订阅器创建失败: {e}")

    def _setup_gripper_subscribers(self):
        """设置夹爪相关订阅器"""
        # 左夹爪状态订阅
        try:
            from qyh_gripper_msgs.msg import GripperState
            
            def left_gripper_state_callback(msg: GripperState):
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
            
            self.node.create_subscription(
                GripperState,
                '/left/gripper_state',
                left_gripper_state_callback,
                10
            )
            print("✅ 左夹爪订阅器创建成功: /left/gripper_state")
        except Exception as e:
            print(f"⚠️  左夹爪订阅器创建失败: {e}")

        # 右夹爪状态订阅
        try:
            from qyh_gripper_msgs.msg import GripperState
            
            def right_gripper_state_callback(msg: GripperState):
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
            
            self.node.create_subscription(
                GripperState,
                '/right/gripper_state',
                right_gripper_state_callback,
                10
            )
            print("✅ 右夹爪订阅器创建成功: /right/gripper_state")
        except Exception as e:
            print(f"⚠️  右夹爪订阅器创建失败: {e}")

    def _setup_waist_subscribers(self):
        """设置腰部相关订阅器"""
        # 腰部状态订阅
        try:
            from qyh_waist_msgs.msg import WaistState
            
            def waist_state_callback(msg: WaistState):
                self.waist_state = {
                    "connected": msg.connected,
                    "enabled": msg.enabled,
                    "current_position": msg.current_position,
                    "current_angle": msg.current_angle,
                    "current_speed": msg.current_speed,
                    "position_reached": msg.position_reached,
                    "alarm": msg.alarm
                }
            
            self.node.create_subscription(
                WaistState,
                '/waist/state',
                waist_state_callback,
                10
            )
            print("✅ 腰部状态订阅器创建成功: /waist/state")
        except Exception as e:
            print(f"⚠️  腰部状态订阅器创建失败: {e}")

    def _setup_chassis_subscribers(self):
        """设置底盘相关订阅器"""
        # StandardRobotStatus 订阅
        try:
            from qyh_standard_robot_msgs.msg import StandardRobotStatus
            import math
            
            def chassis_status_callback(msg: StandardRobotStatus):
                # 更新时间戳
                import time
                self.chassis_last_update_time = time.time()
                
                # 从四元数计算 yaw
                q = msg.pose.pose.pose.orientation
                yaw = math.atan2(
                    2.0 * (q.w * q.z + q.x * q.y),
                    1.0 - 2.0 * (q.y * q.y + q.z * q.z)
                )
                
                self.chassis_status = {
                    "connected": True,
                    "system_status": msg.system_status,
                    "system_status_text": self._get_system_status_text(msg.system_status),
                    "location_status": msg.location_status,
                    "location_status_text": self._get_location_status_text(msg.location_status),
                    "operation_status": msg.operation_status,
                    "operation_status_text": self._get_operation_status_text(msg.operation_status),
                    "scheduling_mode": msg.scheduling_mode,
                    "scheduling_mode_text": self._get_scheduling_mode_text(msg.scheduling_mode),
                    "motion_status": msg.motion_status,
                    "motion_status_text": self._get_motion_status_text(msg.motion_status),
                    "current_station_id": msg.current_station_id,
                    "last_error_code": msg.last_error_code,
                    "current_system_volume": msg.current_system_volume,
                    "ip_address": f"{msg.ip_addresses[0]}.{msg.ip_addresses[1]}.{msg.ip_addresses[2]}.{msg.ip_addresses[3]}",
                    "current_map_name": msg.current_map_name,
                    
                    # 位姿信息
                    "pose": {
                        "x": msg.pose.pose.pose.position.x,
                        "y": msg.pose.pose.pose.position.y,
                        "yaw": yaw,
                        "confidence": msg.pose_confidence
                    },
                    
                    # 速度信息
                    "velocity": {
                        "linear_x": msg.twist.linear.x,
                        "linear_y": msg.twist.linear.y,
                        "angular_z": msg.twist.angular.z
                    },
                    
                    # 电池信息
                    "battery": {
                        "percentage": msg.battery_remaining_percentage,
                        "voltage": msg.battery_voltage,
                        "current": msg.battery_current,
                        "temperature": msg.battery_temperature,
                        "estimated_time": msg.battery_estimated_using_time,
                        "status": msg.battery_status,
                        "status_text": self._get_battery_status_text(msg.battery_status),
                        "cycle_count": msg.battery_cycle_count,
                        "nominal_capacity": msg.battery_nominal_capacity
                    },
                    
                    # 统计信息
                    "statistics": {
                        "total_distance": msg.total_motion_distance,
                        "total_boot_time": msg.total_boot_time,
                        "total_boot_count": msg.total_boot_count
                    },
                    
                    # 状态标志
                    "flags": {
                        "is_emergency_stopped": msg.is_emergency_stopped,
                        "is_emergency_recoverable": msg.is_emergency_recoverable,
                        "is_brake_released": msg.is_brake_released,
                        "is_charging": msg.is_charging,
                        "is_low_power_mode": msg.is_low_power_mode,
                        "obstacle_slowdown": msg.obstacle_slowdown,
                        "obstacle_paused": msg.obstacle_paused,
                        "can_run_motion_task": msg.can_run_motion_task,
                        "is_auto_mode": msg.is_auto_mode,
                        "is_loaded": msg.is_loaded,
                        "has_wifi": msg.has_wifi
                    },
                    
                    # 避障传感器触发
                    "obstacle_sensors": {
                        "main_radar": msg.obstacle_avoidance_triggered_main_radar,
                        "aux_radar": msg.obstacle_avoidance_triggered_aux_radar,
                        "depth_camera1": msg.obstacle_avoidance_triggered_depth_camera1,
                        "depth_camera2": msg.obstacle_avoidance_triggered_depth_camera2,
                        "depth_camera3": msg.obstacle_avoidance_triggered_depth_camera3,
                        "depth_camera4": msg.obstacle_avoidance_triggered_depth_camera4,
                        "obstacle_radar1": msg.obstacle_avoidance_triggered_obstacle_radar1,
                        "obstacle_radar2": msg.obstacle_avoidance_triggered_obstacle_radar2,
                        "obstacle_radar3": msg.obstacle_avoidance_triggered_obstacle_radar3,
                        "obstacle_radar4": msg.obstacle_avoidance_triggered_obstacle_radar4
                    },
                    
                    # 任务信息
                    "mission": {
                        "id": msg.current_mission_id,
                        "status": msg.mission_status,
                        "status_text": self._get_mission_status_text(msg.mission_status),
                        "result": msg.mission_result,
                        "result_text": self._get_mission_result_text(msg.mission_result),
                        "error_code": msg.mission_error_code
                    },
                    
                    # 移动任务信息
                    "move_task": {
                        "no": msg.current_move_task_no,
                        "status": msg.move_task_status,
                        "status_text": self._get_move_task_status_text(msg.move_task_status),
                        "result": msg.move_task_result,
                        "result_text": self._get_move_task_result_text(msg.move_task_result),
                        "start_station": msg.current_move_task_start_station,
                        "dest_station": msg.current_move_task_destination_station,
                        "path_no": msg.current_move_task_path_no
                    }
                }
            
            self.node.create_subscription(
                StandardRobotStatus,
                'standard_robot_status',
                chassis_status_callback,
                10
            )
            print("✅ 底盘状态订阅器创建成功: standard_robot_status")
        except Exception as e:
            print(f"⚠️  底盘状态订阅器创建失败: {e}")

        # NavigationStatus 订阅
        try:
            from qyh_standard_robot_msgs.msg import NavigationStatus
            
            def nav_status_callback(msg: NavigationStatus):
                self.navigation_status = {
                    "communication_pose": {
                        "x": msg.communication_pose.x,
                        "y": msg.communication_pose.y,
                        "yaw": msg.communication_pose.yaw
                    },
                    "autonomous_nav_pose": {
                        "x": msg.autonomous_nav_pose.x,
                        "y": msg.autonomous_nav_pose.y,
                        "yaw": msg.autonomous_nav_pose.yaw
                    },
                    "speed_level": msg.speed_level,
                    "obstacle_strategy": msg.obstacle_strategy,
                    "current_site": msg.current_site,
                    "speaker_volume": msg.speaker_volume
                }
            
            self.node.create_subscription(
                NavigationStatus,
                'navigation_status',
                nav_status_callback,
                10
            )
            print("✅ 导航状态订阅器创建成功: navigation_status")
        except Exception as e:
            print(f"⚠️  导航状态订阅器创建失败: {e}")

    # ==================== 底盘状态文本转换函数 ====================
    def _get_system_status_text(self, status: int) -> str:
        status_map = {
            0x01: '系统正在初始化', 0x02: '系统空闲', 0x03: '系统出错',
            0x04: '正在启动定位', 0x05: '导航正在初始化', 0x06: '导航正在寻路',
            0x07: '正在等待到达目标位置', 0x08: '检测到障碍，减速',
            0x09: '导航正在重新寻路', 0x0A: '遇到障碍暂停运动',
            0x0B: '无法抵达目标位置', 0x0E: '正在初始化执行固定路径',
            0x0F: '正在等待固定路径执行结束', 0x10: '执行固定路径检测到障碍',
            0x11: '执行固定路径遇到障碍暂停', 0x12: '无法检测到目标站点',
            0x13: '用户暂停固定路径', 0x15: '导航过程中出错', 0x16: '硬件错误'
        }
        return status_map.get(status, f'未知 (0x{status:02X})')

    def _get_location_status_text(self, status: int) -> str:
        status_map = {
            0x01: '定位未启动', 0x02: '定位正在初始化',
            0x03: '定位成功', 0x04: '正在重定位', 0x05: '定位错误'
        }
        return status_map.get(status, f'未知 (0x{status:02X})')

    def _get_operation_status_text(self, status: int) -> str:
        status_map = {0x00: '状态不可用', 0x01: '自动控制模式', 0x02: '手动控制模式'}
        return status_map.get(status, f'未知 (0x{status:02X})')

    def _get_scheduling_mode_text(self, mode: int) -> str:
        mode_map = {0x01: '手动模式', 0x02: '自动模式', 0x03: '维保模式'}
        return mode_map.get(mode, f'未知 (0x{mode:02X})')

    def _get_motion_status_text(self, status: int) -> str:
        status_map = {
            0x00: '未知状态', 0x01: '静止', 0x02: '前进', 0x03: '后退',
            0x04: '左转', 0x05: '右转', 0x06: '原地左转',
            0x07: '原地右转', 0x08: '左移', 0x09: '右移'
        }
        return status_map.get(status, f'未知 (0x{status:02X})')

    def _get_battery_status_text(self, status: int) -> str:
        status_map = {0x00: '无效状态', 0x02: '正在充电', 0x03: '未充电'}
        return status_map.get(status, f'未知 (0x{status:02X})')

    def _get_mission_status_text(self, status: int) -> str:
        status_map = {
            0x00: '无效状态', 0x02: '在队列中', 0x03: '正在执行',
            0x04: '暂停执行', 0x05: '执行结束', 0x06: '正在取消'
        }
        return status_map.get(status, f'未知 (0x{status:02X})')

    def _get_mission_result_text(self, result: int) -> str:
        result_map = {
            0x00: '无效状态', 0x01: '任务执行成功',
            0x02: '任务取消', 0x03: '任务执行出错'
        }
        return result_map.get(result, f'未知 (0x{result:02X})')

    def _get_move_task_status_text(self, status: int) -> str:
        status_map = {
            0x00: '无效状态', 0x02: '等待开始执行', 0x03: '正在执行',
            0x04: '暂停执行', 0x05: '执行结束', 0x06: '正在取消',
            0x08: '交通管制'
        }
        return status_map.get(status, f'未知 (0x{status:02X})')

    def _get_move_task_result_text(self, result: int) -> str:
        result_map = {
            0x00: '无效状态', 0x01: '任务执行成功',
            0x02: '任务取消', 0x03: '任务执行出错'
        }
        return result_map.get(result, f'未知 (0x{result:02X})')
    
    def _setup_publishers(self):
        """设置 ROS2 发布器"""
        try:
            from std_msgs.msg import Float64MultiArray
            
            self.arm_command_pub = self.node.create_publisher(
                Float64MultiArray,
                '/arm/joint_command',
                10
            )
        except Exception as e:
            print(f"⚠️  发布器创建失败: {e}")

        # 升降机控制客户端
        try:
            from qyh_lift_msgs.srv import LiftControl
            self.lift_client = self.node.create_client(
                LiftControl, '/lift/control'
            )
        except Exception as e:
            print(f"⚠️  升降机客户端创建失败: {e}")
        # 关机控制客户端
        try:
            from std_srvs.srv import Trigger
            self._shutdown_client = self.node.create_client(
                Trigger, 'qyh_shutdown'
            )
            print("✅ 关机客户端创建成功: qyh_shutdown")
        except Exception as e:
            print(f"⚠️  关机客户端创建失败: {e}")


        # 头部控制发布器
        try:
            from std_msgs.msg import Float64MultiArray
            self.head_cmd_pub = self.node.create_publisher(
                Float64MultiArray, '/head_motor_node/cmd_position', 10
            )
            print("✅ 头部发布器创建成功")
        except Exception as e:
            print(f"⚠️  头部发布器创建失败: {e}")

        # 机械臂服务客户端
        self._setup_arm_clients()
        
        # 任务引擎服务客户端
        self._setup_task_clients()
        
        # 底盘控制发布器和服务客户端
        self._setup_chassis_publishers()
        
        # 夹爪服务客户端
        self._setup_gripper_clients()
        
        # 腰部服务客户端
        self._setup_waist_clients()
    
    def _setup_waist_clients(self):
        """设置腰部服务客户端"""
        try:
            from qyh_waist_msgs.srv import WaistControl
            
            self.waist_control_client = self.node.create_client(
                WaistControl, '/waist/control'
            )
            print("✅ 腰部服务客户端创建成功: /waist/control")
        except Exception as e:
            print(f"⚠️  腰部服务客户端创建失败: {e}")
    
    def _setup_gripper_clients(self):
        """设置夹爪服务客户端"""
        try:
            from qyh_gripper_msgs.srv import ActivateGripper, MoveGripper
            
            # 左夹爪服务客户端
            self.left_gripper_activate_client = self.node.create_client(
                ActivateGripper, '/left/activate_gripper'
            )
            self.left_gripper_move_client = self.node.create_client(
                MoveGripper, '/left/move_gripper'
            )
            
            # 右夹爪服务客户端
            self.right_gripper_activate_client = self.node.create_client(
                ActivateGripper, '/right/activate_gripper'
            )
            self.right_gripper_move_client = self.node.create_client(
                MoveGripper, '/right/move_gripper'
            )
            
            print("✅ 夹爪服务客户端创建成功")
        except Exception as e:
            print(f"⚠️  夹爪服务客户端创建失败: {e}")

    def _setup_arm_clients(self):
        """设置机械臂服务客户端"""
        try:
            from std_srvs.srv import Trigger
            from qyh_jaka_control_msgs.srv import (
                StartServo, StopServo, MoveJ, MoveL
            )
            
            # 基础控制服务
            self.arm_power_on_client = self.node.create_client(
                Trigger, '/jaka/robot/power_on'
            )
            self.arm_power_off_client = self.node.create_client(
                Trigger, '/jaka/robot/power_off'
            )
            self.arm_enable_client = self.node.create_client(
                Trigger, '/jaka/robot/enable'
            )
            self.arm_disable_client = self.node.create_client(
                Trigger, '/jaka/robot/disable'
            )
            self.arm_clear_error_client = self.node.create_client(
                Trigger, '/jaka/robot/clear_error'
            )
            self.arm_motion_abort_client = self.node.create_client(
                Trigger, '/jaka/robot/motion_abort'
            )
            
            # 伺服控制服务
            self.arm_start_servo_client = self.node.create_client(
                StartServo, '/jaka/servo/start'
            )
            self.arm_stop_servo_client = self.node.create_client(
                StopServo, '/jaka/servo/stop'
            )
            
            # 运动控制服务
            self.arm_move_j_client = self.node.create_client(
                MoveJ, '/jaka/move_j'
            )
            self.arm_move_l_client = self.node.create_client(
                MoveL, '/jaka/move_l'
            )
            
            # 点动控制服务 (Jog)
            from qyh_jaka_control_msgs.srv import Jog, JogStop
            self.arm_jog_client = self.node.create_client(
                Jog, '/jaka/jog'
            )
            self.arm_jog_stop_client = self.node.create_client(
                JogStop, '/jaka/jog_stop'
            )
            
            # 负载管理服务 (Payload)
            from qyh_jaka_control_msgs.srv import SetPayload, GetPayload
            self.arm_set_payload_client = self.node.create_client(
                SetPayload, '/jaka/set_payload'
            )
            self.arm_get_payload_client = self.node.create_client(
                GetPayload, '/jaka/get_payload'
            )
            
            print("✅ 机械臂服务客户端创建成功")
        except Exception as e:
            print(f"⚠️  机械臂服务客户端创建失败: {e}")

    def _setup_task_clients(self):
        """设置任务引擎服务客户端"""
        try:
            from qyh_task_engine_msgs.srv import (
                ExecuteTask, PauseTask, ResumeTask, CancelTask, GetTaskStatus
            )
            from qyh_task_engine_msgs.msg import TaskStatus
            
            # 任务控制服务客户端
            self.task_execute_client = self.node.create_client(
                ExecuteTask, '/task_engine/execute'
            )
            self.task_pause_client = self.node.create_client(
                PauseTask, '/task_engine/pause'
            )
            self.task_resume_client = self.node.create_client(
                ResumeTask, '/task_engine/resume'
            )
            self.task_cancel_client = self.node.create_client(
                CancelTask, '/task_engine/cancel'
            )
            self.task_status_client = self.node.create_client(
                GetTaskStatus, '/task_engine/get_status'
            )
            
            # 任务状态订阅
            def task_status_callback(msg: TaskStatus):
                # 解析节点状态列表
                node_statuses = []
                for ns in msg.node_statuses:
                    node_statuses.append({
                        "node_id": ns.node_id,
                        "node_type": ns.node_type,
                        "node_name": ns.node_name,
                        "status": ns.status,
                        "message": ns.message,
                        "duration": ns.duration,
                        # 扩展字段
                        "children_count": ns.children_count,
                        "current_child_index": ns.current_child_index,
                        "current_iteration": ns.current_iteration,
                        "total_iterations": ns.total_iterations
                    })
                
                self.task_status = {
                    "task_id": msg.task_id,
                    "task_name": msg.task_name,
                    "status": msg.status,
                    "progress": msg.progress,
                    "current_node_id": msg.current_node_id,
                    "completed_nodes": msg.completed_nodes,
                    "total_nodes": msg.total_nodes,
                    "message": msg.message,
                    "elapsed_time": msg.elapsed_time,
                    "node_statuses": node_statuses
                }
            
            self.node.create_subscription(
                TaskStatus,
                '/task_engine/status',
                task_status_callback,
                10
            )
            
            print("✅ 任务引擎服务客户端创建成功")
        except Exception as e:
            print(f"⚠️  任务引擎服务客户端创建失败 (可能未安装 qyh_task_engine_msgs): {e}")

    def _setup_chassis_publishers(self):
        """设置底盘控制发布器和服务客户端"""
        # 速度命令发布器 (geometry_msgs/Twist - 已废弃，底盘不订阅)
        # 保留以备其他用途
        try:
            from geometry_msgs.msg import Twist
            self.chassis_cmd_vel_pub = self.node.create_publisher(
                Twist, '/cmd_vel', 10
            )
            print("✅ 底盘速度发布器创建成功: /cmd_vel")
        except Exception as e:
            print(f"⚠️  底盘速度发布器创建失败: {e}")

        # 手动控制命令发布器 (线圈模式)
        try:
            from qyh_standard_robot_msgs.msg import ManualMotionCommand
            self.chassis_manual_pub = self.node.create_publisher(
                ManualMotionCommand, 'manual_motion_cmd', 10
            )
            print("✅ 底盘手动控制发布器创建成功: manual_motion_cmd")
        except Exception as e:
            print(f"⚠️  底盘手动控制发布器创建失败: {e}")

        # 手动速度命令发布器 (速度模式 - 写入寄存器 40022-40024)
        try:
            from qyh_standard_robot_msgs.msg import ManualVelocityCommand
            self.chassis_velocity_pub = self.node.create_publisher(
                ManualVelocityCommand, 'manual_velocity_cmd', 10
            )
            print("✅ 底盘速度命令发布器创建成功: manual_velocity_cmd")
        except Exception as e:
            print(f"⚠️  底盘速度命令发布器创建失败: {e}")

        # 控制服务客户端
        try:
            from qyh_standard_robot_msgs.srv import (
                ControlStartManualControl,
                ControlStopManualControl,
                ControlPauseMove,
                ControlResumeMove,
                ControlStopMove,
                ControlStopLocalization,
                ControlEmergencyStop,
                ControlReleaseEmergencyStop,
                ControlStartCharging,
                ControlStopCharging,
                ControlEnterLowPowerMode,
                ControlExitLowPowerMode,
                ControlSystemReset,
                ControlPauseMission,
                ControlResumeMission,
                ControlCancelMission,
            )
            
            self.chassis_clients = {
                'start_manual': self.node.create_client(
                    ControlStartManualControl, 'control_start_manual_control'),
                'stop_manual': self.node.create_client(
                    ControlStopManualControl, 'control_stop_manual_control'),
                'pause_move': self.node.create_client(
                    ControlPauseMove, 'control_pause_move'),
                'resume_move': self.node.create_client(
                    ControlResumeMove, 'control_resume_move'),
                'stop_move': self.node.create_client(
                    ControlStopMove, 'control_stop_move'),
                'stop_localization': self.node.create_client(
                    ControlStopLocalization, 'control_stop_localization'),
                'emergency_stop': self.node.create_client(
                    ControlEmergencyStop, 'control_emergency_stop'),
                'release_emergency_stop': self.node.create_client(
                    ControlReleaseEmergencyStop, 'control_release_emergency_stop'),
                'start_charging': self.node.create_client(
                    ControlStartCharging, 'control_start_charging'),
                'stop_charging': self.node.create_client(
                    ControlStopCharging, 'control_stop_charging'),
                'enter_low_power': self.node.create_client(
                    ControlEnterLowPowerMode, 'control_enter_low_power_mode'),
                'exit_low_power': self.node.create_client(
                    ControlExitLowPowerMode, 'control_exit_low_power_mode'),
                'system_reset': self.node.create_client(
                    ControlSystemReset, 'control_system_reset'),
                'pause_mission': self.node.create_client(
                    ControlPauseMission, 'control_pause_mission'),
                'resume_mission': self.node.create_client(
                    ControlResumeMission, 'control_resume_mission'),
                'cancel_mission': self.node.create_client(
                    ControlCancelMission, 'control_cancel_mission'),
            }
            print("✅ 底盘控制服务客户端创建成功")
        except Exception as e:
            print(f"⚠️  底盘控制服务客户端创建失败: {e}")

        # Go 系列服务客户端
        try:
            from qyh_standard_robot_msgs.srv import (
                GoNavigateToCoordinate,
                GoExecuteActionTask,
                GoNavigateToSite,
                GoNavigateToSiteWithTask,
                GoForceLocalize,
                GoSetSpeedType,
                GoSetObstacleStrategy,
                GoSetCurrentSite,
                GoSetSpeakerVolume,
                GoSetCurrentMap,
            )
            
            self.chassis_go_nav_coord_client = self.node.create_client(
                GoNavigateToCoordinate, 'go_navigate_to_coordinate')
            self.chassis_go_nav_site_client = self.node.create_client(
                GoExecuteActionTask, 'go_navigate_to_site')
            self.chassis_go_nav_site_simple_client = self.node.create_client(
                GoNavigateToSite, 'go_navigate_to_site_simple')
            self.chassis_go_nav_site_task_client = self.node.create_client(
                GoNavigateToSiteWithTask, 'go_navigate_to_site_with_task')
            self.chassis_go_force_loc_client = self.node.create_client(
                GoForceLocalize, 'go_force_localize')
            self.chassis_go_set_speed_client = self.node.create_client(
                GoSetSpeedType, 'go_set_speed_level')
            self.chassis_go_set_obstacle_client = self.node.create_client(
                GoSetObstacleStrategy, 'go_set_obstacle_strategy')
            self.chassis_go_set_site_client = self.node.create_client(
                GoSetCurrentSite, 'go_set_current_site')
            self.chassis_go_set_volume_client = self.node.create_client(
                GoSetSpeakerVolume, 'go_set_speaker_volume')
            self.chassis_go_set_map_client = self.node.create_client(
                GoSetCurrentMap, 'go_set_current_map')
            print("✅ 底盘 Go 系列服务客户端创建成功")
            
            # 延迟初始化底盘参数（等待服务可用后）
            self.node.create_timer(3.0, self._initialize_chassis_params, one_shot=True)
            
        except Exception as e:
            print(f"⚠️  底盘 Go 系列服务客户端创建失败: {e}")
    
    def _initialize_chassis_params(self):
        """初始化底盘参数（从持久化配置加载）"""
        try:
            import json
            from pathlib import Path
            
            # 读取配置文件
            workspace_root = Path(__file__).parent.parent.parent.parent.parent
            config_file = workspace_root / "persistent" / "web" / "chassis_config.json"
            
            if not config_file.exists():
                print("ℹ️  底盘配置文件不存在，跳过参数初始化")
                return
            
            with open(config_file, 'r', encoding='utf-8') as f:
                config = json.load(f)
            
            speed_level = config.get('speed_level', 50)
            volume = config.get('volume', 50)
            
            print(f"🔧 初始化底盘参数: 速度级别={speed_level}, 音量={volume}")
            
            # 设置速度级别
            if self.chassis_go_set_speed_client and self.chassis_go_set_speed_client.wait_for_service(timeout_sec=1.0):
                from qyh_standard_robot_msgs.srv import GoSetSpeedType
                req = GoSetSpeedType.Request()
                req.speed_level = speed_level
                future = self.chassis_go_set_speed_client.call_async(req)
                
                def speed_callback(f):
                    try:
                        resp = f.result()
                        if resp.success:
                            print(f"✅ 速度级别初始化成功: {speed_level}")
                        else:
                            print(f"⚠️  速度级别初始化失败: {resp.message}")
                    except Exception as e:
                        print(f"❌ 速度级别初始化异常: {e}")
                
                future.add_done_callback(speed_callback)
            
            # 设置音量
            if self.chassis_go_set_volume_client and self.chassis_go_set_volume_client.wait_for_service(timeout_sec=1.0):
                from qyh_standard_robot_msgs.srv import GoSetSpeakerVolume
                req = GoSetSpeakerVolume.Request()
                req.volume = volume
                future = self.chassis_go_set_volume_client.call_async(req)
                
                def volume_callback(f):
                    try:
                        resp = f.result()
                        if resp.success:
                            print(f"✅ 音量初始化成功: {volume}")
                        else:
                            print(f"⚠️  音量初始化失败: {resp.message}")
                    except Exception as e:
                        print(f"❌ 音量初始化异常: {e}")
                
                future.add_done_callback(volume_callback)
                
        except Exception as e:
            print(f"❌ 底盘参数初始化失败: {e}")
    
    def _process_commands(self):
        """处理命令队列"""
        while not self.command_queue.empty():
            cmd = self.command_queue.get()
            self._execute_command(cmd)
    
    def _execute_command(self, cmd: Dict[str, Any]):
        """执行具体命令"""
        cmd_type = cmd.get('type')
        
        try:
            if cmd_type == 'move_joints':
                from std_msgs.msg import Float64MultiArray
                msg = Float64MultiArray()
                msg.data = cmd['params']['positions']
                self.arm_command_pub.publish(msg)
            
            elif cmd_type == 'emergency_stop':
                # 发送急停指令到所有关节
                print("⚠️  发送急停指令")
                # TODO: 实现急停逻辑

            elif cmd_type == 'lift_control':
                from qyh_lift_msgs.srv import LiftControl
                req = LiftControl.Request()
                req.command = cmd['params']['command']
                req.value = float(cmd['params']['value'])
                req.hold = cmd['params']['hold']
                
                if not self.lift_client.wait_for_service(timeout_sec=1.0):
                    raise Exception("Lift service not available")
                
                future = self.lift_client.call_async(req)
                
                def done_callback(f):
                    try:
                        resp = f.result()
                        result = {
                            'success': resp.success,
                            'message': resp.message
                        }
                    except Exception as e:
                        result = {'success': False, 'message': str(e)}
                    
                    if 'loop' in cmd and 'future' in cmd:
                        cmd['loop'].call_soon_threadsafe(
                            cmd['future'].set_result, result
                        )
                
                future.add_done_callback(done_callback)


            elif cmd_type == 'shutdown':
                from std_srvs.srv import Trigger
                req = Trigger.Request()
                
                if not self._shutdown_client.wait_for_service(timeout_sec=1.0):
                    raise Exception("Shutdown service not available")
                
                future = self._shutdown_client.call_async(req)
                
                def done_callback(f):
                    try:
                        resp = f.result()
                        result = {
                            'success': resp.success,
                            'message': resp.message
                        }
                    except Exception as e:
                        result = {'success': False, 'message': str(e)}
                    
                    if 'loop' in cmd and 'future' in cmd:
                        cmd['loop'].call_soon_threadsafe(
                            cmd['future'].set_result, result
                        )
                
                future.add_done_callback(done_callback)

            elif cmd_type == 'head_control':
                from std_msgs.msg import Float64MultiArray
                pan = cmd['params'].get('pan')
                tilt = cmd['params'].get('tilt')
                
                # 头部需要同时发送pan和tilt
                if (pan is not None or tilt is not None) and self.head_cmd_pub:
                    # 如果只提供了一个值,使用当前状态填充另一个
                    current_state = self.get_head_state()
                    if pan is None:
                        pan = current_state.get('pan_normalized', 0.0) if current_state else 0.0
                    if tilt is None:
                        tilt = current_state.get('tilt_normalized', 0.0) if current_state else 0.0
                    
                    msg = Float64MultiArray()
                    msg.data = [float(tilt), float(pan)]  # 注意：节点期望顺序是 [tilt, pan]
                    self.head_cmd_pub.publish(msg)
                
                if 'loop' in cmd and 'future' in cmd:
                    result = {'success': True, 'message': '头部控制已发送'}
                    cmd['loop'].call_soon_threadsafe(
                        cmd['future'].set_result, result
                    )

            elif cmd_type == 'arm_trigger_service':
                self._handle_arm_trigger_service(cmd)

            elif cmd_type == 'arm_move_j':
                self._handle_arm_move_j(cmd)

            elif cmd_type == 'arm_move_l':
                self._handle_arm_move_l(cmd)

            elif cmd_type == 'arm_jog':
                self._handle_arm_jog(cmd)

            elif cmd_type == 'arm_jog_stop':
                self._handle_arm_jog_stop(cmd)

            elif cmd_type == 'arm_set_payload':
                self._handle_arm_set_payload(cmd)

            elif cmd_type == 'arm_get_payload':
                self._handle_arm_get_payload(cmd)

            # 任务引擎命令
            elif cmd_type == 'task_execute':
                self._handle_task_execute(cmd)

            elif cmd_type == 'task_pause':
                self._handle_task_pause(cmd)

            elif cmd_type == 'task_resume':
                self._handle_task_resume(cmd)

            elif cmd_type == 'task_cancel':
                self._handle_task_cancel(cmd)

            elif cmd_type == 'task_get_status':
                self._handle_task_get_status(cmd)

            # ==================== 底盘控制命令 ====================
            elif cmd_type == 'chassis_velocity':
                self._handle_chassis_velocity(cmd)

            elif cmd_type == 'chassis_manual_motion':
                self._handle_chassis_manual_motion(cmd)

            elif cmd_type == 'chassis_control_service':
                self._handle_chassis_control_service(cmd)

            elif cmd_type == 'chassis_nav_coordinate':
                self._handle_chassis_nav_coordinate(cmd)

            elif cmd_type == 'chassis_nav_site':
                self._handle_chassis_nav_site(cmd)
            
            elif cmd_type == 'chassis_nav_site_simple':
                self._handle_chassis_nav_site_simple(cmd)
            
            elif cmd_type == 'chassis_nav_site_task':
                self._handle_chassis_nav_site_task(cmd)

            elif cmd_type == 'chassis_force_localize':
                self._handle_chassis_force_localize(cmd)

            elif cmd_type == 'chassis_set_speed_level':
                self._handle_chassis_set_speed_level(cmd)

            elif cmd_type == 'chassis_set_obstacle_strategy':
                self._handle_chassis_set_obstacle_strategy(cmd)

            elif cmd_type == 'chassis_set_current_site':
                self._handle_chassis_set_current_site(cmd)

            elif cmd_type == 'chassis_set_volume':
                self._handle_chassis_set_volume(cmd)

            elif cmd_type == 'chassis_set_map':
                self._handle_chassis_set_map(cmd)
            
            # ==================== 夹爪命令 ====================
            elif cmd_type == 'gripper_activate':
                self._handle_gripper_activate(cmd)

            elif cmd_type == 'gripper_move':
                self._handle_gripper_move(cmd)
            
            # ==================== 腰部命令 ====================
            elif cmd_type == 'waist_control':
                self._handle_waist_control(cmd)
        
        except Exception as e:
            print(f"❌ 命令执行失败: {cmd_type}, 错误: {e}")
            if 'loop' in cmd and 'future' in cmd:
                result = {'success': False, 'message': str(e)}
                cmd['loop'].call_soon_threadsafe(
                    cmd['future'].set_result, result
                )

    def _handle_arm_trigger_service(self, cmd: Dict[str, Any]):
        """处理机械臂 Trigger 类型服务调用"""
        service_name = cmd['params']['service_name']
        
        # 获取对应的客户端
        client_map = {
            'power_on': self.arm_power_on_client,
            'power_off': self.arm_power_off_client,
            'enable': self.arm_enable_client,
            'disable': self.arm_disable_client,
            'clear_error': self.arm_clear_error_client,
            'motion_abort': self.arm_motion_abort_client,
        }
        
        client = client_map.get(service_name)
        if not client:
            # 伺服服务需要特殊处理
            if service_name == 'start_servo':
                self._handle_start_servo(cmd)
                return
            elif service_name == 'stop_servo':
                self._handle_stop_servo(cmd)
                return
            
            result = {'success': False, 'message': f'未知服务: {service_name}'}
            if 'loop' in cmd and 'future' in cmd:
                cmd['loop'].call_soon_threadsafe(
                    cmd['future'].set_result, result
                )
            return
        
        if not client.wait_for_service(timeout_sec=1.0):
            result = {'success': False, 'message': f'服务不可用: {service_name}'}
            if 'loop' in cmd and 'future' in cmd:
                cmd['loop'].call_soon_threadsafe(
                    cmd['future'].set_result, result
                )
            return
        
        from std_srvs.srv import Trigger
        req = Trigger.Request()
        future = client.call_async(req)
        
        def done_callback(f):
            try:
                resp = f.result()
                result = {
                    'success': resp.success,
                    'message': resp.message
                }
            except Exception as e:
                result = {'success': False, 'message': str(e)}
            
            if 'loop' in cmd and 'future' in cmd:
                cmd['loop'].call_soon_threadsafe(
                    cmd['future'].set_result, result
                )
        
        future.add_done_callback(done_callback)

    def _handle_start_servo(self, cmd: Dict[str, Any]):
        """处理启动伺服服务"""
        if not self.arm_start_servo_client:
            result = {'success': False, 'message': '伺服启动客户端未初始化'}
            if 'loop' in cmd and 'future' in cmd:
                cmd['loop'].call_soon_threadsafe(
                    cmd['future'].set_result, result
                )
            return
        
        if not self.arm_start_servo_client.wait_for_service(timeout_sec=1.0):
            result = {'success': False, 'message': '伺服启动服务不可用'}
            if 'loop' in cmd and 'future' in cmd:
                cmd['loop'].call_soon_threadsafe(
                    cmd['future'].set_result, result
                )
            return
        
        from qyh_jaka_control_msgs.srv import StartServo
        req = StartServo.Request()
        future = self.arm_start_servo_client.call_async(req)
        
        def done_callback(f):
            try:
                resp = f.result()
                result = {
                    'success': resp.success,
                    'message': resp.message
                }
            except Exception as e:
                result = {'success': False, 'message': str(e)}
            
            if 'loop' in cmd and 'future' in cmd:
                cmd['loop'].call_soon_threadsafe(
                    cmd['future'].set_result, result
                )
        
        future.add_done_callback(done_callback)

    def _handle_stop_servo(self, cmd: Dict[str, Any]):
        """处理停止伺服服务"""
        if not self.arm_stop_servo_client:
            result = {'success': False, 'message': '伺服停止客户端未初始化'}
            if 'loop' in cmd and 'future' in cmd:
                cmd['loop'].call_soon_threadsafe(
                    cmd['future'].set_result, result
                )
            return
        
        if not self.arm_stop_servo_client.wait_for_service(timeout_sec=1.0):
            result = {'success': False, 'message': '伺服停止服务不可用'}
            if 'loop' in cmd and 'future' in cmd:
                cmd['loop'].call_soon_threadsafe(
                    cmd['future'].set_result, result
                )
            return
        
        from qyh_jaka_control_msgs.srv import StopServo
        req = StopServo.Request()
        future = self.arm_stop_servo_client.call_async(req)
        
        def done_callback(f):
            try:
                resp = f.result()
                result = {
                    'success': resp.success,
                    'message': resp.message
                }
            except Exception as e:
                result = {'success': False, 'message': str(e)}
            
            if 'loop' in cmd and 'future' in cmd:
                cmd['loop'].call_soon_threadsafe(
                    cmd['future'].set_result, result
                )
        
        future.add_done_callback(done_callback)

    def _handle_arm_move_j(self, cmd: Dict[str, Any]):
        """处理 MoveJ 服务"""
        if not self.arm_move_j_client:
            result = {'success': False, 'message': 'MoveJ 客户端未初始化'}
            if 'loop' in cmd and 'future' in cmd:
                cmd['loop'].call_soon_threadsafe(
                    cmd['future'].set_result, result
                )
            return
        
        if not self.arm_move_j_client.wait_for_service(timeout_sec=1.0):
            result = {'success': False, 'message': 'MoveJ 服务不可用'}
            if 'loop' in cmd and 'future' in cmd:
                cmd['loop'].call_soon_threadsafe(
                    cmd['future'].set_result, result
                )
            return
        
        from qyh_jaka_control_msgs.srv import MoveJ
        params = cmd['params']
        req = MoveJ.Request()
        req.robot_id = params['robot_id']
        req.joint_positions = params['joint_positions']
        req.velocity = params['velocity']
        req.acceleration = params['acceleration']
        req.is_block = params['is_block']
        
        future = self.arm_move_j_client.call_async(req)
        
        def done_callback(f):
            try:
                resp = f.result()
                result = {
                    'success': resp.success,
                    'message': resp.message
                }
            except Exception as e:
                result = {'success': False, 'message': str(e)}
            
            if 'loop' in cmd and 'future' in cmd:
                cmd['loop'].call_soon_threadsafe(
                    cmd['future'].set_result, result
                )
        
        future.add_done_callback(done_callback)

    def _handle_arm_move_l(self, cmd: Dict[str, Any]):
        """处理 MoveL 服务"""
        if not self.arm_move_l_client:
            result = {'success': False, 'message': 'MoveL 客户端未初始化'}
            if 'loop' in cmd and 'future' in cmd:
                cmd['loop'].call_soon_threadsafe(
                    cmd['future'].set_result, result
                )
            return
        
        if not self.arm_move_l_client.wait_for_service(timeout_sec=1.0):
            result = {'success': False, 'message': 'MoveL 服务不可用'}
            if 'loop' in cmd and 'future' in cmd:
                cmd['loop'].call_soon_threadsafe(
                    cmd['future'].set_result, result
                )
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
        # 将 rx, ry, rz 转换为四元数 (简化处理)
        # 实际应该使用 tf2 或其他库进行转换
        # 这里假设 rx, ry, rz 用作姿态表示
        req.target_pose.orientation.x = params['rx']
        req.target_pose.orientation.y = params['ry']
        req.target_pose.orientation.z = params['rz']
        req.target_pose.orientation.w = 1.0
        req.velocity = params['velocity']
        req.acceleration = params['acceleration']
        req.is_block = params['is_block']
        
        future = self.arm_move_l_client.call_async(req)
        
        def done_callback(f):
            try:
                resp = f.result()
                result = {
                    'success': resp.success,
                    'message': resp.message
                }
            except Exception as e:
                result = {'success': False, 'message': str(e)}
            
            if 'loop' in cmd and 'future' in cmd:
                cmd['loop'].call_soon_threadsafe(
                    cmd['future'].set_result, result
                )
        
        future.add_done_callback(done_callback)

    def _handle_arm_jog(self, cmd: Dict[str, Any]):
        """处理 Jog 服务"""
        if not self.arm_jog_client:
            result = {'success': False, 'message': 'Jog 客户端未初始化'}
            if 'loop' in cmd and 'future' in cmd:
                cmd['loop'].call_soon_threadsafe(
                    cmd['future'].set_result, result
                )
            return
        
        if not self.arm_jog_client.wait_for_service(timeout_sec=1.0):
            result = {'success': False, 'message': 'Jog 服务不可用'}
            if 'loop' in cmd and 'future' in cmd:
                cmd['loop'].call_soon_threadsafe(
                    cmd['future'].set_result, result
                )
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
        
        future = self.arm_jog_client.call_async(req)
        
        def done_callback(f):
            try:
                resp = f.result()
                result = {
                    'success': resp.success,
                    'message': resp.message
                }
            except Exception as e:
                result = {'success': False, 'message': str(e)}
            
            if 'loop' in cmd and 'future' in cmd:
                cmd['loop'].call_soon_threadsafe(
                    cmd['future'].set_result, result
                )
        
        future.add_done_callback(done_callback)

    def _handle_arm_jog_stop(self, cmd: Dict[str, Any]):
        """处理 JogStop 服务"""
        if not self.arm_jog_stop_client:
            result = {'success': False, 'message': 'JogStop 客户端未初始化'}
            if 'loop' in cmd and 'future' in cmd:
                cmd['loop'].call_soon_threadsafe(
                    cmd['future'].set_result, result
                )
            return
        
        if not self.arm_jog_stop_client.wait_for_service(timeout_sec=1.0):
            result = {'success': False, 'message': 'JogStop 服务不可用'}
            if 'loop' in cmd and 'future' in cmd:
                cmd['loop'].call_soon_threadsafe(
                    cmd['future'].set_result, result
                )
            return
        
        from qyh_jaka_control_msgs.srv import JogStop
        params = cmd['params']
        
        req = JogStop.Request()
        req.robot_id = params['robot_id']
        req.axis_num = params['axis_num']
        
        future = self.arm_jog_stop_client.call_async(req)
        
        def done_callback(f):
            try:
                resp = f.result()
                result = {
                    'success': resp.success,
                    'message': resp.message
                }
            except Exception as e:
                result = {'success': False, 'message': str(e)}
            
            if 'loop' in cmd and 'future' in cmd:
                cmd['loop'].call_soon_threadsafe(
                    cmd['future'].set_result, result
                )
        
        future.add_done_callback(done_callback)

    def _handle_arm_set_payload(self, cmd: Dict[str, Any]):
        """处理 SetPayload 服务"""
        if not self.arm_set_payload_client:
            result = {'success': False, 'message': 'SetPayload 客户端未初始化'}
            if 'loop' in cmd and 'future' in cmd:
                cmd['loop'].call_soon_threadsafe(
                    cmd['future'].set_result, result
                )
            return
        
        if not self.arm_set_payload_client.wait_for_service(timeout_sec=1.0):
            result = {'success': False, 'message': 'SetPayload 服务不可用'}
            if 'loop' in cmd and 'future' in cmd:
                cmd['loop'].call_soon_threadsafe(
                    cmd['future'].set_result, result
                )
            return
        
        from qyh_jaka_control_msgs.srv import SetPayload
        params = cmd['params']
        
        req = SetPayload.Request()
        req.robot_id = params['robot_id']
        req.mass = params['mass']
        
        future = self.arm_set_payload_client.call_async(req)
        
        def done_callback(f):
            try:
                resp = f.result()
                result = {
                    'success': resp.success,
                    'message': resp.message
                }
            except Exception as e:
                result = {'success': False, 'message': str(e)}
            
            if 'loop' in cmd and 'future' in cmd:
                cmd['loop'].call_soon_threadsafe(
                    cmd['future'].set_result, result
                )
        
        future.add_done_callback(done_callback)

    def _handle_arm_get_payload(self, cmd: Dict[str, Any]):
        """处理 GetPayload 服务"""
        if not self.arm_get_payload_client:
            result = {'success': False, 'message': 'GetPayload 客户端未初始化'}
            if 'loop' in cmd and 'future' in cmd:
                cmd['loop'].call_soon_threadsafe(
                    cmd['future'].set_result, result
                )
            return
        
        if not self.arm_get_payload_client.wait_for_service(timeout_sec=1.0):
            result = {'success': False, 'message': 'GetPayload 服务不可用'}
            if 'loop' in cmd and 'future' in cmd:
                cmd['loop'].call_soon_threadsafe(
                    cmd['future'].set_result, result
                )
            return
        
        from qyh_jaka_control_msgs.srv import GetPayload
        params = cmd['params']
        
        req = GetPayload.Request()
        req.robot_id = params['robot_id']
        
        future = self.arm_get_payload_client.call_async(req)
        
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
            
            if 'loop' in cmd and 'future' in cmd:
                cmd['loop'].call_soon_threadsafe(
                    cmd['future'].set_result, result
                )
        
        future.add_done_callback(done_callback)

    def _handle_task_execute(self, cmd: Dict[str, Any]):
        """处理任务执行请求"""
        if not self.task_execute_client:
            result = {'success': False, 'message': '任务引擎客户端未初始化'}
            if 'loop' in cmd and 'future' in cmd:
                cmd['loop'].call_soon_threadsafe(
                    cmd['future'].set_result, result
                )
            return
        
        if not self.task_execute_client.wait_for_service(timeout_sec=2.0):
            result = {'success': False, 'message': '任务引擎服务不可用'}
            if 'loop' in cmd and 'future' in cmd:
                cmd['loop'].call_soon_threadsafe(
                    cmd['future'].set_result, result
                )
            return
        
        from qyh_task_engine_msgs.srv import ExecuteTask
        params = cmd['params']
        req = ExecuteTask.Request()
        req.task_json = params['task_json']
        
        future = self.task_execute_client.call_async(req)
        
        def done_callback(f):
            try:
                resp = f.result()
                result = {
                    'success': resp.success,
                    'task_id': resp.task_id,
                    'message': resp.message
                }
            except Exception as e:
                result = {'success': False, 'task_id': '', 'message': str(e)}
            
            if 'loop' in cmd and 'future' in cmd:
                cmd['loop'].call_soon_threadsafe(
                    cmd['future'].set_result, result
                )
        
        future.add_done_callback(done_callback)

    def _handle_task_pause(self, cmd: Dict[str, Any]):
        """处理任务暂停请求"""
        if not self.task_pause_client:
            result = {'success': False, 'message': '任务引擎客户端未初始化'}
            if 'loop' in cmd and 'future' in cmd:
                cmd['loop'].call_soon_threadsafe(
                    cmd['future'].set_result, result
                )
            return
        
        if not self.task_pause_client.wait_for_service(timeout_sec=1.0):
            result = {'success': False, 'message': '任务暂停服务不可用'}
            if 'loop' in cmd and 'future' in cmd:
                cmd['loop'].call_soon_threadsafe(
                    cmd['future'].set_result, result
                )
            return
        
        from qyh_task_engine_msgs.srv import PauseTask
        params = cmd['params']
        req = PauseTask.Request()
        req.task_id = params['task_id']
        
        future = self.task_pause_client.call_async(req)
        
        def done_callback(f):
            try:
                resp = f.result()
                result = {
                    'success': resp.success,
                    'message': resp.message
                }
            except Exception as e:
                result = {'success': False, 'message': str(e)}
            
            if 'loop' in cmd and 'future' in cmd:
                cmd['loop'].call_soon_threadsafe(
                    cmd['future'].set_result, result
                )
        
        future.add_done_callback(done_callback)

    def _handle_task_resume(self, cmd: Dict[str, Any]):
        """处理任务恢复请求"""
        if not self.task_resume_client:
            result = {'success': False, 'message': '任务引擎客户端未初始化'}
            if 'loop' in cmd and 'future' in cmd:
                cmd['loop'].call_soon_threadsafe(
                    cmd['future'].set_result, result
                )
            return
        
        if not self.task_resume_client.wait_for_service(timeout_sec=1.0):
            result = {'success': False, 'message': '任务恢复服务不可用'}
            if 'loop' in cmd and 'future' in cmd:
                cmd['loop'].call_soon_threadsafe(
                    cmd['future'].set_result, result
                )
            return
        
        from qyh_task_engine_msgs.srv import ResumeTask
        params = cmd['params']
        req = ResumeTask.Request()
        req.task_id = params['task_id']
        
        future = self.task_resume_client.call_async(req)
        
        def done_callback(f):
            try:
                resp = f.result()
                result = {
                    'success': resp.success,
                    'message': resp.message
                }
            except Exception as e:
                result = {'success': False, 'message': str(e)}
            
            if 'loop' in cmd and 'future' in cmd:
                cmd['loop'].call_soon_threadsafe(
                    cmd['future'].set_result, result
                )
        
        future.add_done_callback(done_callback)

    def _handle_task_cancel(self, cmd: Dict[str, Any]):
        """处理任务取消请求"""
        if not self.task_cancel_client:
            result = {'success': False, 'message': '任务引擎客户端未初始化'}
            if 'loop' in cmd and 'future' in cmd:
                cmd['loop'].call_soon_threadsafe(
                    cmd['future'].set_result, result
                )
            return
        
        if not self.task_cancel_client.wait_for_service(timeout_sec=1.0):
            result = {'success': False, 'message': '任务取消服务不可用'}
            if 'loop' in cmd and 'future' in cmd:
                cmd['loop'].call_soon_threadsafe(
                    cmd['future'].set_result, result
                )
            return
        
        from qyh_task_engine_msgs.srv import CancelTask
        params = cmd['params']
        req = CancelTask.Request()
        req.task_id = params['task_id']
        
        future = self.task_cancel_client.call_async(req)
        
        def done_callback(f):
            try:
                resp = f.result()
                result = {
                    'success': resp.success,
                    'message': resp.message
                }
            except Exception as e:
                result = {'success': False, 'message': str(e)}
            
            if 'loop' in cmd and 'future' in cmd:
                cmd['loop'].call_soon_threadsafe(
                    cmd['future'].set_result, result
                )
        
        future.add_done_callback(done_callback)

    def _handle_task_get_status(self, cmd: Dict[str, Any]):
        """处理任务状态查询请求"""
        if not self.task_status_client:
            result = {'success': False, 'message': '任务引擎客户端未初始化'}
            if 'loop' in cmd and 'future' in cmd:
                cmd['loop'].call_soon_threadsafe(
                    cmd['future'].set_result, result
                )
            return
        
        if not self.task_status_client.wait_for_service(timeout_sec=1.0):
            result = {'success': False, 'message': '任务状态服务不可用'}
            if 'loop' in cmd and 'future' in cmd:
                cmd['loop'].call_soon_threadsafe(
                    cmd['future'].set_result, result
                )
            return
        
        from qyh_task_engine_msgs.srv import GetTaskStatus
        params = cmd['params']
        req = GetTaskStatus.Request()
        req.task_id = params['task_id']
        
        future = self.task_status_client.call_async(req)
        
        def done_callback(f):
            try:
                resp = f.result()
                result = {
                    'success': resp.success,
                    'task_id': resp.status.task_id,
                    'status': resp.status.status,
                    'progress': resp.status.progress,
                    'current_node': resp.status.current_node,
                    'message': resp.status.message,
                    'elapsed_time': resp.status.elapsed_time
                }
            except Exception as e:
                result = {'success': False, 'message': str(e)}
            
            if 'loop' in cmd and 'future' in cmd:
                cmd['loop'].call_soon_threadsafe(
                    cmd['future'].set_result, result
                )
        
        future.add_done_callback(done_callback)

    # ==================== 底盘命令处理函数 ====================

    def _handle_chassis_velocity(self, cmd: Dict[str, Any]):
        """处理底盘速度命令 - 使用 ManualVelocityCommand 话题"""
        try:
            from qyh_standard_robot_msgs.msg import ManualVelocityCommand
            params = cmd['params']
            
            msg = ManualVelocityCommand()
            msg.vx = float(params['linear_x'])
            # 底盘无横向移动能力，忽略 linear_y
            msg.w = float(params['angular_z'])
            
            if self.chassis_velocity_pub:
                self.chassis_velocity_pub.publish(msg)
                result = {'success': True, 'message': '速度命令已发送'}
            else:
                result = {'success': False, 'message': '速度发布器未初始化'}
        except Exception as e:
            result = {'success': False, 'message': str(e)}
        
        if 'loop' in cmd and 'future' in cmd:
            cmd['loop'].call_soon_threadsafe(
                cmd['future'].set_result, result
            )

    def _handle_chassis_manual_motion(self, cmd: Dict[str, Any]):
        """处理底盘手动运动命令"""
        try:
            from qyh_standard_robot_msgs.msg import ManualMotionCommand
            params = cmd['params']
            
            msg = ManualMotionCommand()
            msg.forward = params.get('forward', False)
            msg.backward = params.get('backward', False)
            msg.rotate_left = params.get('rotate_left', False)
            msg.rotate_right = params.get('rotate_right', False)
            
            if self.chassis_manual_pub:
                self.chassis_manual_pub.publish(msg)
                result = {'success': True, 'message': '手动命令已发送'}
            else:
                result = {'success': False, 'message': '手动控制发布器未初始化'}
        except Exception as e:
            result = {'success': False, 'message': str(e)}
        
        if 'loop' in cmd and 'future' in cmd:
            cmd['loop'].call_soon_threadsafe(
                cmd['future'].set_result, result
            )

    def _handle_chassis_control_service(self, cmd: Dict[str, Any]):
        """处理底盘控制服务调用"""
        service_name = cmd['params']['service_name']
        client = self.chassis_clients.get(service_name)
        
        if not client:
            result = {'success': False, 'message': f'未知服务: {service_name}'}
            if 'loop' in cmd and 'future' in cmd:
                cmd['loop'].call_soon_threadsafe(
                    cmd['future'].set_result, result
                )
            return
        
        if not client.wait_for_service(timeout_sec=1.0):
            result = {'success': False, 'message': f'服务不可用: {service_name}'}
            if 'loop' in cmd and 'future' in cmd:
                cmd['loop'].call_soon_threadsafe(
                    cmd['future'].set_result, result
                )
            return
        
        # 创建空请求 (所有控制服务都是空请求)
        req = client.srv_type.Request()
        future = client.call_async(req)
        
        def done_callback(f):
            try:
                resp = f.result()
                result = {
                    'success': getattr(resp, 'success', True),
                    'message': getattr(resp, 'message', '操作成功')
                }
            except Exception as e:
                result = {'success': False, 'message': str(e)}
            
            if 'loop' in cmd and 'future' in cmd:
                cmd['loop'].call_soon_threadsafe(
                    cmd['future'].set_result, result
                )
        
        future.add_done_callback(done_callback)

    def _handle_chassis_nav_coordinate(self, cmd: Dict[str, Any]):
        """处理导航到坐标"""
        if not self.chassis_go_nav_coord_client:
            result = {'success': False, 'message': '导航服务客户端未初始化'}
            if 'loop' in cmd and 'future' in cmd:
                cmd['loop'].call_soon_threadsafe(
                    cmd['future'].set_result, result
                )
            return
        
        if not self.chassis_go_nav_coord_client.wait_for_service(timeout_sec=1.0):
            result = {'success': False, 'message': '导航服务不可用'}
            if 'loop' in cmd and 'future' in cmd:
                cmd['loop'].call_soon_threadsafe(
                    cmd['future'].set_result, result
                )
            return
        
        from qyh_standard_robot_msgs.srv import GoNavigateToCoordinate
        params = cmd['params']
        req = GoNavigateToCoordinate.Request()
        req.x = float(params['x'])
        req.y = float(params['y'])
        req.yaw = float(params['yaw'])
        req.is_localization = params.get('is_localization', False)
        
        future = self.chassis_go_nav_coord_client.call_async(req)
        
        def done_callback(f):
            try:
                resp = f.result()
                result = {
                    'success': getattr(resp, 'success', True),
                    'message': getattr(resp, 'message', '操作成功')
                }
            except Exception as e:
                result = {'success': False, 'message': str(e)}
            
            if 'loop' in cmd and 'future' in cmd:
                cmd['loop'].call_soon_threadsafe(
                    cmd['future'].set_result, result
                )
        
        future.add_done_callback(done_callback)

    def _handle_chassis_nav_site(self, cmd: Dict[str, Any]):
        """处理导航到站点"""
        if not self.chassis_go_nav_site_client:
            result = {'success': False, 'message': '站点导航服务客户端未初始化'}
            if 'loop' in cmd and 'future' in cmd:
                cmd['loop'].call_soon_threadsafe(
                    cmd['future'].set_result, result
                )
            return
        
        if not self.chassis_go_nav_site_client.wait_for_service(timeout_sec=1.0):
            result = {'success': False, 'message': '站点导航服务不可用'}
            if 'loop' in cmd and 'future' in cmd:
                cmd['loop'].call_soon_threadsafe(
                    cmd['future'].set_result, result
                )
            return
        
        from qyh_standard_robot_msgs.srv import GoExecuteActionTask
        params = cmd['params']
        req = GoExecuteActionTask.Request()
        req.site_id = int(params['site_id'])
        req.is_localization = params.get('is_localization', False)
        
        future = self.chassis_go_nav_site_client.call_async(req)
        
        def done_callback(f):
            try:
                resp = f.result()
                result = {
                    'success': getattr(resp, 'success', True),
                    'message': getattr(resp, 'message', '操作成功')
                }
            except Exception as e:
                result = {'success': False, 'message': str(e)}
            
            if 'loop' in cmd and 'future' in cmd:
                cmd['loop'].call_soon_threadsafe(
                    cmd['future'].set_result, result
                )
        
        future.add_done_callback(done_callback)
    
    def _handle_chassis_nav_site_simple(self, cmd: Dict[str, Any]):
        """处理导航到站点（简单版本）"""
        if not self.chassis_go_nav_site_simple_client:
            result = {'success': False, 'message': '站点导航服务客户端未初始化'}
            if 'loop' in cmd and 'future' in cmd:
                cmd['loop'].call_soon_threadsafe(
                    cmd['future'].set_result, result
                )
            return
        
        if not self.chassis_go_nav_site_simple_client.wait_for_service(timeout_sec=1.0):
            result = {'success': False, 'message': '站点导航服务不可用'}
            if 'loop' in cmd and 'future' in cmd:
                cmd['loop'].call_soon_threadsafe(
                    cmd['future'].set_result, result
                )
            return
        
        from qyh_standard_robot_msgs.srv import GoNavigateToSite
        params = cmd['params']
        req = GoNavigateToSite.Request()
        req.site_id = int(params['site_id'])
        
        future = self.chassis_go_nav_site_simple_client.call_async(req)
        
        def done_callback(f):
            try:
                resp = f.result()
                result = {
                    'success': getattr(resp, 'success', True),
                    'message': getattr(resp, 'message', '操作成功')
                }
            except Exception as e:
                result = {'success': False, 'message': str(e)}
            
            if 'loop' in cmd and 'future' in cmd:
                cmd['loop'].call_soon_threadsafe(
                    cmd['future'].set_result, result
                )
        
        future.add_done_callback(done_callback)
    
    def _handle_chassis_nav_site_task(self, cmd: Dict[str, Any]):
        """处理导航到站点（带任务ID）"""
        if not self.chassis_go_nav_site_task_client:
            result = {'success': False, 'message': '站点导航服务客户端未初始化'}
            if 'loop' in cmd and 'future' in cmd:
                cmd['loop'].call_soon_threadsafe(
                    cmd['future'].set_result, result
                )
            return
        
        if not self.chassis_go_nav_site_task_client.wait_for_service(timeout_sec=1.0):
            result = {'success': False, 'message': '站点导航服务不可用'}
            if 'loop' in cmd and 'future' in cmd:
                cmd['loop'].call_soon_threadsafe(
                    cmd['future'].set_result, result
                )
            return
        
        from qyh_standard_robot_msgs.srv import GoNavigateToSiteWithTask
        params = cmd['params']
        req = GoNavigateToSiteWithTask.Request()
        req.site_id = int(params['site_id'])
        req.task_id = int(params['task_id'])
        
        future = self.chassis_go_nav_site_task_client.call_async(req)
        
        def done_callback(f):
            try:
                resp = f.result()
                result = {
                    'success': getattr(resp, 'success', True),
                    'message': getattr(resp, 'message', '操作成功')
                }
            except Exception as e:
                result = {'success': False, 'message': str(e)}
            
            if 'loop' in cmd and 'future' in cmd:
                cmd['loop'].call_soon_threadsafe(
                    cmd['future'].set_result, result
                )
        
        future.add_done_callback(done_callback)

    def _handle_chassis_force_localize(self, cmd: Dict[str, Any]):
        """处理强制定位"""
        if not self.chassis_go_force_loc_client:
            result = {'success': False, 'message': '强制定位服务客户端未初始化'}
            if 'loop' in cmd and 'future' in cmd:
                cmd['loop'].call_soon_threadsafe(
                    cmd['future'].set_result, result
                )
            return
        
        if not self.chassis_go_force_loc_client.wait_for_service(timeout_sec=1.0):
            result = {'success': False, 'message': '强制定位服务不可用'}
            if 'loop' in cmd and 'future' in cmd:
                cmd['loop'].call_soon_threadsafe(
                    cmd['future'].set_result, result
                )
            return
        
        from qyh_standard_robot_msgs.srv import GoForceLocalize
        params = cmd['params']
        req = GoForceLocalize.Request()
        req.x = float(params['x'])
        req.y = float(params['y'])
        req.yaw = float(params['yaw'])
        
        future = self.chassis_go_force_loc_client.call_async(req)
        
        def done_callback(f):
            try:
                resp = f.result()
                result = {
                    'success': getattr(resp, 'success', True),
                    'message': getattr(resp, 'message', '操作成功')
                }
            except Exception as e:
                result = {'success': False, 'message': str(e)}
            
            if 'loop' in cmd and 'future' in cmd:
                cmd['loop'].call_soon_threadsafe(
                    cmd['future'].set_result, result
                )
        
        future.add_done_callback(done_callback)

    def _handle_chassis_set_speed_level(self, cmd: Dict[str, Any]):
        """处理设置速度级别"""
        if not self.chassis_go_set_speed_client:
            result = {'success': False, 'message': '速度设置服务客户端未初始化'}
            if 'loop' in cmd and 'future' in cmd:
                cmd['loop'].call_soon_threadsafe(
                    cmd['future'].set_result, result
                )
            return
        
        if not self.chassis_go_set_speed_client.wait_for_service(timeout_sec=1.0):
            result = {'success': False, 'message': '速度设置服务不可用'}
            if 'loop' in cmd and 'future' in cmd:
                cmd['loop'].call_soon_threadsafe(
                    cmd['future'].set_result, result
                )
            return
        
        from qyh_standard_robot_msgs.srv import GoSetSpeedType
        params = cmd['params']
        req = GoSetSpeedType.Request()
        req.speed_level = int(params['level'])
        
        future = self.chassis_go_set_speed_client.call_async(req)
        
        def done_callback(f):
            try:
                resp = f.result()
                result = {
                    'success': getattr(resp, 'success', True),
                    'message': getattr(resp, 'message', '操作成功')
                }
            except Exception as e:
                result = {'success': False, 'message': str(e)}
            
            if 'loop' in cmd and 'future' in cmd:
                cmd['loop'].call_soon_threadsafe(
                    cmd['future'].set_result, result
                )
        
        future.add_done_callback(done_callback)

    def _handle_chassis_set_obstacle_strategy(self, cmd: Dict[str, Any]):
        """处理设置避障策略"""
        if not self.chassis_go_set_obstacle_client:
            result = {'success': False, 'message': '避障策略服务客户端未初始化'}
            if 'loop' in cmd and 'future' in cmd:
                cmd['loop'].call_soon_threadsafe(
                    cmd['future'].set_result, result
                )
            return
        
        if not self.chassis_go_set_obstacle_client.wait_for_service(timeout_sec=1.0):
            result = {'success': False, 'message': '避障策略服务不可用'}
            if 'loop' in cmd and 'future' in cmd:
                cmd['loop'].call_soon_threadsafe(
                    cmd['future'].set_result, result
                )
            return
        
        from qyh_standard_robot_msgs.srv import GoSetObstacleStrategy
        params = cmd['params']
        req = GoSetObstacleStrategy.Request()
        req.strategy = int(params['strategy'])
        
        future = self.chassis_go_set_obstacle_client.call_async(req)
        
        def done_callback(f):
            try:
                resp = f.result()
                result = {
                    'success': getattr(resp, 'success', True),
                    'message': getattr(resp, 'message', '操作成功')
                }
            except Exception as e:
                result = {'success': False, 'message': str(e)}
            
            if 'loop' in cmd and 'future' in cmd:
                cmd['loop'].call_soon_threadsafe(
                    cmd['future'].set_result, result
                )
        
        future.add_done_callback(done_callback)

    def _handle_chassis_set_current_site(self, cmd: Dict[str, Any]):
        """处理设置当前站点"""
        if not self.chassis_go_set_site_client:
            result = {'success': False, 'message': '站点设置服务客户端未初始化'}
            if 'loop' in cmd and 'future' in cmd:
                cmd['loop'].call_soon_threadsafe(
                    cmd['future'].set_result, result
                )
            return
        
        if not self.chassis_go_set_site_client.wait_for_service(timeout_sec=1.0):
            result = {'success': False, 'message': '站点设置服务不可用'}
            if 'loop' in cmd and 'future' in cmd:
                cmd['loop'].call_soon_threadsafe(
                    cmd['future'].set_result, result
                )
            return
        
        from qyh_standard_robot_msgs.srv import GoSetCurrentSite
        params = cmd['params']
        req = GoSetCurrentSite.Request()
        req.site_id = int(params['site_id'])
        
        future = self.chassis_go_set_site_client.call_async(req)
        
        def done_callback(f):
            try:
                resp = f.result()
                result = {
                    'success': getattr(resp, 'success', True),
                    'message': getattr(resp, 'message', '操作成功')
                }
            except Exception as e:
                result = {'success': False, 'message': str(e)}
            
            if 'loop' in cmd and 'future' in cmd:
                cmd['loop'].call_soon_threadsafe(
                    cmd['future'].set_result, result
                )
        
        future.add_done_callback(done_callback)

    def _handle_chassis_set_volume(self, cmd: Dict[str, Any]):
        """处理设置音量"""
        if not self.chassis_go_set_volume_client:
            result = {'success': False, 'message': '音量设置服务客户端未初始化'}
            if 'loop' in cmd and 'future' in cmd:
                cmd['loop'].call_soon_threadsafe(
                    cmd['future'].set_result, result
                )
            return
        
        if not self.chassis_go_set_volume_client.wait_for_service(timeout_sec=1.0):
            result = {'success': False, 'message': '音量设置服务不可用'}
            if 'loop' in cmd and 'future' in cmd:
                cmd['loop'].call_soon_threadsafe(
                    cmd['future'].set_result, result
                )
            return
        
        from qyh_standard_robot_msgs.srv import GoSetSpeakerVolume
        params = cmd['params']
        req = GoSetSpeakerVolume.Request()
        req.volume = int(params['volume'])
        
        future = self.chassis_go_set_volume_client.call_async(req)
        
        def done_callback(f):
            try:
                resp = f.result()
                result = {
                    'success': getattr(resp, 'success', True),
                    'message': getattr(resp, 'message', '操作成功')
                }
            except Exception as e:
                result = {'success': False, 'message': str(e)}
            
            if 'loop' in cmd and 'future' in cmd:
                cmd['loop'].call_soon_threadsafe(
                    cmd['future'].set_result, result
                )
        
        future.add_done_callback(done_callback)

    def _handle_chassis_set_map(self, cmd: Dict[str, Any]):
        """处理设置当前地图"""
        if not self.chassis_go_set_map_client:
            result = {'success': False, 'message': '地图设置服务客户端未初始化'}
            if 'loop' in cmd and 'future' in cmd:
                cmd['loop'].call_soon_threadsafe(
                    cmd['future'].set_result, result
                )
            return
        
        if not self.chassis_go_set_map_client.wait_for_service(timeout_sec=1.0):
            result = {'success': False, 'message': '地图设置服务不可用'}
            if 'loop' in cmd and 'future' in cmd:
                cmd['loop'].call_soon_threadsafe(
                    cmd['future'].set_result, result
                )
            return
        
        from qyh_standard_robot_msgs.srv import GoSetCurrentMap
        params = cmd['params']
        req = GoSetCurrentMap.Request()
        req.map_name = str(params['map_name'])
        
        future = self.chassis_go_set_map_client.call_async(req)
        
        def done_callback(f):
            try:
                resp = f.result()
                result = {
                    'success': getattr(resp, 'success', True),
                    'message': getattr(resp, 'message', '操作成功')
                }
            except Exception as e:
                result = {'success': False, 'message': str(e)}
            
            if 'loop' in cmd and 'future' in cmd:
                cmd['loop'].call_soon_threadsafe(
                    cmd['future'].set_result, result
                )
        
        future.add_done_callback(done_callback)

    # ==================== 夹爪命令处理函数 ====================

    def _handle_gripper_activate(self, cmd: Dict[str, Any]):
        """处理夹爪激活命令"""
        side = cmd['params']['side']
        
        if side == 'left':
            client = self.left_gripper_activate_client
        else:
            client = self.right_gripper_activate_client
        
        if not client:
            result = {'success': False, 'message': '夹爪激活客户端未初始化'}
            if 'loop' in cmd and 'future' in cmd:
                cmd['loop'].call_soon_threadsafe(
                    cmd['future'].set_result, result
                )
            return
        
        if not client.wait_for_service(timeout_sec=1.0):
            result = {'success': False, 'message': '夹爪激活服务不可用'}
            if 'loop' in cmd and 'future' in cmd:
                cmd['loop'].call_soon_threadsafe(
                    cmd['future'].set_result, result
                )
            return
        
        from qyh_gripper_msgs.srv import ActivateGripper
        req = ActivateGripper.Request()
        future = client.call_async(req)
        
        def done_callback(f):
            try:
                resp = f.result()
                result = {
                    'success': resp.success,
                    'message': resp.message
                }
            except Exception as e:
                result = {'success': False, 'message': str(e)}
            
            if 'loop' in cmd and 'future' in cmd:
                cmd['loop'].call_soon_threadsafe(
                    cmd['future'].set_result, result
                )
        
        future.add_done_callback(done_callback)

    def _handle_gripper_move(self, cmd: Dict[str, Any]):
        """处理夹爪移动命令"""
        params = cmd['params']
        side = params['side']
        
        if side == 'left':
            client = self.left_gripper_move_client
        else:
            client = self.right_gripper_move_client
        
        if not client:
            result = {'success': False, 'message': '夹爪移动客户端未初始化'}
            if 'loop' in cmd and 'future' in cmd:
                cmd['loop'].call_soon_threadsafe(
                    cmd['future'].set_result, result
                )
            return
        
        if not client.wait_for_service(timeout_sec=1.0):
            result = {'success': False, 'message': '夹爪移动服务不可用'}
            if 'loop' in cmd and 'future' in cmd:
                cmd['loop'].call_soon_threadsafe(
                    cmd['future'].set_result, result
                )
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
                result = {
                    'success': resp.success,
                    'message': resp.message
                }
            except Exception as e:
                result = {'success': False, 'message': str(e)}
            
            if 'loop' in cmd and 'future' in cmd:
                cmd['loop'].call_soon_threadsafe(
                    cmd['future'].set_result, result
                )
        
        future.add_done_callback(done_callback)

    def _handle_waist_control(self, cmd: Dict[str, Any]):
        """处理腰部控制命令"""
        if not self.waist_control_client:
            result = {'success': False, 'message': '腰部控制客户端未初始化'}
            if 'loop' in cmd and 'future' in cmd:
                cmd['loop'].call_soon_threadsafe(
                    cmd['future'].set_result, result
                )
            return
        
        if not self.waist_control_client.wait_for_service(timeout_sec=1.0):
            result = {'success': False, 'message': '腰部控制服务不可用'}
            if 'loop' in cmd and 'future' in cmd:
                cmd['loop'].call_soon_threadsafe(
                    cmd['future'].set_result, result
                )
            return
        
        from qyh_waist_msgs.srv import WaistControl
        params = cmd['params']
        req = WaistControl.Request()
        req.command = params['command']
        req.value = float(params['value'])
        req.hold = params['hold']
        
        future = self.waist_control_client.call_async(req)
        
        def done_callback(f):
            try:
                resp = f.result()
                result = {
                    'success': resp.success,
                    'message': resp.message
                }
            except Exception as e:
                result = {'success': False, 'message': str(e)}
            
            if 'loop' in cmd and 'future' in cmd:
                cmd['loop'].call_soon_threadsafe(
                    cmd['future'].set_result, result
                )
        
        future.add_done_callback(done_callback)
    
    def get_robot_state(self) -> Optional[Dict[str, Any]]:
        """获取最新的机器人状态（异步安全）"""
        if self.mock_mode:
            # 返回模拟数据
            return {
                'joints': {
                    'left_arm': [0.1, 0.2, 0.3, 0.4, 0.5, 0.6],
                    'right_arm': [0.0, 0.1, 0.2, 0.3, 0.4, 0.5]
                },
                'timestamp': 'mock'
            }
        
        if self.state_queue.empty():
            return None
        return self.state_queue.get()

    
    def get_shutdown_state(self) -> Optional[Dict[str, Any]]:
        """获取关机状态"""
        return self._shutdown_state

    def get_lift_state(self) -> Optional[Dict[str, Any]]:
        """获取升降电机状态"""
        if self.mock_mode:
            return None  # Mock 模式由 API 层处理
        
        return self.lift_state

    def get_head_state(self) -> Optional[Dict[str, Any]]:
        """获取头部状态"""
        if self.mock_mode:
            return None  # Mock 模式由 API 层处理
        
        return self.head_state

    def get_waist_state(self) -> Optional[Dict[str, Any]]:
        """获取腰部状态"""
        if self.mock_mode:
            return None  # Mock 模式由 API 层处理
        
        return self.waist_state

    # ==================== 夹爪状态获取 ====================
    
    def get_left_gripper_state(self) -> Optional[Dict[str, Any]]:
        """获取左夹爪状态"""
        if self.mock_mode:
            return None  # Mock 模式由 API 层处理
        
        return self.left_gripper_state

    def get_right_gripper_state(self) -> Optional[Dict[str, Any]]:
        """获取右夹爪状态"""
        if self.mock_mode:
            return None  # Mock 模式由 API 层处理
        
        return self.right_gripper_state

    async def call_gripper_activate(self, side: str) -> Optional[Dict]:
        """调用夹爪激活服务"""
        if self.mock_mode:
            return None  # Mock 模式由 API 层处理
        
        import asyncio
        loop = asyncio.get_running_loop()
        future = loop.create_future()
        
        cmd = {
            'type': 'gripper_activate',
            'params': {'side': side},
            'future': future,
            'loop': loop
        }
        self.command_queue.put(cmd)
        return await future

    async def call_gripper_move(
        self,
        side: str,
        position: int,
        speed: int,
        force: int
    ) -> Optional[Dict]:
        """调用夹爪移动服务"""
        if self.mock_mode:
            return None  # Mock 模式由 API 层处理
        
        import asyncio
        loop = asyncio.get_running_loop()
        future = loop.create_future()
        
        cmd = {
            'type': 'gripper_move',
            'params': {
                'side': side,
                'position': position,
                'speed': speed,
                'force': force
            },
            'future': future,
            'loop': loop
        }
        self.command_queue.put(cmd)
        return await future

    def get_robot_description(self) -> Optional[str]:
        """获取机器人 URDF 描述"""
        if self.mock_mode:
            return None  # Mock 模式由 API 层处理
        
        return self.robot_description

    def get_joint_states(self) -> Optional[Dict[str, Any]]:
        """获取 3D 场景用的关节状态"""
        # 即使在 Mock 模式也返回缓存数据，避免上层 API 收到 None
        return self.joint_states_for_3d

    def get_arm_state(self) -> Optional[Dict[str, Any]]:
        """获取机械臂状态"""
        # 在 Mock 模式或真实模式下均返回缓存的状态（可能为空）以便上层使用回退逻辑
        state = {}
        if self.arm_state:
            state = self.arm_state.copy()
        else:
            # 提供一个基础的默认结构，避免上层抛出 KeyError
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
            # 保证字段存在
            state.setdefault('left_joint_positions', [0.0] * 7)
            state.setdefault('right_joint_positions', [0.0] * 7)
        
        return state

    def get_servo_status(self) -> Optional[Dict[str, Any]]:
        """获取伺服状态"""
        # 在 Mock 模式也返回一个默认伺服状态结构，避免上层收到 None
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

    async def call_arm_service(self, service_name: str) -> Optional[Dict]:
        """调用机械臂基础服务 (Trigger 类型)"""
        if self.mock_mode:
            return None  # Mock 模式由 API 层处理
        
        import asyncio
        loop = asyncio.get_running_loop()
        future = loop.create_future()
        
        cmd = {
            'type': 'arm_trigger_service',
            'params': {'service_name': service_name},
            'future': future,
            'loop': loop
        }
        self.command_queue.put(cmd)
        return await future

    async def call_move_j(
        self,
        robot_id: int,
        joint_positions: list,
        velocity: float,
        acceleration: float,
        is_block: bool
    ) -> Optional[Dict[str, Any]]:
        """调用 MoveJ 服务"""
        if self.mock_mode:
            return None  # Mock 模式由 API 层处理
        
        import asyncio
        loop = asyncio.get_running_loop()
        future = loop.create_future()
        
        cmd = {
            'type': 'arm_move_j',
            'params': {
                'robot_id': robot_id,
                'joint_positions': joint_positions,
                'velocity': velocity,
                'acceleration': acceleration,
                'is_block': is_block
            },
            'future': future,
            'loop': loop
        }
        self.command_queue.put(cmd)
        return await future

    async def call_move_l(
        self,
        robot_id: int,
        x: float, y: float, z: float,
        rx: float, ry: float, rz: float,
        velocity: float,
        acceleration: float,
        is_block: bool
    ) -> Optional[Dict[str, Any]]:
        """调用 MoveL 服务"""
        if self.mock_mode:
            return None  # Mock 模式由 API 层处理
        
        import asyncio
        loop = asyncio.get_running_loop()
        future = loop.create_future()
        
        cmd = {
            'type': 'arm_move_l',
            'params': {
                'robot_id': robot_id,
                'x': x, 'y': y, 'z': z,
                'rx': rx, 'ry': ry, 'rz': rz,
                'velocity': velocity,
                'acceleration': acceleration,
                'is_block': is_block
            },
            'future': future,
            'loop': loop
        }
        self.command_queue.put(cmd)
        return await future

    async def call_jog(
        self,
        robot_id: int,
        axis_num: int,
        move_mode: int,
        coord_type: int,
        velocity: float,
        position: float
    ) -> Optional[Dict[str, Any]]:
        """调用 Jog 服务进行点动控制"""
        if self.mock_mode:
            return None  # Mock 模式由 API 层处理
        
        import asyncio
        loop = asyncio.get_running_loop()
        future = loop.create_future()
        
        cmd = {
            'type': 'arm_jog',
            'params': {
                'robot_id': robot_id,
                'axis_num': axis_num,
                'move_mode': move_mode,
                'coord_type': coord_type,
                'velocity': velocity,
                'position': position
            },
            'future': future,
            'loop': loop
        }
        self.command_queue.put(cmd)
        return await future

    async def call_jog_stop(
        self,
        robot_id: int,
        axis_num: int = 0
    ) -> Optional[Dict[str, Any]]:
        """调用 JogStop 服务停止点动"""
        if self.mock_mode:
            return None  # Mock 模式由 API 层处理
        
        import asyncio
        loop = asyncio.get_running_loop()
        future = loop.create_future()
        
        cmd = {
            'type': 'arm_jog_stop',
            'params': {
                'robot_id': robot_id,
                'axis_num': axis_num
            },
            'future': future,
            'loop': loop
        }
        self.command_queue.put(cmd)
        return await future

    async def call_set_payload(
        self,
        robot_id: int,
        mass: float
    ) -> Optional[Dict[str, Any]]:
        """调用 SetPayload 服务设置负载"""
        if self.mock_mode:
            return None  # Mock 模式由 API 层处理
        
        import asyncio
        loop = asyncio.get_running_loop()
        future = loop.create_future()
        
        cmd = {
            'type': 'arm_set_payload',
            'params': {
                'robot_id': robot_id,
                'mass': mass
            },
            'future': future,
            'loop': loop
        }
        self.command_queue.put(cmd)
        return await future

    async def call_get_payload(
        self,
        robot_id: int
    ) -> Optional[Dict[str, Any]]:
        """调用 GetPayload 服务获取负载"""
        if self.mock_mode:
            return None  # Mock 模式由 API 层处理
        
        import asyncio
        loop = asyncio.get_running_loop()
        future = loop.create_future()
        
        cmd = {
            'type': 'arm_get_payload',
            'params': {
                'robot_id': robot_id
            },
            'future': future,
            'loop': loop
        }
        self.command_queue.put(cmd)
        return await future

    async def send_head_command(
        self,
        pan: Optional[float] = None,
        tilt: Optional[float] = None
    ) -> Optional[Dict[str, Any]]:
        """发送头部控制命令"""
        if self.mock_mode:
            return None  # Mock 模式由 API 层处理
        
        import asyncio
        loop = asyncio.get_running_loop()
        future = loop.create_future()
        
        cmd = {
            'type': 'head_control',
            'params': {
                'pan': pan,
                'tilt': tilt
            },
            'future': future,
            'loop': loop
        }
        self.command_queue.put(cmd)
        return await future

    async def call_lift_control(
        self,
        command: int,
        value: float = 0.0,
        hold: bool = False
    ) -> Optional[Dict[str, Any]]:
        """调用升降电机控制服务"""
        if self.mock_mode:
            return None  # Mock 模式由 API 层处理
        
        import asyncio
        loop = asyncio.get_running_loop()
        future = loop.create_future()
        
        cmd = {
            'type': 'lift_control',
            'params': {
                'command': command,
                'value': value,
                'hold': hold
            },
            'future': future,
            'loop': loop
        }
        self.command_queue.put(cmd)
        return await future

    async def call_waist_control(
        self,
        command: int,
        value: float = 0.0,
        hold: bool = False
    ) -> Optional[Dict[str, Any]]:
        """调用腰部电机控制服务"""
        if self.mock_mode:
            return None  # Mock 模式由 API 层处理
        
        import asyncio
        loop = asyncio.get_running_loop()
        future = loop.create_future()
        
        cmd = {
            'type': 'waist_control',
            'params': {
                'command': command,
                'value': value,
                'hold': hold
            },
            'future': future,
            'loop': loop
        }
        self.command_queue.put(cmd)
        return await future

    # ==================== 任务引擎接口 ====================

    async def execute_task(self, task_json: str) -> Optional[Dict[str, Any]]:
        """执行任务"""
        if self.mock_mode:
            return None  # Mock 模式由 API 层处理
        
        # 如果任务引擎客户端未初始化，回退到 Mock 模式
        if not hasattr(self, 'task_execute_client') or self.task_execute_client is None:
            return None
        
        import asyncio
        loop = asyncio.get_running_loop()
        future = loop.create_future()
        
        cmd = {
            'type': 'task_execute',
            'params': {'task_json': task_json},
            'future': future,
            'loop': loop
        }
        self.command_queue.put(cmd)
        return await future

    async def pause_task(self, task_id: str) -> Optional[Dict[str, Any]]:
        """暂停任务"""
        if self.mock_mode:
            return None  # Mock 模式由 API 层处理
        
        # 如果任务引擎客户端未初始化，回退到 Mock 模式
        if not hasattr(self, 'task_pause_client') or not self.task_pause_client:
            return None
        
        import asyncio
        loop = asyncio.get_running_loop()
        future = loop.create_future()
        
        cmd = {
            'type': 'task_pause',
            'params': {'task_id': task_id},
            'future': future,
            'loop': loop
        }
        self.command_queue.put(cmd)
        return await future

    async def resume_task(self, task_id: str) -> Optional[Dict[str, Any]]:
        """恢复任务"""
        if self.mock_mode:
            return None  # Mock 模式由 API 层处理
        
        # 如果任务引擎客户端未初始化，回退到 Mock 模式
        if not hasattr(self, 'task_resume_client') or not self.task_resume_client:
            return None
        
        import asyncio
        loop = asyncio.get_running_loop()
        future = loop.create_future()
        
        cmd = {
            'type': 'task_resume',
            'params': {'task_id': task_id},
            'future': future,
            'loop': loop
        }
        self.command_queue.put(cmd)
        return await future

    async def cancel_task(self, task_id: str) -> Optional[Dict[str, Any]]:
        """取消任务"""
        if self.mock_mode:
            return None  # Mock 模式由 API 层处理
        
        # 如果任务引擎客户端未初始化，回退到 Mock 模式
        if not hasattr(self, 'task_cancel_client') or not self.task_cancel_client:
            return None
        
        import asyncio
        loop = asyncio.get_running_loop()
        future = loop.create_future()
        
        cmd = {
            'type': 'task_cancel',
            'params': {'task_id': task_id},
            'future': future,
            'loop': loop
        }
        self.command_queue.put(cmd)
        return await future

    async def get_task_status(self, task_id: str) -> Optional[Dict[str, Any]]:
        """获取任务状态"""
        if self.mock_mode:
            return None  # Mock 模式由 API 层处理
        
        import asyncio
        loop = asyncio.get_running_loop()
        future = loop.create_future()
        
        cmd = {
            'type': 'task_get_status',
            'params': {'task_id': task_id},
            'future': future,
            'loop': loop
        }
        self.command_queue.put(cmd)
        return await future

    def get_cached_task_status(self) -> Optional[Dict[str, Any]]:
        """获取缓存的任务状态（从订阅器接收）"""
        if self.mock_mode:
            return None  # Mock 模式由 API 层处理
        
        return self.task_status

    # ==================== 底盘控制接口 ====================

    def get_chassis_status(self) -> Optional[Dict[str, Any]]:
        """获取底盘状态（带超时检测）"""
        if self.mock_mode:
            return None  # Mock 模式由 API 层处理
        
        if self.chassis_status is None:
            return None
        
        # 检查是否超时
        if self.chassis_last_update_time is not None:
            import time
            elapsed = time.time() - self.chassis_last_update_time
            if elapsed > self.chassis_timeout_seconds:
                # 超时，标记为断连
                status = self.chassis_status.copy()
                status["connected"] = False
                return status
        
        return self.chassis_status

    def get_navigation_status(self) -> Optional[Dict[str, Any]]:
        """获取导航状态"""
        if self.mock_mode:
            return None  # Mock 模式由 API 层处理
        return self.navigation_status

    async def send_chassis_velocity(
        self,
        linear_x: float,
        linear_y: float,
        angular_z: float
    ) -> Optional[Dict[str, Any]]:
        """发送底盘速度命令"""
        if self.mock_mode:
            return None
        
        import asyncio
        loop = asyncio.get_running_loop()
        future = loop.create_future()
        
        cmd = {
            'type': 'chassis_velocity',
            'params': {
                'linear_x': linear_x,
                'linear_y': linear_y,
                'angular_z': angular_z
            },
            'future': future,
            'loop': loop
        }
        self.command_queue.put(cmd)
        return await future

    async def send_manual_motion_command(
        self,
        forward: bool = False,
        backward: bool = False,
        left: bool = False,
        right: bool = False,
        rotate_left: bool = False,
        rotate_right: bool = False
    ) -> Optional[Dict[str, Any]]:
        """发送手动运动命令"""
        if self.mock_mode:
            return None
        
        import asyncio
        loop = asyncio.get_running_loop()
        future = loop.create_future()
        
        cmd = {
            'type': 'chassis_manual_motion',
            'params': {
                'forward': forward,
                'backward': backward,
                'left': left,
                'right': right,
                'rotate_left': rotate_left,
                'rotate_right': rotate_right
            },
            'future': future,
            'loop': loop
        }
        self.command_queue.put(cmd)
        return await future

    async def call_chassis_service(self, service_name: str) -> Optional[Dict[str, Any]]:
        """调用底盘控制服务"""
        if self.mock_mode:
            return None
        
        if not self.chassis_clients.get(service_name):
            return None
        
        import asyncio
        loop = asyncio.get_running_loop()
        future = loop.create_future()
        
        cmd = {
            'type': 'chassis_control_service',
            'params': {'service_name': service_name},
            'future': future,
            'loop': loop
        }
        self.command_queue.put(cmd)
        return await future

    async def navigate_to_coordinate(
        self,
        x: float,
        y: float,
        yaw: float,
        is_localization: bool = False
    ) -> Optional[Dict[str, Any]]:
        """导航到坐标点"""
        if self.mock_mode:
            return None
        
        import asyncio
        loop = asyncio.get_running_loop()
        future = loop.create_future()
        
        cmd = {
            'type': 'chassis_nav_coordinate',
            'params': {
                'x': x, 'y': y, 'yaw': yaw,
                'is_localization': is_localization
            },
            'future': future,
            'loop': loop
        }
        self.command_queue.put(cmd)
        return await future

    async def navigate_to_site(
        self,
        site_id: int,
        is_localization: bool = False
    ) -> Optional[Dict[str, Any]]:
        """导航到站点（兼容旧接口，使用GoExecuteActionTask）"""
        if self.mock_mode:
            return None
        
        import asyncio
        loop = asyncio.get_running_loop()
        future = loop.create_future()
        
        cmd = {
            'type': 'chassis_nav_site',
            'params': {
                'site_id': site_id,
                'is_localization': is_localization
            },
            'future': future,
            'loop': loop
        }
        self.command_queue.put(cmd)
        return await future
    
    async def navigate_to_site_simple(
        self,
        site_id: int
    ) -> Optional[Dict[str, Any]]:
        """导航到站点（简单版本，不带任务ID）"""
        if self.mock_mode:
            return None
        
        import asyncio
        loop = asyncio.get_running_loop()
        future = loop.create_future()
        
        cmd = {
            'type': 'chassis_nav_site_simple',
            'params': {'site_id': site_id},
            'future': future,
            'loop': loop
        }
        self.command_queue.put(cmd)
        return await future
    
    async def navigate_to_site_with_task(
        self,
        site_id: int,
        task_id: int
    ) -> Optional[Dict[str, Any]]:
        """导航到站点（带任务ID）"""
        if self.mock_mode:
            return None
        
        import asyncio
        loop = asyncio.get_running_loop()
        future = loop.create_future()
        
        cmd = {
            'type': 'chassis_nav_site_task',
            'params': {
                'site_id': site_id,
                'task_id': task_id
            },
            'future': future,
            'loop': loop
        }
        self.command_queue.put(cmd)
        return await future

    async def force_localize(
        self,
        x: float,
        y: float,
        yaw: float
    ) -> Optional[Dict[str, Any]]:
        """强制定位"""
        if self.mock_mode:
            return None
        
        import asyncio
        loop = asyncio.get_running_loop()
        future = loop.create_future()
        
        cmd = {
            'type': 'chassis_force_localize',
            'params': {'x': x, 'y': y, 'yaw': yaw},
            'future': future,
            'loop': loop
        }
        self.command_queue.put(cmd)
        return await future

    async def set_chassis_speed_level(self, level: int) -> Optional[Dict[str, Any]]:
        """设置底盘速度级别"""
        if self.mock_mode:
            return None
        
        import asyncio
        loop = asyncio.get_running_loop()
        future = loop.create_future()
        
        cmd = {
            'type': 'chassis_set_speed_level',
            'params': {'level': level},
            'future': future,
            'loop': loop
        }
        self.command_queue.put(cmd)
        return await future

    async def set_chassis_obstacle_strategy(self, strategy: int) -> Optional[Dict[str, Any]]:
        """设置底盘避障策略"""
        if self.mock_mode:
            return None
        
        import asyncio
        loop = asyncio.get_running_loop()
        future = loop.create_future()
        
        cmd = {
            'type': 'chassis_set_obstacle_strategy',
            'params': {'strategy': strategy},
            'future': future,
            'loop': loop
        }
        self.command_queue.put(cmd)
        return await future

    async def set_chassis_current_site(self, site_id: int) -> Optional[Dict[str, Any]]:
        """设置当前站点"""
        if self.mock_mode:
            return None
        
        import asyncio
        loop = asyncio.get_running_loop()
        future = loop.create_future()
        
        cmd = {
            'type': 'chassis_set_current_site',
            'params': {'site_id': site_id},
            'future': future,
            'loop': loop
        }
        self.command_queue.put(cmd)
        return await future

    async def set_chassis_volume(self, volume: int) -> Optional[Dict[str, Any]]:
        """设置音量"""
        if self.mock_mode:
            return None
        
        import asyncio
        loop = asyncio.get_running_loop()
        future = loop.create_future()
        
        cmd = {
            'type': 'chassis_set_volume',
            'params': {'volume': volume},
            'future': future,
            'loop': loop
        }
        self.command_queue.put(cmd)
        return await future

    async def set_chassis_map(self, map_name: str) -> Optional[Dict[str, Any]]:
        """设置当前地图"""
        if self.mock_mode:
            return None
        
        import asyncio
        loop = asyncio.get_running_loop()
        future = loop.create_future()
        
        cmd = {
            'type': 'chassis_set_map',
            'params': {'map_name': map_name},
            'future': future,
            'loop': loop
        }
        self.command_queue.put(cmd)
        return await future
    
    def send_command(self, cmd: Dict[str, Any]):
        """发送命令到 ROS2（异步安全）"""
        self.command_queue.put(cmd)
    

    async def call_shutdown(self) -> Optional[Dict[str, Any]]:
        """调用系统关机服务"""
        if self.mock_mode:
            return {"success": False, "message": "Mock模式下不执行实际关机"}
        
        import asyncio
        loop = asyncio.get_running_loop()
        future = loop.create_future()
        
        cmd = {
            'type': 'shutdown',
            'future': future,
            'loop': loop
        }
        
        try:
            await self.command_queue.put(cmd)
            result = await asyncio.wait_for(future, timeout=10.0)
            return result
        except asyncio.TimeoutError:
            return {"success": False, "message": "关机命令超时"}
        except Exception as e:
            return {"success": False, "message": f"关机命令失败: {str(e)}"}

    def is_connected(self) -> bool:
        """检查 ROS2 是否连接"""
        if self.mock_mode:
            return True
        return self.running and self.node is not None
    
    def shutdown(self):
        """关闭 ROS2 Bridge"""
        self.running = False
        if self.executor:
            self.executor.shutdown()
        if self.thread:
            self.thread.join(timeout=5.0)
        print("🛑 ROS2 Bridge 已关闭")


# 全局单例
ros2_bridge = ROS2Bridge()
