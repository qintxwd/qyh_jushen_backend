"""
QYH Jushen Control Plane - ROS2 服务客户端

提供与 ROS2 节点通信的统一接口。

⚠️ 重要：此客户端只用于低频管理操作（<5Hz），
   高频控制请使用 Data Plane 的 WebSocket 接口！

支持的 ROS2 服务:
- qyh_bag_recorder: 录制控制
- qyh_task_engine: 任务执行
- qyh_shutdown: 关机控制
- qyh_jaka_control: 机械臂控制
- qyh_lift_control: 升降控制
- qyh_waist_control: 腰部控制
- qyh_gripper_control: 夹爪控制
- qyh_head_motor_control: 头部控制
"""
import asyncio
import json
import logging
import math
import os
import threading
import time
from pathlib import Path
from typing import Optional, Any, Dict
from dataclasses import dataclass, field
from enum import IntEnum

logger = logging.getLogger(__name__)

# ==================== ROS2 可用性检测 ====================

ROS2_AVAILABLE = False
rclpy = None

try:
    import rclpy
    from rclpy.executors import MultiThreadedExecutor
    # from rclpy.node import Node
    ROS2_AVAILABLE = True
    logger.info("ROS2 (rclpy) is available")
except ImportError:
    logger.error("ROS2 (rclpy) not available")


# ==================== 服务定义常量 ====================

class LiftCommand(IntEnum):
    """升降控制命令类型 (对应 LiftControl.srv)"""
    ENABLE = 1
    DISABLE = 2
    SET_SPEED = 3
    GO_POSITION = 4
    MOVE_UP = 5
    MOVE_DOWN = 6
    RESET_ALARM = 7
    STOP = 8
    ELECTROMAGNET = 9


class WaistCommand(IntEnum):
    """腰部控制命令类型 (对应 WaistControl.srv)"""
    ENABLE = 1
    DISABLE = 2
    SET_SPEED = 3
    GO_POSITION = 4
    GO_ANGLE = 5
    LEAN_FORWARD = 6
    LEAN_BACK = 7
    RESET_ALARM = 8
    STOP = 9
    GO_UPRIGHT = 10


class RobotSide(IntEnum):
    """机械臂选择 (对应 MoveJ.srv robot_id)"""
    LEFT = 0
    RIGHT = 1
    DUAL = -1


# ==================== 服务响应数据类 ====================

@dataclass
class ServiceResponse:
    """通用服务响应"""
    success: bool
    message: str = ""
    data: dict = field(default_factory=dict)


@dataclass
class RecordingStatus:
    """录制状态"""
    is_recording: bool = False
    action_name: str = ""
    duration_sec: float = 0.0
    bag_path: str = ""
    topics: list = field(default_factory=list)


# ==================== ROS2 服务客户端 ====================

class ROS2ServiceClient:
    """
    ROS2 服务客户端
    
    使用真实 ROS2 服务调用与话题订阅。
    
    使用示例:
    ```python
    client = ROS2ServiceClient()
    await client.initialize()
    
    # 录制控制
    result = await client.start_recording("pickup_cube", "admin", "1.0", [...])
    status = await client.get_recording_status()
    
    # 关机
    result = await client.request_shutdown()
    
    await client.shutdown()
    ```
    """
    
    _instance: Optional['ROS2ServiceClient'] = None
    
    def __new__(cls):
        """单例模式"""
        if cls._instance is None:
            cls._instance = super().__new__(cls)
            cls._instance._initialized = False
        return cls._instance
    
    def __init__(self):
        if self._initialized:
            return

        if not ROS2_AVAILABLE:
            raise RuntimeError("ROS2 (rclpy) not available")
        
        self._node = None
        self._executor = None
        self._spin_thread = None
        self._service_clients = {}

        # 订阅状态缓存
        self._latest_joint_state: Optional[Dict[str, Any]] = None
        self._latest_left_arm_joint_state: Optional[Dict[str, Any]] = None
        self._latest_right_arm_joint_state: Optional[Dict[str, Any]] = None
        self._latest_head_joint_state: Optional[Dict[str, Any]] = None
        self._latest_lift_state: Optional[Dict[str, Any]] = None
        self._latest_waist_state: Optional[Dict[str, Any]] = None
        self._latest_left_gripper_state: Optional[Dict[str, Any]] = None
        self._latest_right_gripper_state: Optional[Dict[str, Any]] = None
        self._latest_robot_status: Optional[Dict[str, Any]] = None
        self._latest_odom: Optional[Dict[str, Any]] = None
        self._latest_shutdown_state: Optional[Dict[str, Any]] = None
        self._state_lock = threading.Lock()

        # 缓存 joint_states 的索引映射，避免每次 get_robot_state 都解析
        # 结构: { "left_arm": [indice1, ...], "head": [...], "hash": ... }
        self._joint_indices_cache = {}
        
        # 紧急停止发布器
        self._emergency_stop_publisher = None
        # LED 发布器
        self._led_color_publisher = None
        self._led_blink_publisher = None

        # 注意: VR 状态不再由 Control Plane 管理
        # VR 使用 Data Plane 的专用 WebSocket 通道 (/vr)
        # 参见 VR_ARCHITECTURE.md

        # 底盘持久化配置应用状态
        self._chassis_config_applied = False
        self._chassis_config_last_attempt = 0.0
        self._chassis_config_retry_interval = 5.0
        self._chassis_config_lock = threading.Lock()
        
        self._initialized = True
    
    async def initialize(self) -> bool:
        """
        初始化 ROS2 节点和服务客户端
        
        Returns:
            bool: 是否成功初始化
        """
        if not ROS2_AVAILABLE:
            raise RuntimeError("ROS2 (rclpy) not available")
        
        try:
            if not rclpy.ok():
                rclpy.init()
            
            self._node = rclpy.create_node('control_plane_client')
            
            # 使用多线程执行器
            self._executor = MultiThreadedExecutor()
            self._executor.add_node(self._node)

            # 启动后台 Spin 线程
            self._spin_thread = threading.Thread(
                target=self._spin_background,
                daemon=True,
            )
            self._spin_thread.start()

            # 创建服务客户端
            await self._create_service_clients()

            # 创建订阅（状态）
            await self._create_subscriptions()
            
            # 创建发布器
            await self._create_publishers()
            
            logger.info("ROS2 client initialized successfully")
            return True
            
        except Exception as e:
            logger.error(f"Failed to initialize ROS2 client: {e}")
            return False

    def _spin_background(self):
        """后台持续 Spin，处理订阅消息"""
        if not self._executor:
            return
        
        try:
            # 这里的 spin 会阻塞，直到 executor shutdown
            self._executor.spin()
        except Exception as e:
            # shutdown 时可能会抛出异常，忽略即可
            if rclpy.ok():
                logger.error(f"ROS2 executor spin failed: {e}")
    
    async def _create_service_clients(self):
        """创建所有服务客户端"""
        if self._node is None:
            return
        
        try:
            # 动态导入服务类型（避免在非 ROS2 环境报错）
            from qyh_bag_recorder.srv import (
                StartRecording,
                StopRecording,
                GetRecordingStatus,
            )
            from qyh_task_engine_msgs.srv import (
                ExecuteTask,
                CancelTask,
                PauseTask,
                ResumeTask,
            )
            from std_srvs.srv import Trigger as ShutdownTrigger
            from qyh_jaka_control_msgs.srv import MoveJ
            from qyh_lift_msgs.srv import LiftControl
            from qyh_waist_msgs.srv import WaistControl
            from qyh_gripper_msgs.srv import MoveGripper
            from std_srvs.srv import SetBool
            from qyh_standard_robot_msgs.srv import (
                GoSetSpeedType,
                GoSetSpeakerVolume,
                GoSetObstacleStrategy,
            )
            
            # 录制服务
            self._service_clients['start_recording'] = (
                self._node.create_client(
                    StartRecording,
                    '/bag_recorder/start_recording',
                )
            )
            self._service_clients['stop_recording'] = self._node.create_client(
                StopRecording, '/bag_recorder/stop_recording'
            )
            self._service_clients['get_recording_status'] = (
                self._node.create_client(
                    GetRecordingStatus,
                    '/bag_recorder/get_status',
                )
            )
            
            # 任务服务
            self._service_clients['execute_task'] = self._node.create_client(
                ExecuteTask, '/task_engine/execute'
            )
            self._service_clients['cancel_task'] = self._node.create_client(
                CancelTask, '/task_engine/cancel'
            )
            self._service_clients['pause_task'] = self._node.create_client(
                PauseTask, '/task_engine/pause_task'
            )
            self._service_clients['resume_task'] = self._node.create_client(
                ResumeTask, '/task_engine/resume_task'
            )
            
            # 关机服务
            self._service_clients['shutdown'] = self._node.create_client(
                ShutdownTrigger, '/qyh_shutdown'
            )
            
            # 机械臂服务
            self._service_clients['arm_move_j'] = self._node.create_client(
                MoveJ, '/jaka/move_j'
            )
            
            # 升降服务
            self._service_clients['lift_control'] = self._node.create_client(
                LiftControl, '/lift/control'
            )
            
            # 腰部服务
            self._service_clients['waist_control'] = self._node.create_client(
                WaistControl, '/waist/control'
            )
            
            # 夹爪服务
            self._service_clients['gripper_left'] = self._node.create_client(
                MoveGripper, '/left/move_gripper'
            )
            self._service_clients['gripper_right'] = self._node.create_client(
                MoveGripper, '/right/move_gripper'
            )
            
            # 头部服务
            self._service_clients['head_enable_torque'] = (
                self._node.create_client(
                    SetBool,
                    '/head_motor_node/enable_torque',
                )
            )

            # 底盘参数服务
            self._service_clients['chassis_set_speed_level'] = (
                self._node.create_client(
                    GoSetSpeedType,
                    'go_set_speed_level',
                )
            )
            self._service_clients['chassis_set_volume'] = (
                self._node.create_client(
                    GoSetSpeakerVolume,
                    'go_set_speaker_volume',
                )
            )
            self._service_clients['chassis_set_obstacle'] = (
                self._node.create_client(
                    GoSetObstacleStrategy,
                    'go_set_obstacle_strategy',
                )
            )
            
            logger.info(
                "Created %d ROS2 service clients",
                len(self._service_clients),
            )
            
        except ImportError as e:
            logger.warning(f"Some ROS2 message types not available: {e}")

    async def _create_subscriptions(self):
        """创建状态订阅"""
        if self._node is None:
            return

        try:
            from sensor_msgs.msg import JointState
            from nav_msgs.msg import Odometry
            from qyh_lift_msgs.msg import LiftState
            from qyh_waist_msgs.msg import WaistState
            from qyh_gripper_msgs.msg import GripperState
            from qyh_standard_robot_msgs.msg import StandardRobotStatus

            self._node.create_subscription(
                JointState,
                '/joint_states',
                self._on_joint_state,
                10
            )

            self._node.create_subscription(
                JointState,
                '/left_arm/joint_states',
                self._on_left_arm_joint_state,
                10
            )

            self._node.create_subscription(
                JointState,
                '/right_arm/joint_states',
                self._on_right_arm_joint_state,
                10
            )

            self._node.create_subscription(
                JointState,
                '/head/joint_states',
                self._on_head_joint_state,
                10
            )

            self._node.create_subscription(
                LiftState,
                '/lift/state',
                self._on_lift_state,
                10
            )

            self._node.create_subscription(
                WaistState,
                '/waist/state',
                self._on_waist_state,
                10
            )

            self._node.create_subscription(
                GripperState,
                '/left/gripper_state',
                self._on_left_gripper_state,
                10
            )

            self._node.create_subscription(
                GripperState,
                '/right/gripper_state',
                self._on_right_gripper_state,
                10
            )

            self._node.create_subscription(
                StandardRobotStatus,
                '/standard_robot_node/standard_robot_status',
                self._on_standard_robot_status,
                10
            )

            self._node.create_subscription(
                Odometry,
                '/odom',
                self._on_odom,
                10
            )

            # 注意: VR 订阅已移除
            # VR 使用 Data Plane 的专用 WebSocket 通道 (/vr)
            # 参见 VR_ARCHITECTURE.md

            # 订阅关机状态（如果有）
            # TODO: 需要定义 ShutdownState 消息类型
            # from qyh_shutdown_msgs.msg import ShutdownState
            # self._node.create_subscription(
            #     ShutdownState,
            #     '/qyh_shutdown_node/state',
            #     self._on_shutdown_state,
            #     10
            # )

            logger.info("ROS2 subscriptions created: /joint_states, ...")

        except ImportError as e:
            logger.warning(f"ROS2 topic types not available: {e}")

    async def _create_publishers(self):
        """创建发布器"""
        if self._node is None:
            return
        
        try:
            from geometry_msgs.msg import Twist
            from std_msgs.msg import ColorRGBA, String
            
            # 创建 cmd_vel 发布器（用于紧急停止）
            self._emergency_stop_publisher = self._node.create_publisher(
                Twist,
                '/cmd_vel',
                10
            )

            # LED 发布器
            self._led_color_publisher = self._node.create_publisher(
                ColorRGBA,
                '/robot_led/set_color',
                10
            )
            self._led_blink_publisher = self._node.create_publisher(
                String,
                '/robot_led/blink',
                10
            )
            
            logger.info("ROS2 publishers created: /cmd_vel, /robot_led/*")
            
        except ImportError as e:
            logger.warning(
                f"ROS2 message types for publishers not available: {e}"
            )

    async def _wait_for_future(self, future, timeout=5.0):
        """等待 ROS2 Future 完成 (非阻塞)"""
        start_time = time.time()
        while not future.done():
            if time.time() - start_time > timeout:
                future.cancel()
                return None
            await asyncio.sleep(0.01)
        return future.result()

    def _on_joint_state(self, msg):
        data = {
            "stamp": time.time(),
            "name": list(msg.name),
            "position": list(msg.position),
            "velocity": list(msg.velocity),
            "effort": list(msg.effort),
        }
        with self._state_lock:
            self._latest_joint_state = data

    def _on_left_arm_joint_state(self, msg):
        data = {
            "stamp": time.time(),
            "name": list(msg.name),
            "position": list(msg.position),
            "velocity": list(msg.velocity),
            "effort": list(msg.effort),
        }
        with self._state_lock:
            self._latest_left_arm_joint_state = data

    def _on_right_arm_joint_state(self, msg):
        data = {
            "stamp": time.time(),
            "name": list(msg.name),
            "position": list(msg.position),
            "velocity": list(msg.velocity),
            "effort": list(msg.effort),
        }
        with self._state_lock:
            self._latest_right_arm_joint_state = data

    def _on_head_joint_state(self, msg):
        data = {
            "stamp": time.time(),
            "name": list(msg.name),
            "position": list(msg.position),
            "velocity": list(msg.velocity),
            "effort": list(msg.effort),
        }
        with self._state_lock:
            self._latest_head_joint_state = data

    def _on_lift_state(self, msg):
        data = {
            "stamp": time.time(),
            "enabled": msg.enabled,
            "current_position": msg.current_position,
            "current_speed": msg.current_speed,
            "alarm": msg.alarm,
            "position_reached": msg.position_reached,
            "electromagnet_on": msg.electromagnet_on,
            "connected": msg.connected,
            "shutdown_requested": msg.shutdown_requested,
        }
        with self._state_lock:
            self._latest_lift_state = data

    def _on_waist_state(self, msg):
        data = {
            "stamp": time.time(),
            "enabled": msg.enabled,
            "current_position": msg.current_position,
            "current_angle": msg.current_angle,
            "current_speed": msg.current_speed,
            "alarm": msg.alarm,
            "position_reached": msg.position_reached,
            "connected": msg.connected,
        }
        with self._state_lock:
            self._latest_waist_state = data

    def _on_left_gripper_state(self, msg):
        data = {
            "stamp": time.time(),
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
            "communication_ok": msg.communication_ok,
        }
        with self._state_lock:
            self._latest_left_gripper_state = data

    def _on_right_gripper_state(self, msg):
        data = {
            "stamp": time.time(),
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
            "communication_ok": msg.communication_ok,
        }
        with self._state_lock:
            self._latest_right_gripper_state = data

    def _on_standard_robot_status(self, msg):
        data = {
            "stamp": time.time(),
            "pose": msg.pose.pose,
            "twist": msg.twist,
            "battery_remaining_percentage": msg.battery_remaining_percentage,
            "is_emergency_stopped": msg.is_emergency_stopped,
            "is_charging": msg.is_charging,
            "system_status": msg.system_status,
            "location_status": msg.location_status,
            "operation_status": msg.operation_status,
        }
        with self._state_lock:
            self._latest_robot_status = data

    def _on_odom(self, msg):
        data = {
            "stamp": time.time(),
            "pose": msg.pose.pose,
            "twist": msg.twist.twist,
        }
        with self._state_lock:
            self._latest_odom = data

    def _on_shutdown_state(self, msg):
        """关机状态回调（当消息类型可用时）"""
        data = {
            "shutdown_in_progress": msg.shutdown_in_progress,
            "trigger_source": msg.trigger_source,
            "countdown_seconds": msg.countdown_seconds,
            "plc_connected": msg.plc_connected,
        }
        with self._state_lock:
            self._latest_shutdown_state = data

    # 注意: VR 回调函数已移除 (原 _on_vr_* 系列)
    # VR 状态由 Data Plane 管理，参见 VR_ARCHITECTURE.md

    def get_shutdown_state(self) -> Dict[str, Any]:
        """获取关机状态"""
        with self._state_lock:
            if self._latest_shutdown_state:
                return self._latest_shutdown_state.copy()
        
        # 返回默认状态
        return {
            "shutdown_in_progress": False,
            "trigger_source": 0,
            "countdown_seconds": -1,
            "plc_connected": False,
        }

    def get_standard_robot_status(self) -> Optional[Dict[str, Any]]:
        """获取标准机器人状态 (底盘状态)"""
        with self._state_lock:
            if self._latest_robot_status:
                return self._latest_robot_status.copy()
        return None

    def get_odom(self) -> Optional[Dict[str, Any]]:
        """获取里程计数据 (位姿和速度)"""
        with self._state_lock:
            if self._latest_odom:
                return self._latest_odom.copy()
        return None

    async def publish_cmd_vel(
        self, linear_x: float, linear_y: float, angular_z: float
    ) -> None:
        """发布速度命令到 /cmd_vel"""
        if not self._emergency_stop_publisher:
            raise RuntimeError("cmd_vel publisher not initialized")
        
        try:
            from geometry_msgs.msg import Twist
            msg = Twist()
            msg.linear.x = float(linear_x)
            msg.linear.y = float(linear_y)
            msg.linear.z = 0.0
            msg.angular.x = 0.0
            msg.angular.y = 0.0
            msg.angular.z = float(angular_z)
            self._emergency_stop_publisher.publish(msg)
        except Exception as e:
            logger.error(f"publish_cmd_vel error: {e}")
            raise

    async def navigate_to_pose(
        self, x: float, y: float, yaw: float
    ) -> ServiceResponse:
        """导航到指定位姿"""
        # TODO: 实现 Nav2 导航接口
        # 目前返回不支持
        return ServiceResponse(
            success=False,
            message="导航服务尚未实现，请使用底盘原生导航接口"
        )

    async def navigate_to_station(self, station_id: int) -> ServiceResponse:
        """导航到站点"""
        # TODO: 实现站点导航接口
        return ServiceResponse(
            success=False,
            message="站点导航服务尚未实现，请使用底盘原生导航接口"
        )

    # 注意: get_vr_state() 已移除
    # VR 状态由 Data Plane 管理，查询接口见 /api/v1/vr/status
    # 实时 VR 位姿请订阅 Data Plane 的 VRSystemState 话题

    def get_robot_state(self) -> Optional[Dict[str, Any]]:
        """返回当前机器人状态（基于 ROS2 topic 缓存）"""
        if self._node is None:
            return None

        # [REFACTORED] 不再需要主动 spin_once，后台线程在持续 spin
        # try:
        #     rclpy.spin_once(self._node, timeout_sec=0.0)
        # except Exception:
        #     pass

        # 复制缓存
        with self._state_lock:
            latest_joint_state = self._latest_joint_state
            latest_left_arm_joint_state = self._latest_left_arm_joint_state
            latest_right_arm_joint_state = self._latest_right_arm_joint_state
            # ... copies refs ...
            latest_head_joint_state = self._latest_head_joint_state
            latest_lift_state = self._latest_lift_state
            latest_waist_state = self._latest_waist_state
            latest_left_gripper_state = self._latest_left_gripper_state
            latest_right_gripper_state = self._latest_right_gripper_state
            latest_robot_status = self._latest_robot_status
            latest_odom = self._latest_odom

        if (
            not latest_joint_state
            and not latest_left_arm_joint_state
            and not latest_right_arm_joint_state
            and not latest_head_joint_state
            and not latest_lift_state
            and not latest_waist_state
            and not latest_left_gripper_state
            and not latest_right_gripper_state
            and not latest_robot_status
            and not latest_odom
        ):
            return None

        state: Dict[str, Any] = {}

        # 解析关节状态
        if latest_joint_state:
            names = latest_joint_state.get("name", [])
            positions = latest_joint_state.get("position", [])
            velocities = latest_joint_state.get("velocity", [])
            efforts = latest_joint_state.get("effort", [])
            
            # [OPTIMIZED] 缓存关节名称分类索引，避免每帧重复字符串匹配
            names_tuple = tuple(names)
            cache_entry = self._joint_indices_cache.get("joint_idx")
            
            if cache_entry is None or cache_entry["names"] != names_tuple:
                # 重新构建缓存
                left_arm_indices = []
                right_arm_indices = []
                head_indices = []
                lift_indices = []
                waist_indices = []
                left_gripper_idx = -1
                right_gripper_idx = -1

                for i, n in enumerate(names):
                    if "gripper" in n:
                        if "left" in n or n.startswith("l_"):
                            left_gripper_idx = i
                        elif "right" in n or n.startswith("r_"):
                            right_gripper_idx = i
                    elif "head" in n:
                        head_indices.append(i)
                    elif "lift" in n:
                        lift_indices.append(i)
                    elif "waist" in n:
                        waist_indices.append(i)
                    elif (
                        "left" in n
                        or n.startswith("l_")
                        or n.startswith("l-")
                    ):
                        left_arm_indices.append(i)
                    elif (
                        "right" in n
                        or n.startswith("r_")
                        or n.startswith("r-")
                    ):
                        right_arm_indices.append(i)
                
                cache_entry = {
                    "names": names_tuple,
                    "left_arm": left_arm_indices,
                    "right_arm": right_arm_indices,
                    "head": head_indices,
                    "lift": lift_indices,
                    "waist": waist_indices,
                    "left_gripper": left_gripper_idx,
                    "right_gripper": right_gripper_idx,
                }
                self._joint_indices_cache["joint_idx"] = cache_entry

            # 使用缓存的索引直接提取数据
            def get_joints(indices):
                res = []
                for i in indices:
                    pos = positions[i] if i < len(positions) else 0.0
                    vel = velocities[i] if i < len(velocities) else 0.0
                    eff = efforts[i] if i < len(efforts) else 0.0
                    res.append({
                        "name": names[i],
                        "position": pos,
                        "velocity": vel,
                        "effort": eff,
                    })
                return res

            if cache_entry["left_arm"]:
                state["left_arm"] = {
                    "joints": get_joints(cache_entry["left_arm"]),
                    "joint_count": len(cache_entry["left_arm"]),
                }
            if cache_entry["right_arm"]:
                state["right_arm"] = {
                    "joints": get_joints(cache_entry["right_arm"]),
                    "joint_count": len(cache_entry["right_arm"]),
                }
            if cache_entry["head"]:
                state["head"] = {
                    "joints": get_joints(cache_entry["head"]),
                    "joint_count": len(cache_entry["head"]),
                }
            if cache_entry["lift"]:
                state["lift"] = {
                    "joints": get_joints(cache_entry["lift"]),
                    "joint_count": len(cache_entry["lift"]),
                }
            if cache_entry["waist"]:
                state["waist"] = {
                    "joints": get_joints(cache_entry["waist"]),
                    "joint_count": len(cache_entry["waist"]),
                }
            
            if cache_entry["left_gripper"] != -1:
                i = cache_entry["left_gripper"]
                p = positions[i] if i < len(positions) else 0.0
                e = efforts[i] if i < len(efforts) else 0.0
                state["left_gripper"] = {
                    "position": p,
                    "force": e,
                    "is_grasping": e > 0.1,
                }
            if cache_entry["right_gripper"] != -1:
                i = cache_entry["right_gripper"]
                p = positions[i] if i < len(positions) else 0.0
                e = efforts[i] if i < len(efforts) else 0.0
                state["right_gripper"] = {
                    "position": p,
                    "force": e,
                    "is_grasping": e > 0.1,
                }

        # 头部关节状态（专用话题）
        if latest_head_joint_state:
            names = latest_head_joint_state.get("name", [])
            positions = latest_head_joint_state.get("position", [])
            velocities = latest_head_joint_state.get("velocity", [])
            efforts = latest_head_joint_state.get("effort", [])
            head_joints = []
            for idx, name in enumerate(names):
                head_joints.append({
                    "name": name,
                    "position": (
                        positions[idx] if idx < len(positions) else 0.0
                    ),
                    "velocity": (
                        velocities[idx] if idx < len(velocities) else 0.0
                    ),
                    "effort": efforts[idx] if idx < len(efforts) else 0.0,
                })
            if head_joints:
                state["head"] = {
                    "joints": head_joints,
                    "joint_count": len(head_joints),
                }

        # 左/右机械臂关节状态（专用话题）
        if latest_left_arm_joint_state:
            names = latest_left_arm_joint_state.get("name", [])
            positions = latest_left_arm_joint_state.get("position", [])
            velocities = latest_left_arm_joint_state.get("velocity", [])
            efforts = latest_left_arm_joint_state.get("effort", [])
            joints = []
            for idx, name in enumerate(names):
                joints.append({
                    "name": name,
                    "position": (
                        positions[idx] if idx < len(positions) else 0.0
                    ),
                    "velocity": (
                        velocities[idx] if idx < len(velocities) else 0.0
                    ),
                    "effort": efforts[idx] if idx < len(efforts) else 0.0,
                })
            if joints:
                state["left_arm"] = {
                    "joints": joints,
                    "joint_count": len(joints),
                }

        if latest_right_arm_joint_state:
            names = latest_right_arm_joint_state.get("name", [])
            positions = latest_right_arm_joint_state.get("position", [])
            velocities = latest_right_arm_joint_state.get("velocity", [])
            efforts = latest_right_arm_joint_state.get("effort", [])
            joints = []
            for idx, name in enumerate(names):
                joints.append({
                    "name": name,
                    "position": (
                        positions[idx] if idx < len(positions) else 0.0
                    ),
                    "velocity": (
                        velocities[idx] if idx < len(velocities) else 0.0
                    ),
                    "effort": efforts[idx] if idx < len(efforts) else 0.0,
                })
            if joints:
                state["right_arm"] = {
                    "joints": joints,
                    "joint_count": len(joints),
                }

        # 升降/腰部状态
        if latest_lift_state:
            state["lift"] = latest_lift_state
        if latest_waist_state:
            state["waist"] = latest_waist_state

        # 夹爪状态
        if latest_left_gripper_state:
            state["left_gripper"] = {
                "position": (
                    latest_left_gripper_state.get("current_position", 0)
                    / 255.0
                ),
                "force": latest_left_gripper_state.get("current_force", 0),
                "is_grasping": (
                    latest_left_gripper_state.get("object_status", 0) == 2
                ),
            }
        if latest_right_gripper_state:
            state["right_gripper"] = {
                "position": (
                    latest_right_gripper_state.get("current_position", 0)
                    / 255.0
                ),
                "force": latest_right_gripper_state.get("current_force", 0),
                "is_grasping": (
                    latest_right_gripper_state.get("object_status", 0) == 2
                ),
            }

        # 解析底盘状态（优先 standard_robot_status）
        if latest_robot_status:
            pose = latest_robot_status["pose"]
            twist = latest_robot_status["twist"]

            qx = pose.orientation.x
            qy = pose.orientation.y
            qz = pose.orientation.z
            qw = pose.orientation.w

            siny_cosp = 2.0 * (qw * qz + qx * qy)
            cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
            yaw = math.atan2(siny_cosp, cosy_cosp)

            state["chassis"] = {
                "x": pose.position.x,
                "y": pose.position.y,
                "theta": yaw,
                "linear_velocity": twist.linear.x,
                "angular_velocity": twist.angular.z,
            }

            state["battery"] = latest_robot_status.get(
                "battery_remaining_percentage"
            )

        # 解析底盘状态（备用 odom）
        elif latest_odom:
            pose = latest_odom["pose"]
            twist = latest_odom["twist"]

            qx = pose.orientation.x
            qy = pose.orientation.y
            qz = pose.orientation.z
            qw = pose.orientation.w

            siny_cosp = 2.0 * (qw * qz + qx * qy)
            cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
            yaw = math.atan2(siny_cosp, cosy_cosp)

            state["chassis"] = {
                "x": pose.position.x,
                "y": pose.position.y,
                "theta": yaw,
                "linear_velocity": twist.linear.x,
                "angular_velocity": twist.angular.z,
            }

        return state

    # ==================== 底盘持久化配置 ====================

    def _get_chassis_config_file(self) -> Path:
        """获取底盘配置文件路径"""
        workspace_root = Path(
            os.environ.get(
                'QYH_WORKSPACE_ROOT',
                Path.home() / 'qyh-robot-system',
            )
        )
        config_dir = workspace_root / "persistent" / "web"
        config_dir.mkdir(parents=True, exist_ok=True)
        return config_dir / "chassis_config.json"

    def _load_chassis_config(self) -> Dict[str, Any]:
        """加载底盘配置"""
        config_file = self._get_chassis_config_file()
        if config_file.exists():
            try:
                return json.loads(config_file.read_text(encoding='utf-8'))
            except Exception:
                pass
        return {
            "speed_level": 50,
            "volume": 50,
            "obstacle_strategy": 1,
        }

    async def set_chassis_speed_level(self, level: int) -> ServiceResponse:
        """设置底盘速度级别"""
        if self._node is None:
            return ServiceResponse(False, "ROS2 client not initialized")

        try:
            from qyh_standard_robot_msgs.srv import GoSetSpeedType

            client = self._service_clients.get('chassis_set_speed_level')
            if not client or not client.wait_for_service(timeout_sec=2.0):
                return ServiceResponse(False, "底盘速度服务不可用")

            request = GoSetSpeedType.Request()
            request.speed_level = int(level)

            future = client.call_async(request)
            result = await self._wait_for_future(future, timeout=5.0)

            if result is not None:
                return ServiceResponse(result.success, result.message)
            return ServiceResponse(False, "服务调用超时")
        except Exception as e:
            logger.error(f"set_chassis_speed_level error: {e}")
            return ServiceResponse(False, str(e))

    async def set_chassis_volume(self, volume: int) -> ServiceResponse:
        """设置底盘扬声器音量"""
        if self._node is None:
            return ServiceResponse(False, "ROS2 client not initialized")

        try:
            from qyh_standard_robot_msgs.srv import GoSetSpeakerVolume

            client = self._service_clients.get('chassis_set_volume')
            if not client or not client.wait_for_service(timeout_sec=2.0):
                return ServiceResponse(False, "底盘音量服务不可用")

            request = GoSetSpeakerVolume.Request()
            request.volume = int(volume)

            future = client.call_async(request)
            result = await self._wait_for_future(future, timeout=5.0)

            if result is not None:
                return ServiceResponse(result.success, result.message)
            return ServiceResponse(False, "服务调用超时")
        except Exception as e:
            logger.error(f"set_chassis_volume error: {e}")
            return ServiceResponse(False, str(e))

    async def apply_chassis_config_if_needed(
        self,
        chassis_connected: bool,
    ) -> None:
        """底盘连接后应用持久化配置"""
        if not chassis_connected:
            return

        now = time.time()
        with self._chassis_config_lock:
            if self._chassis_config_applied:
                return
            if (
                now - self._chassis_config_last_attempt
                < self._chassis_config_retry_interval
            ):
                return
            self._chassis_config_last_attempt = now

        config = self._load_chassis_config()
        speed_level = config.get("speed_level", 50)
        volume = config.get("volume", 50)

        speed_result = await self.set_chassis_speed_level(speed_level)
        volume_result = await self.set_chassis_volume(volume)

        if speed_result.success and volume_result.success:
            with self._chassis_config_lock:
                self._chassis_config_applied = True
        else:
            logger.warning(
                "Apply chassis config failed: speed=%s, volume=%s",
                speed_result.message,
                volume_result.message,
            )

    # ==================== LED 控制 ====================

    async def set_led_color(
        self,
        r: int,
        g: int,
        b: int,
        w: int = 0,
    ) -> ServiceResponse:
        """设置 LED 纯色"""
        if self._node is None or self._led_color_publisher is None:
            return ServiceResponse(False, "LED 发布器未初始化")

        try:
            from std_msgs.msg import ColorRGBA

            msg = ColorRGBA()
            msg.r = max(0.0, min(1.0, r / 255.0))
            msg.g = max(0.0, min(1.0, g / 255.0))
            msg.b = max(0.0, min(1.0, b / 255.0))
            msg.a = max(0.0, min(1.0, w / 255.0))
            self._led_color_publisher.publish(msg)
            return ServiceResponse(True, "LED 颜色已发送")
        except Exception as e:
            logger.error(f"set_led_color error: {e}")
            return ServiceResponse(False, str(e))

    async def set_led_blink(self, command: str) -> ServiceResponse:
        """设置 LED 闪烁模式"""
        if self._node is None or self._led_blink_publisher is None:
            return ServiceResponse(False, "LED 闪烁发布器未初始化")

        try:
            from std_msgs.msg import String

            msg = String()
            msg.data = command
            self._led_blink_publisher.publish(msg)
            return ServiceResponse(True, "LED 闪烁指令已发送")
        except Exception as e:
            logger.error(f"set_led_blink error: {e}")
            return ServiceResponse(False, str(e))
    
    # ==================== LED 控制 ====================

    async def set_led_color(
        self,
        r: int,
        g: int,
        b: int,
        w: int = 0,
    ) -> ServiceResponse:
        """设置 LED 纯色"""
        if self._node is None or self._led_color_publisher is None:
            return ServiceResponse(False, "LED 发布器未初始化")

        try:
            from std_msgs.msg import ColorRGBA

            msg = ColorRGBA()
            msg.r = max(0.0, min(1.0, r / 255.0))
            msg.g = max(0.0, min(1.0, g / 255.0))
            msg.b = max(0.0, min(1.0, b / 255.0))
            msg.a = max(0.0, min(1.0, w / 255.0))
            self._led_color_publisher.publish(msg)
            return ServiceResponse(True, "LED 颜色已发送")
        except Exception as e:
            logger.error(f"set_led_color error: {e}")
            return ServiceResponse(False, str(e))

    async def set_led_blink(self, command: str) -> ServiceResponse:
        """设置 LED 闪烁模式"""
        if self._node is None or self._led_blink_publisher is None:
            return ServiceResponse(False, "LED 闪烁发布器未初始化")

        try:
            from std_msgs.msg import String

            msg = String()
            msg.data = command
            self._led_blink_publisher.publish(msg)
            return ServiceResponse(True, "LED 闪烁指令已发送")
        except Exception as e:
            logger.error(f"set_led_blink error: {e}")
            return ServiceResponse(False, str(e))
    
    async def shutdown(self):
        """关闭 ROS2 节点"""
        # 关闭执行器
        if self._executor:
            self._executor.shutdown()
            self._executor = None
        
        # 等待后台线程退出
        if self._spin_thread and self._spin_thread.is_alive():
            self._spin_thread.join(timeout=1.0)
            self._spin_thread = None

        if self._node is not None:
            self._node.destroy_node()
            self._node = None
        
        if ROS2_AVAILABLE and rclpy.ok():
            rclpy.shutdown()
        
        logger.info("ROS2 client shutdown")
    
    # ==================== 录制服务 ====================
    
    async def start_recording(
        self,
        action_name: str,
        user_name: str,
        version: str,
        topics: list[str],
    ) -> ServiceResponse:
        """
        开始录制
        
        Args:
            action_name: 动作名称
            user_name: 用户名
            version: 版本号
            topics: 要录制的话题列表
            
        Returns:
            ServiceResponse: 包含 success, message, data(bag_path)
        """
        if self._node is None:
            return ServiceResponse(False, "ROS2 client not initialized")
        
        try:
            from qyh_bag_recorder.srv import StartRecording
            
            client = self._service_clients.get('start_recording')
            if not client or not client.wait_for_service(timeout_sec=2.0):
                return ServiceResponse(False, "录制服务不可用")
            
            request = StartRecording.Request()
            request.action_name = action_name
            request.user_name = user_name
            request.version = version
            request.topics = topics
            
            future = client.call_async(request)
            result = await self._wait_for_future(future, timeout=5.0)

            if result is not None:
                return ServiceResponse(
                    success=result.success,
                    message=result.message,
                    data={"bag_path": result.bag_path}
                )
            else:
                return ServiceResponse(False, "服务调用超时")
                
        except Exception as e:
            logger.error(f"start_recording error: {e}")
            return ServiceResponse(False, str(e))
    
    async def stop_recording(self) -> ServiceResponse:
        """
        停止录制
        
        Returns:
            ServiceResponse: 包含 success, message, data(duration_sec, bag_path)
        """
        if self._node is None:
            return ServiceResponse(False, "ROS2 client not initialized")
        
        try:
            from qyh_bag_recorder.srv import StopRecording
            
            client = self._service_clients.get('stop_recording')
            if not client or not client.wait_for_service(timeout_sec=2.0):
                return ServiceResponse(False, "录制服务不可用")
            
            request = StopRecording.Request()
            
            future = client.call_async(request)
            result = await self._wait_for_future(future, timeout=10.0)

            if result is not None:
                return ServiceResponse(
                    success=result.success,
                    message=result.message,
                    data={
                        "duration_sec": result.duration_sec,
                        "bag_path": result.bag_path
                    }
                )
            else:
                return ServiceResponse(False, "服务调用超时")
                
        except Exception as e:
            logger.error(f"stop_recording error: {e}")
            return ServiceResponse(False, str(e))

    async def get_recording_status(self) -> RecordingStatus:
        """获取录制状态"""
        if self._node is None:
            return RecordingStatus()
            
        try:
            from qyh_bag_recorder.srv import GetRecordingStatus
            
            client = self._service_clients.get('get_recording_status')
            if not client or not client.wait_for_service(timeout_sec=1.0):
                # 服务不可用，假设未录制
                return RecordingStatus()
            
            request = GetRecordingStatus.Request()
            future = client.call_async(request)
            result = await self._wait_for_future(future, timeout=2.0)
            
            if result:
                return RecordingStatus(
                    is_recording=result.is_recording,
                    action_name=result.action_name,
                    duration_sec=result.duration_sec,
                    bag_path=result.bag_path,
                    topics=list(result.topics)
                )
            return RecordingStatus()
            
        except Exception as e:
            logger.error(f"get_recording_status error: {e}")
            return RecordingStatus()
    
    async def get_recording_status(self) -> RecordingStatus:
        """
        获取录制状态
        
        Returns:
            RecordingStatus: 录制状态信息
        """
        if self._node is None:
            return RecordingStatus()
        
        try:
            from qyh_bag_recorder.srv import GetRecordingStatus
            
            client = self._service_clients.get('get_recording_status')
            if not client or not client.wait_for_service(timeout_sec=2.0):
                return RecordingStatus()
            
            request = GetRecordingStatus.Request()
            
            future = client.call_async(request)
            result = await self._wait_for_future(future, timeout=2.0)
            
            if result is not None:
                return RecordingStatus(
                    is_recording=result.is_recording,
                    action_name=result.action_name,
                    duration_sec=result.duration_sec,
                    bag_path=result.bag_path,
                    topics=list(result.topics),
                )
            else:
                return RecordingStatus()
                
        except Exception as e:
            logger.error(f"get_recording_status error: {e}")
            return RecordingStatus()
    
    # ==================== 任务服务 ====================
    
    async def execute_task(
        self,
        task_json: str,
        debug_mode: bool = False,
    ) -> ServiceResponse:
        """
        执行任务
        
        Args:
            task_json: 任务 JSON 描述（行为树结构）
            debug_mode: 是否为调试模式（单步执行）
            
        Returns:
            ServiceResponse: 包含 success, message, data(task_id)
        """
        if self._node is None:
            return ServiceResponse(False, "ROS2 client not initialized")
        
        try:
            from qyh_task_engine_msgs.srv import ExecuteTask
            
            client = self._service_clients.get('execute_task')
            if not client or not client.wait_for_service(timeout_sec=2.0):
                return ServiceResponse(False, "任务引擎服务不可用")
            
            request = ExecuteTask.Request()
            request.task_json = task_json
            request.debug_mode = debug_mode
            
            future = client.call_async(request)
            result = await self._wait_for_future(future, timeout=5.0)

            if result is not None:
                return ServiceResponse(
                    success=result.success,
                    message=result.message,
                    data={"task_id": result.task_id}
                )
            else:
                return ServiceResponse(False, "服务调用超时")
                
        except Exception as e:
            logger.error(f"execute_task error: {e}")
            return ServiceResponse(False, str(e))
    
    async def cancel_task(self, task_id: str) -> ServiceResponse:
        """取消任务"""
        if self._node is None:
            return ServiceResponse(False, "ROS2 client not initialized")
        
        try:
            from qyh_task_engine_msgs.srv import CancelTask
            
            client = self._service_clients.get('cancel_task')
            if not client or not client.wait_for_service(timeout_sec=2.0):
                return ServiceResponse(False, "任务引擎服务不可用")
            
            request = CancelTask.Request()
            request.task_id = task_id
            
            future = client.call_async(request)
            result = await self._wait_for_future(future, timeout=5.0)
            
            if result is not None:
                return ServiceResponse(result.success, result.message)
            else:
                return ServiceResponse(False, "服务调用超时")
                
        except Exception as e:
            logger.error(f"cancel_task error: {e}")
            return ServiceResponse(False, str(e))
    
    async def pause_task(self, task_id: str) -> ServiceResponse:
        """暂停任务"""
        if self._node is None:
            return ServiceResponse(False, "ROS2 client not initialized")
        
        try:
            from qyh_task_engine_msgs.srv import PauseTask
            
            client = self._service_clients.get('pause_task')
            if not client or not client.wait_for_service(timeout_sec=2.0):
                return ServiceResponse(False, "任务引擎服务不可用")
            
            request = PauseTask.Request()
            request.task_id = task_id
            
            future = client.call_async(request)
            result = await self._wait_for_future(future, timeout=5.0)
            
            if result is not None:
                return ServiceResponse(result.success, result.message)
            else:
                return ServiceResponse(False, "服务调用超时")
                
        except Exception as e:
            logger.error(f"pause_task error: {e}")
            return ServiceResponse(False, str(e))
    
    async def resume_task(self, task_id: str) -> ServiceResponse:
        """恢复任务"""
        if self._node is None:
            return ServiceResponse(False, "ROS2 client not initialized")
        
        try:
            from qyh_task_engine_msgs.srv import ResumeTask
            
            client = self._service_clients.get('resume_task')
            if not client or not client.wait_for_service(timeout_sec=2.0):
                return ServiceResponse(False, "任务引擎服务不可用")
            
            request = ResumeTask.Request()
            request.task_id = task_id
            
            future = client.call_async(request)
            result = await self._wait_for_future(future, timeout=5.0)
            
            if result is not None:
                return ServiceResponse(result.success, result.message)
            else:
                return ServiceResponse(False, "服务调用超时")
                
        except Exception as e:
            logger.error(f"resume_task error: {e}")
            return ServiceResponse(False, str(e))
    
    # ==================== 关机服务 ====================
    
    async def request_shutdown(self) -> ServiceResponse:
        """
        请求系统关机
        
        调用 qyh_shutdown_node 的 /control 服务触发关机流程
        """
        if self._node is None:
            return ServiceResponse(False, "ROS2 client not initialized")
        
        try:
            from std_srvs.srv import Trigger
            
            client = self._service_clients.get('shutdown')
            if not client or not client.wait_for_service(timeout_sec=2.0):
                return ServiceResponse(False, "关机服务不可用")
            
            request = Trigger.Request()
            
            future = client.call_async(request)
            result = await self._wait_for_future(future, timeout=5.0)
            
            if result is not None:
                return ServiceResponse(result.success, result.message)
            else:
                return ServiceResponse(False, "服务调用超时")
                
        except Exception as e:
            logger.error(f"request_shutdown error: {e}")
            return ServiceResponse(False, str(e))
    
    async def request_reboot(self) -> ServiceResponse:
        """
        请求系统重启
        
        注意：当前 qyh_shutdown 节点只支持关机，重启需要额外实现
        """
        if self._node is None:
            return ServiceResponse(False, "ROS2 client not initialized")
        
        # TODO: 实现重启功能（需要 qyh_shutdown 节点支持）
        return ServiceResponse(False, "重启功能暂未实现")
    
    # ==================== 紧急停止 ====================
    
    async def publish_emergency_stop(self) -> None:
        """
        发布紧急停止命令
        
        通过发布零速度到 /cmd_vel 来停止机器人运动
        这是一个后备方案，正常情况下应该通过 Data Plane WebSocket 发送
        """
        if self._node is None or self._emergency_stop_publisher is None:
            raise RuntimeError("ROS2 client not initialized")
        
        try:
            from geometry_msgs.msg import Twist
            
            # 发布零速度
            msg = Twist()
            msg.linear.x = 0.0
            msg.linear.y = 0.0
            msg.linear.z = 0.0
            msg.angular.x = 0.0
            msg.angular.y = 0.0
            msg.angular.z = 0.0
            
            self._emergency_stop_publisher.publish(msg)
            logger.warning("Emergency stop command published to /cmd_vel")
            
        except Exception as e:
            logger.error(f"Failed to publish emergency stop: {e}")
            raise
    
    async def release_emergency_stop(self) -> None:
        """
        解除紧急停止状态
        
        注意：此方法当前仅记录日志，实际的急停解除需要硬件层面确认
        """
        logger.info("Emergency stop release requested")
        # TODO: 如果有专门的急停解除服务，在这里调用
    
    # ==================== 机械臂服务 ====================
    
    async def arm_move_j(
        self,
        joint_positions: list[float],
        robot_id: RobotSide = RobotSide.DUAL,
        velocity: float = 0.5,
        acceleration: float = 0.3,
        is_block: bool = True,
    ) -> ServiceResponse:
        """
        机械臂关节运动
        
        Args:
            joint_positions: 14个关节角度 [left_arm(7), right_arm(7)]，单位弧度
            robot_id: 机械臂选择（LEFT/RIGHT/DUAL）
            velocity: 关节速度 (rad/s)
            acceleration: 关节加速度 (rad/s^2)
            is_block: 是否阻塞等待完成
            
        Returns:
            ServiceResponse: 执行结果
        """
        if self._node is None:
            return ServiceResponse(False, "ROS2 client not initialized")
        
        try:
            from qyh_jaka_control_msgs.srv import MoveJ
            
            client = self._service_clients.get('arm_move_j')
            if not client or not client.wait_for_service(timeout_sec=2.0):
                return ServiceResponse(False, "机械臂控制服务不可用")
            
            request = MoveJ.Request()
            request.robot_id = int(robot_id)
            request.joint_positions = joint_positions
            request.move_mode = False  # ABS mode
            request.velocity = velocity
            request.acceleration = acceleration
            request.is_block = is_block
            
            future = client.call_async(request)
            timeout = 30.0 if is_block else 5.0
            result = await self._wait_for_future(future, timeout=timeout)
            
            if result is not None:
                return ServiceResponse(result.success, result.message)
            else:
                return ServiceResponse(False, "服务调用超时")
                
        except Exception as e:
            logger.error(f"arm_move_j error: {e}")
            return ServiceResponse(False, str(e))
    
    # ==================== 升降服务 ====================
    
    async def lift_go_position(
        self,
        position: float,
        speed: float = 100.0,
    ) -> ServiceResponse:
        """
        升降机构移动到指定位置
        
        Args:
            position: 目标位置（单位根据实际配置）
            speed: 运动速度
            
        Returns:
            ServiceResponse: 执行结果
        """
        if self._node is None:
            return ServiceResponse(False, "ROS2 client not initialized")
        
        try:
            from qyh_lift_msgs.srv import LiftControl
            
            client = self._service_clients.get('lift_control')
            if not client or not client.wait_for_service(timeout_sec=2.0):
                return ServiceResponse(False, "升降控制服务不可用")
            
            # 先设置速度
            speed_request = LiftControl.Request()
            speed_request.command = LiftCommand.SET_SPEED
            speed_request.value = speed
            speed_request.hold = False
            
            future = client.call_async(speed_request)
            await self._wait_for_future(future, timeout=2.0)
            
            # 再执行运动
            move_request = LiftControl.Request()
            move_request.command = LiftCommand.GO_POSITION
            move_request.value = position
            move_request.hold = False
            
            future = client.call_async(move_request)
            result = await self._wait_for_future(future, timeout=30.0)
            
            if result is not None:
                return ServiceResponse(result.success, result.message)
            else:
                return ServiceResponse(False, "服务调用超时")
                
        except Exception as e:
            logger.error(f"lift_go_position error: {e}")
            return ServiceResponse(False, str(e))
    
    # ==================== 腰部服务 ====================
    
    async def waist_go_angle(
        self,
        angle: float,
        speed: float = 50.0,
    ) -> ServiceResponse:
        """
        腰部移动到指定角度
        
        Args:
            angle: 目标角度 (0~45度，0为竖直)
            speed: 运动速度
            
        Returns:
            ServiceResponse: 执行结果
        """
        if self._node is None:
            return ServiceResponse(False, "ROS2 client not initialized")
        
        try:
            from qyh_waist_msgs.srv import WaistControl
            
            client = self._service_clients.get('waist_control')
            if not client or not client.wait_for_service(timeout_sec=2.0):
                return ServiceResponse(False, "腰部控制服务不可用")
            
            # 先设置速度
            speed_request = WaistControl.Request()
            speed_request.command = WaistCommand.SET_SPEED
            speed_request.value = speed
            speed_request.hold = False
            
            future = client.call_async(speed_request)
            await self._wait_for_future(future, timeout=2.0)
            
            # 再执行运动
            move_request = WaistControl.Request()
            move_request.command = WaistCommand.GO_ANGLE
            move_request.value = angle
            move_request.hold = False
            
            future = client.call_async(move_request)
            result = await self._wait_for_future(future, timeout=30.0)
            
            if result is not None:
                return ServiceResponse(result.success, result.message)
            else:
                return ServiceResponse(False, "服务调用超时")
                
        except Exception as e:
            logger.error(f"waist_go_angle error: {e}")
            return ServiceResponse(False, str(e))
    
    # ==================== 夹爪服务 ====================
    
    async def gripper_move(
        self,
        side: str,
        position: int,
        speed: int = 100,
        force: int = 50,
    ) -> ServiceResponse:
        """
        夹爪移动
        
        Args:
            side: "left" 或 "right"
            position: 位置 (0=全开, 255=全闭)
            speed: 速度 (0-255)
            force: 力限制 (0-255)
            
        Returns:
            ServiceResponse: 执行结果
        """
        if self._node is None:
            return ServiceResponse(False, "ROS2 client not initialized")
        
        try:
            from qyh_gripper_msgs.srv import MoveGripper
            
            client_key = f'gripper_{side}'
            client = self._service_clients.get(client_key)
            if not client or not client.wait_for_service(timeout_sec=2.0):
                return ServiceResponse(False, f"{side}夹爪控制服务不可用")
            
            request = MoveGripper.Request()
            request.position = position
            request.speed = speed
            request.force = force
            
            future = client.call_async(request)
            result = await self._wait_for_future(future, timeout=5.0)

            if result is not None:
                return ServiceResponse(result.success, result.message)
            else:
                return ServiceResponse(False, "服务调用超时")
                
        except Exception as e:
            logger.error(f"gripper_move error: {e}")
            return ServiceResponse(False, str(e))
    
    # ==================== 头部服务 ====================
    
    async def head_enable_torque(self, enable: bool) -> ServiceResponse:
        """
        头部电机扭矩使能
        
        Args:
            enable: True=使能, False=失能
            
        Returns:
            ServiceResponse: 执行结果
        """
        if self._node is None:
            return ServiceResponse(False, "ROS2 client not initialized")
        
        try:
            from std_srvs.srv import SetBool
            
            client = self._service_clients.get('head_enable_torque')
            if not client or not client.wait_for_service(timeout_sec=2.0):
                return ServiceResponse(False, "头部控制服务不可用")
            
            request = SetBool.Request()
            request.data = enable
            
            future = client.call_async(request)
            result = await self._wait_for_future(future, timeout=2.0)
            
            if result is not None:
                return ServiceResponse(result.success, result.message)
            else:
                return ServiceResponse(False, "服务调用超时")
                
        except Exception as e:
            logger.error(f"head_enable_torque error: {e}")
            return ServiceResponse(False, str(e))
    
    # ==================== 连接检查 ====================
    
    async def check_ros2_connection(self) -> bool:
        """
        检查 ROS2 连接状态
        
        Returns:
            bool: 是否连接正常
        """
        try:
            # 检查节点是否存在
            if self._node is None:
                return False
            
            # 尝试获取节点列表
            node_names = self._node.get_node_names()
            return len(node_names) > 0
            
        except Exception as e:
            logger.error(f"check_ros2_connection error: {e}")
            return False
    
    async def check_service_available(self, service_name: str) -> bool:
        """
        检查指定服务是否可用
        
        Args:
            service_name: 服务键名（如 'start_recording'）
            
        Returns:
            bool: 服务是否可用
        """
        if self._node is None:
            return False
        client = self._service_clients.get(service_name)
        if client is None:
            return False
        
        return client.wait_for_service(timeout_sec=1.0)


# ==================== 全局实例 ====================

# 获取全局客户端实例
def get_ros2_client() -> ROS2ServiceClient:
    """获取 ROS2 服务客户端单例"""
    if not ROS2_AVAILABLE:
        raise RuntimeError("ROS2 (rclpy) not available")
    return ROS2ServiceClient()


# FastAPI 依赖注入
async def get_ros2_client_dependency() -> ROS2ServiceClient:
    """FastAPI 依赖：获取 ROS2 客户端"""
    client = get_ros2_client()
    if not client._initialized:
        await client.initialize()
    return client
