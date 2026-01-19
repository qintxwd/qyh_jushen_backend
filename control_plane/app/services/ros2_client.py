"""
QYH Jushen Control Plane - ROS2 服务客户端

提供与 ROS2 节点通信的统一接口。
在非 ROS2 环境下自动降级为 Mock 模式。

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
import logging
import os
from typing import Optional, Any
from dataclasses import dataclass, field
from enum import IntEnum

logger = logging.getLogger(__name__)

# ==================== ROS2 可用性检测 ====================

ROS2_AVAILABLE = False
rclpy = None

try:
    import rclpy
    from rclpy.node import Node
    ROS2_AVAILABLE = True
    logger.info("ROS2 (rclpy) is available")
except ImportError:
    logger.warning("ROS2 (rclpy) not available, using mock mode")


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
    
    在 ROS2 环境可用时使用真实服务调用，
    否则使用 Mock 实现用于开发/测试。
    
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
        
        self._node = None
        self._mock_mode = not ROS2_AVAILABLE
        self._service_clients = {}
        
        # Mock 状态
        self._mock_recording_status = RecordingStatus()
        self._mock_shutdown_in_progress = False
        
        self._initialized = True
    
    async def initialize(self) -> bool:
        """
        初始化 ROS2 节点和服务客户端
        
        Returns:
            bool: 是否成功初始化（Mock 模式下始终返回 True）
        """
        if self._mock_mode:
            logger.info("ROS2 client initialized in MOCK mode")
            return True
        
        try:
            if not rclpy.ok():
                rclpy.init()
            
            self._node = rclpy.create_node('control_plane_client')
            
            # 创建服务客户端
            await self._create_service_clients()
            
            logger.info("ROS2 client initialized successfully")
            return True
            
        except Exception as e:
            logger.error(f"Failed to initialize ROS2 client: {e}")
            self._mock_mode = True
            return False
    
    async def _create_service_clients(self):
        """创建所有服务客户端"""
        if self._mock_mode or self._node is None:
            return
        
        try:
            # 动态导入服务类型（避免在非 ROS2 环境报错）
            from qyh_bag_recorder.srv import StartRecording, StopRecording, GetRecordingStatus
            from qyh_task_engine_msgs.srv import ExecuteTask, CancelTask, PauseTask, ResumeTask
            from std_srvs.srv import Trigger as ShutdownTrigger
            from qyh_jaka_control_msgs.srv import MoveJ
            from qyh_lift_msgs.srv import LiftControl
            from qyh_waist_msgs.srv import WaistControl
            from qyh_gripper_msgs.srv import MoveGripper
            from std_srvs.srv import SetBool
            
            # 录制服务
            self._service_clients['start_recording'] = self._node.create_client(
                StartRecording, '/bag_recorder/start_recording'
            )
            self._service_clients['stop_recording'] = self._node.create_client(
                StopRecording, '/bag_recorder/stop_recording'
            )
            self._service_clients['get_recording_status'] = self._node.create_client(
                GetRecordingStatus, '/bag_recorder/get_status'
            )
            
            # 任务服务
            self._service_clients['execute_task'] = self._node.create_client(
                ExecuteTask, '/task_engine/execute_task'
            )
            self._service_clients['cancel_task'] = self._node.create_client(
                CancelTask, '/task_engine/cancel_task'
            )
            self._service_clients['pause_task'] = self._node.create_client(
                PauseTask, '/task_engine/pause_task'
            )
            self._service_clients['resume_task'] = self._node.create_client(
                ResumeTask, '/task_engine/resume_task'
            )
            
            # 关机服务
            self._service_clients['shutdown'] = self._node.create_client(
                ShutdownTrigger, '/qyh_shutdown_node/control'
            )
            
            # 机械臂服务
            self._service_clients['arm_move_j'] = self._node.create_client(
                MoveJ, '/jaka_control/move_j'
            )
            
            # 升降服务
            self._service_clients['lift_control'] = self._node.create_client(
                LiftControl, '/lift_control/control'
            )
            
            # 腰部服务
            self._service_clients['waist_control'] = self._node.create_client(
                WaistControl, '/waist_control/control'
            )
            
            # 夹爪服务
            self._service_clients['gripper_left'] = self._node.create_client(
                MoveGripper, '/gripper_left/move'
            )
            self._service_clients['gripper_right'] = self._node.create_client(
                MoveGripper, '/gripper_right/move'
            )
            
            # 头部服务
            self._service_clients['head_enable_torque'] = self._node.create_client(
                SetBool, '/head_motor_node/enable_torque'
            )
            
            logger.info(f"Created {len(self._service_clients)} ROS2 service clients")
            
        except ImportError as e:
            logger.warning(f"Some ROS2 message types not available: {e}")
    
    async def shutdown(self):
        """关闭 ROS2 节点"""
        if self._node is not None:
            self._node.destroy_node()
            self._node = None
        
        if ROS2_AVAILABLE and rclpy.ok():
            rclpy.shutdown()
        
        logger.info("ROS2 client shutdown")
    
    @property
    def is_mock_mode(self) -> bool:
        """是否为 Mock 模式"""
        return self._mock_mode
    
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
        if self._mock_mode:
            self._mock_recording_status = RecordingStatus(
                is_recording=True,
                action_name=action_name,
                duration_sec=0.0,
                bag_path=f"~/qyh-robot-system/model_actions/{action_name}/data/bags/episode_001",
                topics=topics,
            )
            logger.info(f"[MOCK] Started recording: {action_name}")
            return ServiceResponse(
                success=True,
                message="录制已开始 (Mock)",
                data={"bag_path": self._mock_recording_status.bag_path}
            )
        
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
            response = await asyncio.get_event_loop().run_in_executor(
                None, lambda: rclpy.spin_until_future_complete(self._node, future, timeout_sec=5.0)
            )
            
            if future.done():
                result = future.result()
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
        if self._mock_mode:
            if not self._mock_recording_status.is_recording:
                return ServiceResponse(False, "当前没有在录制")
            
            result = ServiceResponse(
                success=True,
                message="录制已停止 (Mock)",
                data={
                    "duration_sec": 10.5,
                    "bag_path": self._mock_recording_status.bag_path
                }
            )
            self._mock_recording_status = RecordingStatus()
            logger.info("[MOCK] Stopped recording")
            return result
        
        try:
            from qyh_bag_recorder.srv import StopRecording
            
            client = self._service_clients.get('stop_recording')
            if not client or not client.wait_for_service(timeout_sec=2.0):
                return ServiceResponse(False, "录制服务不可用")
            
            request = StopRecording.Request()
            
            future = client.call_async(request)
            await asyncio.get_event_loop().run_in_executor(
                None, lambda: rclpy.spin_until_future_complete(self._node, future, timeout_sec=10.0)
            )
            
            if future.done():
                result = future.result()
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
        """
        获取录制状态
        
        Returns:
            RecordingStatus: 录制状态信息
        """
        if self._mock_mode:
            return self._mock_recording_status
        
        try:
            from qyh_bag_recorder.srv import GetRecordingStatus
            
            client = self._service_clients.get('get_recording_status')
            if not client or not client.wait_for_service(timeout_sec=2.0):
                return RecordingStatus()
            
            request = GetRecordingStatus.Request()
            
            future = client.call_async(request)
            await asyncio.get_event_loop().run_in_executor(
                None, lambda: rclpy.spin_until_future_complete(self._node, future, timeout_sec=2.0)
            )
            
            if future.done():
                result = future.result()
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
        if self._mock_mode:
            import uuid
            task_id = str(uuid.uuid4())[:8]
            logger.info(f"[MOCK] Execute task: {task_id}")
            return ServiceResponse(
                success=True,
                message="任务已提交 (Mock)",
                data={"task_id": task_id}
            )
        
        try:
            from qyh_task_engine_msgs.srv import ExecuteTask
            
            client = self._service_clients.get('execute_task')
            if not client or not client.wait_for_service(timeout_sec=2.0):
                return ServiceResponse(False, "任务引擎服务不可用")
            
            request = ExecuteTask.Request()
            request.task_json = task_json
            request.debug_mode = debug_mode
            
            future = client.call_async(request)
            await asyncio.get_event_loop().run_in_executor(
                None, lambda: rclpy.spin_until_future_complete(self._node, future, timeout_sec=5.0)
            )
            
            if future.done():
                result = future.result()
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
        if self._mock_mode:
            logger.info(f"[MOCK] Cancel task: {task_id}")
            return ServiceResponse(True, "任务已取消 (Mock)")
        
        try:
            from qyh_task_engine_msgs.srv import CancelTask
            
            client = self._service_clients.get('cancel_task')
            if not client or not client.wait_for_service(timeout_sec=2.0):
                return ServiceResponse(False, "任务引擎服务不可用")
            
            request = CancelTask.Request()
            request.task_id = task_id
            
            future = client.call_async(request)
            await asyncio.get_event_loop().run_in_executor(
                None, lambda: rclpy.spin_until_future_complete(self._node, future, timeout_sec=5.0)
            )
            
            if future.done():
                result = future.result()
                return ServiceResponse(result.success, result.message)
            else:
                return ServiceResponse(False, "服务调用超时")
                
        except Exception as e:
            logger.error(f"cancel_task error: {e}")
            return ServiceResponse(False, str(e))
    
    async def pause_task(self, task_id: str) -> ServiceResponse:
        """暂停任务"""
        if self._mock_mode:
            logger.info(f"[MOCK] Pause task: {task_id}")
            return ServiceResponse(True, "任务已暂停 (Mock)")
        
        try:
            from qyh_task_engine_msgs.srv import PauseTask
            
            client = self._service_clients.get('pause_task')
            if not client or not client.wait_for_service(timeout_sec=2.0):
                return ServiceResponse(False, "任务引擎服务不可用")
            
            request = PauseTask.Request()
            request.task_id = task_id
            
            future = client.call_async(request)
            await asyncio.get_event_loop().run_in_executor(
                None, lambda: rclpy.spin_until_future_complete(self._node, future, timeout_sec=5.0)
            )
            
            if future.done():
                result = future.result()
                return ServiceResponse(result.success, result.message)
            else:
                return ServiceResponse(False, "服务调用超时")
                
        except Exception as e:
            logger.error(f"pause_task error: {e}")
            return ServiceResponse(False, str(e))
    
    async def resume_task(self, task_id: str) -> ServiceResponse:
        """恢复任务"""
        if self._mock_mode:
            logger.info(f"[MOCK] Resume task: {task_id}")
            return ServiceResponse(True, "任务已恢复 (Mock)")
        
        try:
            from qyh_task_engine_msgs.srv import ResumeTask
            
            client = self._service_clients.get('resume_task')
            if not client or not client.wait_for_service(timeout_sec=2.0):
                return ServiceResponse(False, "任务引擎服务不可用")
            
            request = ResumeTask.Request()
            request.task_id = task_id
            
            future = client.call_async(request)
            await asyncio.get_event_loop().run_in_executor(
                None, lambda: rclpy.spin_until_future_complete(self._node, future, timeout_sec=5.0)
            )
            
            if future.done():
                result = future.result()
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
        if self._mock_mode:
            self._mock_shutdown_in_progress = True
            logger.info("[MOCK] System shutdown requested")
            return ServiceResponse(True, "关机请求已发送 (Mock)")
        
        try:
            from std_srvs.srv import Trigger
            
            client = self._service_clients.get('shutdown')
            if not client or not client.wait_for_service(timeout_sec=2.0):
                return ServiceResponse(False, "关机服务不可用")
            
            request = Trigger.Request()
            
            future = client.call_async(request)
            await asyncio.get_event_loop().run_in_executor(
                None, lambda: rclpy.spin_until_future_complete(self._node, future, timeout_sec=5.0)
            )
            
            if future.done():
                result = future.result()
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
        if self._mock_mode:
            logger.info("[MOCK] System reboot requested")
            return ServiceResponse(True, "重启请求已发送 (Mock)")
        
        # TODO: 实现重启功能（需要 qyh_shutdown 节点支持）
        return ServiceResponse(False, "重启功能暂未实现")
    
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
        if self._mock_mode:
            logger.info(f"[MOCK] Arm MoveJ: robot_id={robot_id}, v={velocity}")
            return ServiceResponse(True, "机械臂运动已执行 (Mock)")
        
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
            await asyncio.get_event_loop().run_in_executor(
                None, lambda: rclpy.spin_until_future_complete(self._node, future, timeout_sec=timeout)
            )
            
            if future.done():
                result = future.result()
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
        if self._mock_mode:
            logger.info(f"[MOCK] Lift go to position: {position}")
            return ServiceResponse(True, "升降运动已执行 (Mock)")
        
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
            await asyncio.get_event_loop().run_in_executor(
                None, lambda: rclpy.spin_until_future_complete(self._node, future, timeout_sec=2.0)
            )
            
            # 再执行运动
            move_request = LiftControl.Request()
            move_request.command = LiftCommand.GO_POSITION
            move_request.value = position
            move_request.hold = False
            
            future = client.call_async(move_request)
            await asyncio.get_event_loop().run_in_executor(
                None, lambda: rclpy.spin_until_future_complete(self._node, future, timeout_sec=30.0)
            )
            
            if future.done():
                result = future.result()
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
        if self._mock_mode:
            logger.info(f"[MOCK] Waist go to angle: {angle}")
            return ServiceResponse(True, "腰部运动已执行 (Mock)")
        
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
            await asyncio.get_event_loop().run_in_executor(
                None, lambda: rclpy.spin_until_future_complete(self._node, future, timeout_sec=2.0)
            )
            
            # 再执行运动
            move_request = WaistControl.Request()
            move_request.command = WaistCommand.GO_ANGLE
            move_request.value = angle
            move_request.hold = False
            
            future = client.call_async(move_request)
            await asyncio.get_event_loop().run_in_executor(
                None, lambda: rclpy.spin_until_future_complete(self._node, future, timeout_sec=30.0)
            )
            
            if future.done():
                result = future.result()
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
        if self._mock_mode:
            logger.info(f"[MOCK] Gripper {side} move to: {position}")
            return ServiceResponse(True, "夹爪运动已执行 (Mock)")
        
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
            await asyncio.get_event_loop().run_in_executor(
                None, lambda: rclpy.spin_until_future_complete(self._node, future, timeout_sec=5.0)
            )
            
            if future.done():
                result = future.result()
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
        if self._mock_mode:
            logger.info(f"[MOCK] Head torque enable: {enable}")
            return ServiceResponse(True, f"头部扭矩{'使能' if enable else '失能'}成功 (Mock)")
        
        try:
            from std_srvs.srv import SetBool
            
            client = self._service_clients.get('head_enable_torque')
            if not client or not client.wait_for_service(timeout_sec=2.0):
                return ServiceResponse(False, "头部控制服务不可用")
            
            request = SetBool.Request()
            request.data = enable
            
            future = client.call_async(request)
            await asyncio.get_event_loop().run_in_executor(
                None, lambda: rclpy.spin_until_future_complete(self._node, future, timeout_sec=2.0)
            )
            
            if future.done():
                result = future.result()
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
        if self._mock_mode:
            return False
        
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
        if self._mock_mode:
            return True  # Mock 模式下所有服务都"可用"
        
        client = self._service_clients.get(service_name)
        if client is None:
            return False
        
        return client.wait_for_service(timeout_sec=1.0)


# ==================== 全局实例 ====================

# 获取全局客户端实例
def get_ros2_client() -> ROS2ServiceClient:
    """获取 ROS2 服务客户端单例"""
    return ROS2ServiceClient()


# FastAPI 依赖注入
async def get_ros2_client_dependency() -> ROS2ServiceClient:
    """FastAPI 依赖：获取 ROS2 客户端"""
    client = get_ros2_client()
    if not client._initialized:
        await client.initialize()
    return client
