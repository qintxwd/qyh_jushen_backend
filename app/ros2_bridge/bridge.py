"""ROS2 Bridge - 多线程隔离的 ROS2 集成 (组件化版本)"""
import threading
import time
import json
import rclpy
from queue import Queue
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from typing import Optional, Dict, Any, List

from app.config import settings

# 引入组件
from .components.base import BridgeComponent
from .components.chassis import ChassisComponent
from .components.arm import ArmComponent
from .components.gripper import GripperComponent
from .components.waist import WaistComponent
from .components.lift import LiftComponent
from .components.head import HeadComponent
from .components.camera import CameraComponent
from .components.task_engine import TaskEngineComponent
from .components.vr import VRComponent

class ROS2Bridge:
    """ROS2 桥接器（聚合组件）"""
    
    _instance = None
    _lock = threading.Lock()
    
    def __new__(cls, *args, **kwargs):
        if not cls._instance:
            with cls._lock:
                if not cls._instance:
                    cls._instance = super(ROS2Bridge, cls).__new__(cls)
        return cls._instance

    def __init__(self):
        if hasattr(self, 'initialized'):
            return
            
        self.node = None
        self.executor = None
        self.thread: Optional[threading.Thread] = None
        self.p_thread: Optional[threading.Thread] = None
        self.running = False
        
        # 线程安全的数据队列
        self.command_queue = Queue()
        
        # 关机状态 (保留在主 Bridge 中，因为涉及系统级操作)
        self._shutdown_client = None
        self._shutdown_state = {
            "shutdown_in_progress": False,
            "trigger_source": 0,
            "trigger_source_text": "",
            "countdown_seconds": -1,
            "plc_connected": False
        }
        
        # LED 发布器 (保留在主 Bridge 中)
        self.led_color_pub = None
        self.led_blink_pub = None

        # 组件初始化
        self.chassis = ChassisComponent()
        self.arm = ArmComponent()
        self.gripper = GripperComponent()
        self.waist = WaistComponent()
        self.lift = LiftComponent()
        self.head = HeadComponent()
        self.camera = CameraComponent()
        self.task = TaskEngineComponent()
        self.vr = VRComponent()
        
        self.components: List[BridgeComponent] = [
            self.chassis, self.arm, self.gripper, 
            self.waist, self.lift, self.head, 
            self.camera, self.task, self.vr
        ]
        
        self.initialized = True

    # ==================== 属性映射 (兼容旧 API) ====================
    @property
    def chassis_status(self): return self.chassis.chassis_status
    @property
    def navigation_status(self): return self.chassis.navigation_status
    @property
    def arm_state(self): return self.arm.arm_state
    @property
    def lift_state(self): return self.lift.lift_state
    @property
    def waist_state(self): return self.waist.waist_state
    @property
    def head_state(self): return self.head.head_state
    @property
    def left_gripper_state(self): return self.gripper.left_gripper_state
    @property
    def right_gripper_state(self): return self.gripper.right_gripper_state
    @property
    def vr_state(self): return self.vr.vr_state
    @property
    def task_status(self): return self.task.task_status
    @property
    def joint_states_for_3d(self): return self.arm.get_joint_states()
    @property
    def robot_description(self): return self.arm.get_robot_description()

    # ==================== 生命周期管理 ====================
    def start(self):
        """启动 ROS2 Bridge"""
        if self.running:
            return
            
        print("🚀 正在启动 ROS2 Bridge (Component Mode)...")
        
        try:
            rclpy.init(args=None)
            self.node = rclpy.create_node('ros2_bridge_node')
            
            # 初始化所有组件
            for component in self.components:
                component.initialize(self.node, self.command_queue)
                component.setup_subscribers()
                component.setup_publishers()
            
            # 设置系统级订阅/发布
            self._setup_system_subscribers()
            self._setup_system_publishers()
            
            self.executor = MultiThreadedExecutor()
            self.executor.add_node(self.node)
            
            self.running = True
            
            # 启动 ROS2 线程
            self.thread = threading.Thread(target=self._run_ros2, daemon=True)
            self.thread.start()
            
            # 启动命令处理线程
            self.p_thread = threading.Thread(target=self._process_commands, daemon=True)
            self.p_thread.start()
            
            print("✅ ROS2 Bridge 启动成功")
            
        except Exception as e:
            print(f"❌ ROS2 Bridge 启动失败: {e}")
            self.running = False

    def shutdown(self):
        """关闭 ROS2 Bridge"""
        self.running = False
        if self.executor:
            self.executor.shutdown()
        if self.node:
            self.node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        print("🛑 ROS2 Bridge 已关闭")

    def _run_ros2(self):
        """ROS2 运行循环"""
        if self.executor:
            self.executor.spin()

    def _setup_system_subscribers(self):
        """设置系统级订阅器 (如关机状态)"""
        try:
            from qyh_system_msgs.msg import ShutdownState
            
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
            
            # 关机客户端
            from qyh_system_msgs.srv import Shutdown
            self._shutdown_client = self.node.create_client(Shutdown, '/system/shutdown')
            
        except Exception as e:
            print(f"⚠️  系统订阅器创建失败: {e}")

    def _setup_system_publishers(self):
        """设置系统级发布器 (如LED)"""
        try:
            from std_msgs.msg import ColorRGBA, String
            self.led_color_pub = self.node.create_publisher(ColorRGBA, '/robot_led/set_color', 10)
            self.led_blink_pub = self.node.create_publisher(String, '/robot_led/blink', 10)
            print("✅ LED发布器创建成功: /robot_led/set_color, /robot_led/blink")
        except Exception as e:
            print(f"⚠️  LED发布器创建失败: {e}")

    # ==================== 命令处理 ====================
    def handle_command(self, cmd: Dict[str, Any]):
        """接收命令并放入队列"""
        if not self.running:
            return
        
        # 预处理：回调函数无法序列化，这里假设 cmd 是纯数据或回调在 execute 中处理
        # 实际架构中，handle_command 只是入队。
        self.command_queue.put(cmd)

    def _process_commands(self):
        """命令处理循环"""
        while self.running:
            try:
                cmd = self.command_queue.get(timeout=0.1)
                self._execute_command(cmd)
            except Exception:
                continue

    def _execute_command(self, cmd: Dict[str, Any]):
        """执行命令的分发"""
        try:
            cmd_type = cmd.get("type", "")
            
            # 1. 优先处理系统级命令
            if cmd_type == "shutdown":
                self._handle_shutdown(cmd)
                return
            elif cmd_type == "reboot":
                self._handle_reboot(cmd)
                return
            elif cmd_type.startswith("led_"):
                self._handle_led(cmd)
                return
            
            # 2. 分发给组件
            handled = False
            for component in self.components:
                if component.handle_command(cmd):
                    handled = True
                    break
            
            if not handled and cmd_type != "ping":
                if "callback" in cmd and callable(cmd["callback"]):
                    try:
                        cmd["callback"](False)
                    except:
                        pass
                        
        except Exception as e:
            print(f"❌ 执行命令出错: {e}")

    # ==================== 系统命令处理 ====================
    def _handle_shutdown(self, cmd: Dict[str, Any]):
        cb = cmd.get("callback")
        try:
            if self._shutdown_client and self._shutdown_client.service_is_ready():
                from qyh_system_msgs.srv import Shutdown
                req = Shutdown.Request()
                req.action = 1 # SHUTDOWN
                req.force = cmd.get("force", False)
                self._shutdown_client.call_async(req)
                if cb: cb(True)
            else:
                if cb: cb(False)
        except Exception:
            if cb: cb(False)

    def _handle_reboot(self, cmd: Dict[str, Any]):
        cb = cmd.get("callback")
        try:
            if self._shutdown_client and self._shutdown_client.service_is_ready():
                from qyh_system_msgs.srv import Shutdown
                req = Shutdown.Request()
                req.action = 2 # REBOOT
                req.force = cmd.get("force", False)
                self._shutdown_client.call_async(req)
                if cb: cb(True)
            else:
                if cb: cb(False)
        except Exception:
            if cb: cb(False)

    def _handle_led(self, cmd: Dict[str, Any]):
        cmd_type = cmd.get("type")
        cb = cmd.get("callback")
        try:
            if cmd_type == "led_set_color":
                from std_msgs.msg import ColorRGBA
                msg = ColorRGBA()
                msg.r = float(cmd.get("r", 0))
                msg.g = float(cmd.get("g", 0))
                msg.b = float(cmd.get("b", 0))
                msg.a = float(cmd.get("a", 1))
                if self.led_color_pub:
                    self.led_color_pub.publish(msg)
                    if cb: cb(True)
            elif cmd_type == "led_blink":
                from std_msgs.msg import String
                msg = String()
                msg.data = str(cmd.get("pattern", "default"))
                if self.led_blink_pub:
                    self.led_blink_pub.publish(msg)
                    if cb: cb(True)
        except Exception:
            if cb: cb(False)

    # ==================== 公共接口封装 (保持兼容性) ====================
    def get_chassis_status(self) -> Optional[Dict[str, Any]]:
        return self.chassis.get_status()
        
    def get_navigation_status(self) -> Optional[Dict[str, Any]]:
        return self.chassis.get_navigation_status()

    def get_camera_status(self, camera_id: str) -> Optional[Dict[str, Any]]:
        return self.camera.get_camera_status(camera_id)
        
    def get_all_camera_status(self) -> Dict[str, Dict[str, Any]]:
        return self.camera.get_all_camera_status()
        
    def get_available_cameras(self) -> List[str]:
        return self.camera.get_available_cameras()

    # 其他 helper 方法
    def _get_system_status_text(self, status: int) -> str:
        return "N/A"

# Global instance
ros2_bridge = ROS2Bridge()
