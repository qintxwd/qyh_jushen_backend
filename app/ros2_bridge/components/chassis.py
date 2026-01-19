"""底盘控制组件 - 管理底盘状态、手动控制、导航等功能"""
import time
import math
import json
from pathlib import Path
from typing import Optional, Dict, Any
from .base import BridgeComponent


class ChassisComponent(BridgeComponent):
    """底盘控制组件
    
    功能:
    - 底盘状态订阅和缓存
    - 导航状态订阅
    - 手动控制（线圈模式/速度模式）
    - 导航控制
    - 系统参数设置（速度、音量、避障策略等）
    """
    
    def __init__(self):
        super().__init__()
        # 状态缓存
        self.chassis_status: Optional[Dict[str, Any]] = None
        self.chassis_last_update_time: Optional[float] = None
        self.chassis_timeout_seconds: float = 2.0
        self.navigation_status: Optional[Dict[str, Any]] = None
        
        # 发布器
        self.cmd_vel_pub = None
        self.manual_pub = None
        self.velocity_pub = None
        
        # 服务客户端
        self.control_clients: Dict[str, Any] = {}
        self.go_nav_coord_client = None
        self.go_nav_site_client = None
        self.go_nav_site_simple_client = None
        self.go_nav_site_task_client = None
        self.go_force_loc_client = None
        self.go_set_speed_client = None
        self.go_set_obstacle_client = None
        self.go_set_site_client = None
        self.go_set_volume_client = None
        self.go_set_map_client = None
    
    def setup_subscribers(self):
        """设置底盘相关订阅器"""
        self._setup_status_subscriber()
        self._setup_navigation_subscriber()
    
    def _setup_status_subscriber(self):
        """设置底盘状态订阅器"""
        try:
            from qyh_standard_robot_msgs.msg import StandardRobotStatus
            
            def chassis_status_callback(msg: StandardRobotStatus):
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
    
    def _setup_navigation_subscriber(self):
        """设置导航状态订阅器"""
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
    
    def setup_publishers(self):
        """设置底盘发布器和服务客户端"""
        self._setup_velocity_publishers()
        self._setup_control_clients()
        self._setup_go_clients()
    
    def _setup_velocity_publishers(self):
        """设置速度发布器"""
        # cmd_vel 发布器（通用）
        try:
            from geometry_msgs.msg import Twist
            self.cmd_vel_pub = self.node.create_publisher(
                Twist, '/cmd_vel', 10
            )
            print("✅ 底盘速度发布器创建成功: /cmd_vel")
        except Exception as e:
            print(f"⚠️  底盘速度发布器创建失败: {e}")
        
        # 手动控制命令发布器（线圈模式）
        try:
            from qyh_standard_robot_msgs.msg import ManualMotionCommand
            self.manual_pub = self.node.create_publisher(
                ManualMotionCommand, 'manual_motion_cmd', 10
            )
            print("✅ 底盘手动控制发布器创建成功: manual_motion_cmd")
        except Exception as e:
            print(f"⚠️  底盘手动控制发布器创建失败: {e}")
        
        # 手动速度命令发布器（速度模式）
        try:
            from qyh_standard_robot_msgs.msg import ManualVelocityCommand
            self.velocity_pub = self.node.create_publisher(
                ManualVelocityCommand, 'manual_velocity_cmd', 10
            )
            print("✅ 底盘速度命令发布器创建成功: manual_velocity_cmd")
        except Exception as e:
            print(f"⚠️  底盘速度命令发布器创建失败: {e}")
    
    def _setup_control_clients(self):
        """设置控制服务客户端"""
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
            
            self.control_clients = {
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
    
    def _setup_go_clients(self):
        """设置 Go 系列服务客户端"""
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
            
            self.go_nav_coord_client = self.node.create_client(
                GoNavigateToCoordinate, 'go_navigate_to_coordinate')
            self.go_nav_site_client = self.node.create_client(
                GoExecuteActionTask, 'go_navigate_to_site')
            self.go_nav_site_simple_client = self.node.create_client(
                GoNavigateToSite, 'go_navigate_to_site_simple')
            self.go_nav_site_task_client = self.node.create_client(
                GoNavigateToSiteWithTask, 'go_navigate_to_site_with_task')
            self.go_force_loc_client = self.node.create_client(
                GoForceLocalize, 'go_force_localize')
            self.go_set_speed_client = self.node.create_client(
                GoSetSpeedType, 'go_set_speed_level')
            self.go_set_obstacle_client = self.node.create_client(
                GoSetObstacleStrategy, 'go_set_obstacle_strategy')
            self.go_set_site_client = self.node.create_client(
                GoSetCurrentSite, 'go_set_current_site')
            self.go_set_volume_client = self.node.create_client(
                GoSetSpeakerVolume, 'go_set_speaker_volume')
            self.go_set_map_client = self.node.create_client(
                GoSetCurrentMap, 'go_set_current_map')
            print("✅ 底盘 Go 系列服务客户端创建成功")
            
            # 延迟初始化底盘参数
            self.node.create_timer(3.0, self._initialize_chassis_params, one_shot=True)
            
        except Exception as e:
            print(f"⚠️  底盘 Go 系列服务客户端创建失败: {e}")
    
    def _initialize_chassis_params(self):
        """初始化底盘参数（从持久化配置加载）"""
        try:
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
            if self.go_set_speed_client and self.go_set_speed_client.wait_for_service(timeout_sec=1.0):
                from qyh_standard_robot_msgs.srv import GoSetSpeedType
                req = GoSetSpeedType.Request()
                req.speed_level = speed_level
                future = self.go_set_speed_client.call_async(req)
                future.add_done_callback(
                    lambda f: print(f"✅ 速度级别初始化: {f.result().success if not f.exception() else f.exception()}")
                )
            
            # 设置音量
            if self.go_set_volume_client and self.go_set_volume_client.wait_for_service(timeout_sec=1.0):
                from qyh_standard_robot_msgs.srv import GoSetSpeakerVolume
                req = GoSetSpeakerVolume.Request()
                req.volume = volume
                future = self.go_set_volume_client.call_async(req)
                future.add_done_callback(
                    lambda f: print(f"✅ 音量初始化: {f.result().success if not f.exception() else f.exception()}")
                )
                
        except Exception as e:
            print(f"❌ 底盘参数初始化失败: {e}")
    
    # ==================== 命令处理 ====================
    
    def handle_command(self, cmd: Dict[str, Any]) -> bool:
        """处理底盘相关命令"""
        cmd_type = cmd.get('type', '')
        
        handlers = {
            'chassis_velocity': self._handle_velocity,
            'chassis_manual_motion': self._handle_manual_motion,
            'chassis_control_service': self._handle_control_service,
            'chassis_nav_coordinate': self._handle_nav_coordinate,
            'chassis_nav_site': self._handle_nav_site,
            'chassis_nav_site_simple': self._handle_nav_site_simple,
            'chassis_nav_site_task': self._handle_nav_site_task,
            'chassis_force_localize': self._handle_force_localize,
            'chassis_set_speed_level': self._handle_set_speed_level,
            'chassis_set_obstacle_strategy': self._handle_set_obstacle_strategy,
            'chassis_set_current_site': self._handle_set_current_site,
            'chassis_set_volume': self._handle_set_volume,
            'chassis_set_map': self._handle_set_map,
        }
        
        handler = handlers.get(cmd_type)
        if handler:
            handler(cmd)
            return True
        return False
    
    def _handle_velocity(self, cmd: Dict[str, Any]):
        """处理速度命令"""
        try:
            from qyh_standard_robot_msgs.msg import ManualVelocityCommand
            params = cmd['params']
            
            msg = ManualVelocityCommand()
            msg.vx = float(params['linear_x'])
            msg.w = float(params['angular_z'])
            
            if self.velocity_pub:
                self.velocity_pub.publish(msg)
                result = {'success': True, 'message': '速度命令已发送'}
            else:
                result = {'success': False, 'message': '速度发布器未初始化'}
        except Exception as e:
            result = {'success': False, 'message': str(e)}
        
        self._send_result(cmd, result)
    
    def _handle_manual_motion(self, cmd: Dict[str, Any]):
        """处理手动运动命令"""
        try:
            from qyh_standard_robot_msgs.msg import ManualMotionCommand
            params = cmd['params']
            
            msg = ManualMotionCommand()
            msg.forward = params.get('forward', False)
            msg.backward = params.get('backward', False)
            msg.rotate_left = params.get('rotate_left', False)
            msg.rotate_right = params.get('rotate_right', False)
            
            if self.manual_pub:
                self.manual_pub.publish(msg)
                result = {'success': True, 'message': '手动命令已发送'}
            else:
                result = {'success': False, 'message': '手动控制发布器未初始化'}
        except Exception as e:
            result = {'success': False, 'message': str(e)}
        
        self._send_result(cmd, result)
    
    def _handle_control_service(self, cmd: Dict[str, Any]):
        """处理控制服务调用"""
        service_name = cmd['params']['service_name']
        client = self.control_clients.get(service_name)
        
        if not client:
            self._send_result(cmd, {'success': False, 'message': f'未知服务: {service_name}'})
            return
        
        if not client.wait_for_service(timeout_sec=1.0):
            self._send_result(cmd, {'success': False, 'message': f'服务不可用: {service_name}'})
            return
        
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
            self._send_result(cmd, result)
        
        future.add_done_callback(done_callback)
    
    def _handle_nav_coordinate(self, cmd: Dict[str, Any]):
        """处理导航到坐标"""
        if not self.go_nav_coord_client:
            self._send_result(cmd, {'success': False, 'message': '导航服务客户端未初始化'})
            return
        
        if not self.go_nav_coord_client.wait_for_service(timeout_sec=1.0):
            self._send_result(cmd, {'success': False, 'message': '导航服务不可用'})
            return
        
        from qyh_standard_robot_msgs.srv import GoNavigateToCoordinate
        params = cmd['params']
        req = GoNavigateToCoordinate.Request()
        req.x = float(params['x'])
        req.y = float(params['y'])
        req.yaw = float(params['yaw'])
        req.is_localization = params.get('is_localization', False)
        
        future = self.go_nav_coord_client.call_async(req)
        future.add_done_callback(lambda f: self._handle_service_response(cmd, f))
    
    def _handle_nav_site(self, cmd: Dict[str, Any]):
        """处理导航到站点"""
        if not self.go_nav_site_client:
            self._send_result(cmd, {'success': False, 'message': '站点导航服务客户端未初始化'})
            return
        
        if not self.go_nav_site_client.wait_for_service(timeout_sec=1.0):
            self._send_result(cmd, {'success': False, 'message': '站点导航服务不可用'})
            return
        
        from qyh_standard_robot_msgs.srv import GoExecuteActionTask
        params = cmd['params']
        req = GoExecuteActionTask.Request()
        req.site_id = int(params['site_id'])
        req.is_localization = params.get('is_localization', False)
        
        future = self.go_nav_site_client.call_async(req)
        future.add_done_callback(lambda f: self._handle_service_response(cmd, f))
    
    def _handle_nav_site_simple(self, cmd: Dict[str, Any]):
        """处理导航到站点（简单版本）"""
        if not self.go_nav_site_simple_client:
            self._send_result(cmd, {'success': False, 'message': '站点导航服务客户端未初始化'})
            return
        
        if not self.go_nav_site_simple_client.wait_for_service(timeout_sec=1.0):
            self._send_result(cmd, {'success': False, 'message': '站点导航服务不可用'})
            return
        
        from qyh_standard_robot_msgs.srv import GoNavigateToSite
        params = cmd['params']
        req = GoNavigateToSite.Request()
        req.site_id = int(params['site_id'])
        
        future = self.go_nav_site_simple_client.call_async(req)
        future.add_done_callback(lambda f: self._handle_service_response(cmd, f))
    
    def _handle_nav_site_task(self, cmd: Dict[str, Any]):
        """处理导航到站点（带任务ID）"""
        if not self.go_nav_site_task_client:
            self._send_result(cmd, {'success': False, 'message': '站点导航服务客户端未初始化'})
            return
        
        if not self.go_nav_site_task_client.wait_for_service(timeout_sec=1.0):
            self._send_result(cmd, {'success': False, 'message': '站点导航服务不可用'})
            return
        
        from qyh_standard_robot_msgs.srv import GoNavigateToSiteWithTask
        params = cmd['params']
        req = GoNavigateToSiteWithTask.Request()
        req.site_id = int(params['site_id'])
        req.task_id = int(params['task_id'])
        
        future = self.go_nav_site_task_client.call_async(req)
        future.add_done_callback(lambda f: self._handle_service_response(cmd, f))
    
    def _handle_force_localize(self, cmd: Dict[str, Any]):
        """处理强制定位"""
        if not self.go_force_loc_client:
            self._send_result(cmd, {'success': False, 'message': '强制定位服务客户端未初始化'})
            return
        
        if not self.go_force_loc_client.wait_for_service(timeout_sec=1.0):
            self._send_result(cmd, {'success': False, 'message': '强制定位服务不可用'})
            return
        
        from qyh_standard_robot_msgs.srv import GoForceLocalize
        params = cmd['params']
        req = GoForceLocalize.Request()
        req.x = float(params['x'])
        req.y = float(params['y'])
        req.yaw = float(params['yaw'])
        
        future = self.go_force_loc_client.call_async(req)
        future.add_done_callback(lambda f: self._handle_service_response(cmd, f))
    
    def _handle_set_speed_level(self, cmd: Dict[str, Any]):
        """处理设置速度级别"""
        if not self.go_set_speed_client:
            self._send_result(cmd, {'success': False, 'message': '速度设置服务客户端未初始化'})
            return
        
        if not self.go_set_speed_client.wait_for_service(timeout_sec=1.0):
            self._send_result(cmd, {'success': False, 'message': '速度设置服务不可用'})
            return
        
        from qyh_standard_robot_msgs.srv import GoSetSpeedType
        params = cmd['params']
        req = GoSetSpeedType.Request()
        req.speed_level = int(params['level'])
        
        future = self.go_set_speed_client.call_async(req)
        future.add_done_callback(lambda f: self._handle_service_response(cmd, f))
    
    def _handle_set_obstacle_strategy(self, cmd: Dict[str, Any]):
        """处理设置避障策略"""
        if not self.go_set_obstacle_client:
            self._send_result(cmd, {'success': False, 'message': '避障策略服务客户端未初始化'})
            return
        
        if not self.go_set_obstacle_client.wait_for_service(timeout_sec=1.0):
            self._send_result(cmd, {'success': False, 'message': '避障策略服务不可用'})
            return
        
        from qyh_standard_robot_msgs.srv import GoSetObstacleStrategy
        params = cmd['params']
        req = GoSetObstacleStrategy.Request()
        req.strategy = int(params['strategy'])
        
        future = self.go_set_obstacle_client.call_async(req)
        future.add_done_callback(lambda f: self._handle_service_response(cmd, f))
    
    def _handle_set_current_site(self, cmd: Dict[str, Any]):
        """处理设置当前站点"""
        if not self.go_set_site_client:
            self._send_result(cmd, {'success': False, 'message': '站点设置服务客户端未初始化'})
            return
        
        if not self.go_set_site_client.wait_for_service(timeout_sec=1.0):
            self._send_result(cmd, {'success': False, 'message': '站点设置服务不可用'})
            return
        
        from qyh_standard_robot_msgs.srv import GoSetCurrentSite
        params = cmd['params']
        req = GoSetCurrentSite.Request()
        req.site_id = int(params['site_id'])
        
        future = self.go_set_site_client.call_async(req)
        future.add_done_callback(lambda f: self._handle_service_response(cmd, f))
    
    def _handle_set_volume(self, cmd: Dict[str, Any]):
        """处理设置音量"""
        if not self.go_set_volume_client:
            self._send_result(cmd, {'success': False, 'message': '音量设置服务客户端未初始化'})
            return
        
        if not self.go_set_volume_client.wait_for_service(timeout_sec=1.0):
            self._send_result(cmd, {'success': False, 'message': '音量设置服务不可用'})
            return
        
        from qyh_standard_robot_msgs.srv import GoSetSpeakerVolume
        params = cmd['params']
        req = GoSetSpeakerVolume.Request()
        req.volume = int(params['volume'])
        
        future = self.go_set_volume_client.call_async(req)
        future.add_done_callback(lambda f: self._handle_service_response(cmd, f))
    
    def _handle_set_map(self, cmd: Dict[str, Any]):
        """处理设置当前地图"""
        if not self.go_set_map_client:
            self._send_result(cmd, {'success': False, 'message': '地图设置服务客户端未初始化'})
            return
        
        if not self.go_set_map_client.wait_for_service(timeout_sec=1.0):
            self._send_result(cmd, {'success': False, 'message': '地图设置服务不可用'})
            return
        
        from qyh_standard_robot_msgs.srv import GoSetCurrentMap
        params = cmd['params']
        req = GoSetCurrentMap.Request()
        req.map_name = str(params['map_name'])
        
        future = self.go_set_map_client.call_async(req)
        future.add_done_callback(lambda f: self._handle_service_response(cmd, f))
    
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
    
    def get_status(self) -> Optional[Dict[str, Any]]:
        """获取底盘状态（带超时检测）"""
        if self.mock_mode:
            return None
        
        if self.chassis_status is None:
            return None
        
        # 检查是否超时
        if self.chassis_last_update_time is not None:
            elapsed = time.time() - self.chassis_last_update_time
            if elapsed > self.chassis_timeout_seconds:
                status = self.chassis_status.copy()
                status["connected"] = False
                return status
        
        return self.chassis_status
    
    def get_navigation_status(self) -> Optional[Dict[str, Any]]:
        """获取导航状态"""
        if self.mock_mode:
            return None
        return self.navigation_status
    
    # ==================== 异步接口 ====================
    
    async def send_velocity(
        self,
        linear_x: float,
        linear_y: float,
        angular_z: float
    ) -> Optional[Dict[str, Any]]:
        """发送速度命令"""
        if self.mock_mode:
            return None
        
        return await self._create_async_command('chassis_velocity', {
            'linear_x': linear_x,
            'linear_y': linear_y,
            'angular_z': angular_z
        })
    
    async def send_manual_motion(
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
        
        return await self._create_async_command('chassis_manual_motion', {
            'forward': forward,
            'backward': backward,
            'left': left,
            'right': right,
            'rotate_left': rotate_left,
            'rotate_right': rotate_right
        })
    
    async def call_service(self, service_name: str) -> Optional[Dict[str, Any]]:
        """调用控制服务"""
        if self.mock_mode:
            return None
        
        if not self.control_clients.get(service_name):
            return None
        
        return await self._create_async_command('chassis_control_service', {
            'service_name': service_name
        })
    
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
        
        return await self._create_async_command('chassis_nav_coordinate', {
            'x': x, 'y': y, 'yaw': yaw,
            'is_localization': is_localization
        })
    
    async def navigate_to_site(
        self,
        site_id: int,
        is_localization: bool = False
    ) -> Optional[Dict[str, Any]]:
        """导航到站点"""
        if self.mock_mode:
            return None
        
        return await self._create_async_command('chassis_nav_site', {
            'site_id': site_id,
            'is_localization': is_localization
        })
    
    async def navigate_to_site_simple(self, site_id: int) -> Optional[Dict[str, Any]]:
        """导航到站点（简单版本）"""
        if self.mock_mode:
            return None
        
        return await self._create_async_command('chassis_nav_site_simple', {
            'site_id': site_id
        })
    
    async def navigate_to_site_with_task(
        self,
        site_id: int,
        task_id: int
    ) -> Optional[Dict[str, Any]]:
        """导航到站点（带任务ID）"""
        if self.mock_mode:
            return None
        
        return await self._create_async_command('chassis_nav_site_task', {
            'site_id': site_id,
            'task_id': task_id
        })
    
    async def force_localize(
        self,
        x: float,
        y: float,
        yaw: float
    ) -> Optional[Dict[str, Any]]:
        """强制定位"""
        if self.mock_mode:
            return None
        
        return await self._create_async_command('chassis_force_localize', {
            'x': x, 'y': y, 'yaw': yaw
        })
    
    async def set_speed_level(self, level: int) -> Optional[Dict[str, Any]]:
        """设置速度级别"""
        if self.mock_mode:
            return None
        
        return await self._create_async_command('chassis_set_speed_level', {'level': level})
    
    async def set_obstacle_strategy(self, strategy: int) -> Optional[Dict[str, Any]]:
        """设置避障策略"""
        if self.mock_mode:
            return None
        
        return await self._create_async_command('chassis_set_obstacle_strategy', {'strategy': strategy})
    
    async def set_current_site(self, site_id: int) -> Optional[Dict[str, Any]]:
        """设置当前站点"""
        if self.mock_mode:
            return None
        
        return await self._create_async_command('chassis_set_current_site', {'site_id': site_id})
    
    async def set_volume(self, volume: int) -> Optional[Dict[str, Any]]:
        """设置音量"""
        if self.mock_mode:
            return None
        
        return await self._create_async_command('chassis_set_volume', {'volume': volume})
    
    async def set_map(self, map_name: str) -> Optional[Dict[str, Any]]:
        """设置当前地图"""
        if self.mock_mode:
            return None
        
        return await self._create_async_command('chassis_set_map', {'map_name': map_name})
    
    # ==================== 状态文本转换 ====================
    
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
