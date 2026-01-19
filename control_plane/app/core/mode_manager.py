"""
QYH Jushen Control Plane - 模式状态机

管理机器人工作模式的切换
"""
from enum import Enum
from typing import Optional, Dict, Any, List
from datetime import datetime
import threading


class RobotMode(str, Enum):
    """机器人工作模式"""
    IDLE = "idle"               # 空闲
    TELEOP = "teleop"           # 遥操作
    AUTO = "auto"               # 自主任务
    MAINTENANCE = "maintenance" # 维护模式
    ERROR = "error"             # 错误状态


# 模式转换规则
MODE_TRANSITIONS = {
    RobotMode.IDLE: [RobotMode.TELEOP, RobotMode.AUTO, RobotMode.MAINTENANCE],
    RobotMode.TELEOP: [RobotMode.IDLE],
    RobotMode.AUTO: [RobotMode.IDLE],  # 任务完成或取消后才能退出
    RobotMode.MAINTENANCE: [RobotMode.IDLE],
    RobotMode.ERROR: [RobotMode.IDLE, RobotMode.MAINTENANCE],  # 错误状态可切换到空闲或维护
}


class ModeManager:
    """
    模式状态机管理器
    
    管理机器人工作模式的状态和切换
    """
    
    def __init__(self):
        self._current_mode: RobotMode = RobotMode.IDLE
        self._previous_mode: Optional[RobotMode] = None
        self._mode_changed_at: datetime = datetime.now()
        self._lock = threading.Lock()
        self._mode_data: Dict[str, Any] = {}  # 模式相关的额外数据
    
    @property
    def current_mode(self) -> RobotMode:
        """获取当前模式"""
        with self._lock:
            return self._current_mode
    
    @property
    def previous_mode(self) -> Optional[RobotMode]:
        """获取上一个模式"""
        with self._lock:
            return self._previous_mode
    
    def can_switch_to(self, target_mode: RobotMode) -> bool:
        """
        检查是否可以切换到目标模式
        
        Args:
            target_mode: 目标模式
        
        Returns:
            是否可以切换
        """
        with self._lock:
            if self._current_mode == target_mode:
                return True  # 已经是目标模式
            
            allowed_transitions = MODE_TRANSITIONS.get(self._current_mode, [])
            return target_mode in allowed_transitions
    
    def switch_to(
        self,
        target_mode: RobotMode,
        mode_data: Optional[Dict[str, Any]] = None,
        force: bool = False,
    ) -> tuple[bool, str]:
        """
        切换到目标模式
        
        Args:
            target_mode: 目标模式
            mode_data: 模式相关数据
            force: 是否强制切换（忽略转换规则）
        
        Returns:
            (是否成功, 消息)
        """
        with self._lock:
            if self._current_mode == target_mode:
                return True, f"已经是 {target_mode.value} 模式"
            
            if not force:
                allowed_transitions = MODE_TRANSITIONS.get(self._current_mode, [])
                if target_mode not in allowed_transitions:
                    return False, f"不允许从 {self._current_mode.value} 切换到 {target_mode.value}"
            
            # 执行切换
            self._previous_mode = self._current_mode
            self._current_mode = target_mode
            self._mode_changed_at = datetime.now()
            self._mode_data = mode_data or {}
            
            return True, f"已切换到 {target_mode.value} 模式"
    
    def get_status(self) -> Dict[str, Any]:
        """获取模式状态"""
        with self._lock:
            return {
                "current_mode": self._current_mode.value,
                "previous_mode": self._previous_mode.value if self._previous_mode else None,
                "mode_changed_at": self._mode_changed_at.isoformat(),
                "available_transitions": [
                    m.value for m in MODE_TRANSITIONS.get(self._current_mode, [])
                ],
                "mode_data": self._mode_data,
            }
    
    def get_available_modes(self) -> List[Dict[str, Any]]:
        """获取所有可用模式及其描述"""
        mode_descriptions = {
            RobotMode.IDLE: {
                "name": "idle",
                "display_name": "空闲",
                "description": "机器人处于空闲状态，可进行状态查询",
                "allowed_operations": ["status_query", "mode_switch"],
            },
            RobotMode.TELEOP: {
                "name": "teleop",
                "display_name": "遥操作",
                "description": "VR/手柄控制模式",
                "allowed_operations": ["status_query", "teleop_control"],
            },
            RobotMode.AUTO: {
                "name": "auto",
                "display_name": "自主任务",
                "description": "执行预设任务",
                "allowed_operations": ["status_query", "task_control"],
            },
            RobotMode.MAINTENANCE: {
                "name": "maintenance",
                "display_name": "维护",
                "description": "维护模式，可进行参数配置和标定",
                "allowed_operations": ["status_query", "config", "calibration"],
            },
            RobotMode.ERROR: {
                "name": "error",
                "display_name": "错误",
                "description": "系统错误状态",
                "allowed_operations": ["status_query", "error_recovery"],
            },
        }
        
        result = []
        for mode in RobotMode:
            info = mode_descriptions.get(mode, {})
            info["is_current"] = (mode == self._current_mode)
            info["can_switch_to"] = self.can_switch_to(mode)
            result.append(info)
        
        return result
    
    def set_error(self, error_message: str):
        """进入错误状态"""
        self.switch_to(
            RobotMode.ERROR,
            mode_data={"error_message": error_message},
            force=True,
        )
    
    def clear_error(self) -> bool:
        """清除错误状态，返回空闲"""
        if self._current_mode == RobotMode.ERROR:
            return self.switch_to(RobotMode.IDLE, force=True)[0]
        return False


# 全局单例
mode_manager = ModeManager()
