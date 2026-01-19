"""
QYH Jushen Control Plane - 控制权锁机制

实现机器人控制权的互斥访问
"""
from datetime import datetime, timedelta
from typing import Optional, Dict, Any
import threading


class ControlLock:
    """
    控制权互斥锁
    
    确保同一时间只有一个用户可以控制机器人
    """
    
    def __init__(self):
        self._holder: Optional[int] = None  # user_id
        self._holder_username: Optional[str] = None
        self._expires_at: Optional[datetime] = None
        self._session_type: Optional[str] = None  # teleop, auto
        self._lock = threading.Lock()
    
    def acquire(
        self,
        user_id: int,
        username: str,
        duration: int = 300,
        session_type: str = "teleop",
    ) -> bool:
        """
        获取控制权
        
        Args:
            user_id: 用户 ID
            username: 用户名
            duration: 持续时间（秒），默认 5 分钟
            session_type: 会话类型 (teleop, auto)
        
        Returns:
            是否成功获取
        """
        with self._lock:
            now = datetime.now()
            
            # 如果没有持有者，或已过期
            if self._holder is None or (self._expires_at and now > self._expires_at):
                self._holder = user_id
                self._holder_username = username
                self._expires_at = now + timedelta(seconds=duration)
                self._session_type = session_type
                return True
            
            # 如果是同一用户，允许续约
            if self._holder == user_id:
                self._expires_at = now + timedelta(seconds=duration)
                return True
            
            return False
    
    def release(self, user_id: int) -> bool:
        """
        释放控制权
        
        Args:
            user_id: 用户 ID
        
        Returns:
            是否成功释放
        """
        with self._lock:
            if self._holder == user_id:
                self._clear()
                return True
            return False
    
    def force_release(self, reason: str = "forced") -> Optional[Dict[str, Any]]:
        """
        强制释放控制权（Admin 或安全系统使用）
        
        Returns:
            之前的持有者信息（用于记录日志）
        """
        with self._lock:
            old_holder = self.get_holder_unsafe()
            self._clear()
            return old_holder
    
    def _clear(self):
        """清除控制权状态（内部方法，需在锁内调用）"""
        self._holder = None
        self._holder_username = None
        self._expires_at = None
        self._session_type = None
    
    def get_holder(self) -> Optional[Dict[str, Any]]:
        """
        获取当前持有者信息
        
        如果已过期，自动释放
        
        Returns:
            持有者信息，如果无持有者则返回 None
        """
        with self._lock:
            if self._expires_at and datetime.now() > self._expires_at:
                # 已过期，自动释放
                self._clear()
            
            return self.get_holder_unsafe()
    
    def get_holder_unsafe(self) -> Optional[Dict[str, Any]]:
        """获取持有者信息（不检查过期，需在锁内调用）"""
        if self._holder is None:
            return None
        
        return {
            "user_id": self._holder,
            "username": self._holder_username,
            "expires_at": self._expires_at.isoformat() if self._expires_at else None,
            "session_type": self._session_type,
            "remaining_seconds": self._get_remaining_seconds(),
        }
    
    def _get_remaining_seconds(self) -> int:
        """获取剩余时间（秒）"""
        if self._expires_at is None:
            return 0
        remaining = (self._expires_at - datetime.now()).total_seconds()
        return max(0, int(remaining))
    
    def is_held_by(self, user_id: int) -> bool:
        """检查是否被指定用户持有"""
        holder = self.get_holder()
        return holder is not None and holder["user_id"] == user_id
    
    def is_locked(self) -> bool:
        """检查是否被锁定"""
        return self.get_holder() is not None


# 全局单例
control_lock = ControlLock()
