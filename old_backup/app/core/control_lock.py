"""控制权锁机制"""
from datetime import datetime, timedelta
from typing import Optional
import threading


class ControlLock:
    """控制权互斥锁"""
    
    def __init__(self):
        self.holder: Optional[int] = None  # user_id
        self.holder_username: Optional[str] = None
        self.expires_at: Optional[datetime] = None
        self.lock = threading.Lock()
    
    def acquire(self, user_id: int, username: str, duration: int = 300) -> bool:
        """
        获取控制权
        
        Args:
            user_id: 用户ID
            username: 用户名
            duration: 持续时间（秒）
        
        Returns:
            是否成功获取
        """
        with self.lock:
            now = datetime.now()
            
            # 如果没有持有者，或已过期
            if self.holder is None or now > self.expires_at:
                self.holder = user_id
                self.holder_username = username
                self.expires_at = now + timedelta(seconds=duration)
                return True
            
            # 如果是同一用户，允许续约
            if self.holder == user_id:
                self.expires_at = now + timedelta(seconds=duration)
                return True
            
            return False
    
    def release(self, user_id: int) -> bool:
        """
        释放控制权
        
        Args:
            user_id: 用户ID
        
        Returns:
            是否成功释放
        """
        with self.lock:
            if self.holder == user_id:
                self.holder = None
                self.holder_username = None
                self.expires_at = None
                return True
            return False
    
    def force_release(self):
        """强制释放（Admin 或看门狗）"""
        with self.lock:
            self.holder = None
            self.holder_username = None
            self.expires_at = None
    
    def get_holder(self) -> Optional[dict]:
        """
        获取当前持有者信息
        
        Returns:
            持有者信息，如果无持有者则返回 None
        """
        with self.lock:
            if self.expires_at and datetime.now() > self.expires_at:
                # 已过期，自动释放
                self.holder = None
                self.holder_username = None
                self.expires_at = None
            
            if self.holder is None:
                return None
            
            return {
                "user_id": self.holder,
                "username": self.holder_username,
                "expires_at": self.expires_at
            }
    
    def is_held_by(self, user_id: int) -> bool:
        """检查是否被指定用户持有"""
        holder = self.get_holder()
        return holder is not None and holder["user_id"] == user_id


# 全局单例
control_lock = ControlLock()
