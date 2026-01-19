"""安全看门狗 - 心跳监控"""
import time
import threading
from app.core.control_lock import control_lock


class SafetyWatchdog:
    """安全看门狗（心跳监控）"""
    
    def __init__(self, timeout: float = 1.0):
        self.timeout = timeout
        self.last_heartbeat = time.time()
        self.thread: threading.Thread = None
        self.running = False
        self.lock = threading.Lock()
    
    def start(self):
        """启动看门狗"""
        if self.running:
            return
        
        self.running = True
        self.thread = threading.Thread(target=self._monitor, daemon=True)
        self.thread.start()
        print("✅ 安全看门狗已启动")
    
    def stop(self):
        """停止看门狗"""
        self.running = False
        if self.thread:
            self.thread.join(timeout=2.0)
        print("🛑 安全看门狗已停止")
    
    def heartbeat(self):
        """接收心跳"""
        with self.lock:
            self.last_heartbeat = time.time()
    
    def _monitor(self):
        """监控线程"""
        while self.running:
            time.sleep(0.1)
            
            with self.lock:
                elapsed = time.time() - self.last_heartbeat
            
            if elapsed > self.timeout:
                self._emergency_stop()
                # 重置心跳，避免重复触发
                with self.lock:
                    self.last_heartbeat = time.time()
    
    def _emergency_stop(self):
        """执行急停"""
        print(f"⚠️  心跳超时 ({self.timeout}s)，触发急停")
        
        # 发送急停指令到 ROS2
        try:
            from app.ros2_bridge.bridge import ros2_bridge
            ros2_bridge.send_command({
                'type': 'emergency_stop',
                'params': {}
            })
        except Exception as e:
            print(f"❌ 急停指令发送失败: {e}")
        
        # 释放控制权
        control_lock.force_release()
        
        # TODO: 记录审计日志


# 全局单例
watchdog = SafetyWatchdog()
