"""Safety watchdog for control heartbeat monitoring."""
from __future__ import annotations

import threading
import time

from app.core.control_lock import control_lock


class SafetyWatchdog:
    def __init__(self, timeout: float = 1.0) -> None:
        self.timeout = timeout
        self.last_heartbeat = time.time()
        self.thread: threading.Thread | None = None
        self.running = False
        self._lock = threading.Lock()

    def start(self) -> None:
        if self.running:
            return
        self.running = True
        self.thread = threading.Thread(target=self._monitor, daemon=True)
        self.thread.start()

    def stop(self) -> None:
        self.running = False
        if self.thread:
            self.thread.join(timeout=2.0)

    def heartbeat(self) -> None:
        with self._lock:
            self.last_heartbeat = time.time()

    def _monitor(self) -> None:
        while self.running:
            time.sleep(0.1)
            with self._lock:
                elapsed = time.time() - self.last_heartbeat
            if elapsed > self.timeout:
                self._emergency_release()
                with self._lock:
                    self.last_heartbeat = time.time()

    def _emergency_release(self) -> None:
        control_lock.force_release()


watchdog = SafetyWatchdog()
