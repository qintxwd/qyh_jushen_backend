"""Control lock for exclusive robot operation."""
from __future__ import annotations

from datetime import datetime, timedelta
from threading import Lock
from typing import Optional


class ControlLock:
    def __init__(self) -> None:
        self.holder: Optional[int] = None
        self.holder_username: Optional[str] = None
        self.expires_at: Optional[datetime] = None
        self._lock = Lock()

    def acquire(self, user_id: int, username: str, duration: int = 300) -> bool:
        with self._lock:
            now = datetime.now()
            if self.holder is None or (self.expires_at and now > self.expires_at):
                self.holder = user_id
                self.holder_username = username
                self.expires_at = now + timedelta(seconds=duration)
                return True
            if self.holder == user_id:
                self.expires_at = now + timedelta(seconds=duration)
                return True
            return False

    def release(self, user_id: int) -> bool:
        with self._lock:
            if self.holder == user_id:
                self.holder = None
                self.holder_username = None
                self.expires_at = None
                return True
            return False

    def force_release(self) -> None:
        with self._lock:
            self.holder = None
            self.holder_username = None
            self.expires_at = None

    def get_holder(self) -> Optional[dict]:
        with self._lock:
            if self.expires_at and datetime.now() > self.expires_at:
                self.holder = None
                self.holder_username = None
                self.expires_at = None
            if self.holder is None:
                return None
            return {
                "user_id": self.holder,
                "username": self.holder_username,
                "expires_at": self.expires_at,
            }

    def is_held_by(self, user_id: int) -> bool:
        holder = self.get_holder()
        return holder is not None and holder["user_id"] == user_id


control_lock = ControlLock()
