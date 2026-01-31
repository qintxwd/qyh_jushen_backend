from pydantic import BaseModel
from typing import List


class IceServer(BaseModel):
    urls: str
    username: str | None = None
    credential: str | None = None


class Settings(BaseModel):
    database_url: str = "sqlite:///./data/signaling.db"
    jwt_secret: str = "qyh-signaling-secret-change-me"
    jwt_expire_minutes: int = 60 * 24
    robot_shared_secret: str = "qyh-robot-secret"
    control_watchdog_timeout: float = 1.0

    default_admin_username: str = "admin"
    default_admin_password: str = "admin123"
    default_admin_email: str = "admin@example.com"

    ice_servers: List[IceServer] = [
        IceServer(urls="stun:stun.l.google.com:19302"),
    ]


settings = Settings()
