from pydantic import BaseModel
from typing import List


class IceServer(BaseModel):
    urls: str
    username: str | None = None
    credential: str | None = None


class Settings(BaseModel):
    jwt_secret: str = "qyh-signaling-secret-change-me"
    jwt_expire_minutes: int = 60 * 24
    robot_shared_secret: str = "qyh-robot-secret"

    default_users: dict[str, str] = {
        "admin": "admin123",
        "demo": "demo123",
    }

    ice_servers: List[IceServer] = [
        IceServer(urls="stun:stun.l.google.com:19302"),
    ]


settings = Settings()
