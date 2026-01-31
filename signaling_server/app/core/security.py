from datetime import datetime, timedelta, timezone
from typing import Optional

import jwt
from fastapi import HTTPException, Request
from passlib.context import CryptContext

from app.config import settings

ALGORITHM = "HS256"
pwd_context = CryptContext(schemes=["bcrypt"], deprecated="auto")


class TokenPayload:
    def __init__(self, sub: str, role: str = "user") -> None:
        self.sub = sub
        self.role = role


def create_access_token(subject: str, role: str = "user") -> str:
    now = datetime.now(timezone.utc)
    exp = now + timedelta(minutes=settings.jwt_expire_minutes)
    payload = {
        "sub": subject,
        "role": role,
        "iat": int(now.timestamp()),
        "exp": int(exp.timestamp()),
    }
    return jwt.encode(payload, settings.jwt_secret, algorithm=ALGORITHM)


def verify_password(plain_password: str, hashed_password: str) -> bool:
    return pwd_context.verify(plain_password, hashed_password)


def get_password_hash(password: str) -> str:
    return pwd_context.hash(password)


def decode_token(token: str) -> TokenPayload:
    try:
        payload = jwt.decode(
            token,
            settings.jwt_secret,
            algorithms=[ALGORITHM],
        )
        return TokenPayload(
            sub=payload["sub"],
            role=payload.get("role", "user"),
        )
    except jwt.PyJWTError:
        raise HTTPException(status_code=401, detail="Invalid or expired token")


def get_current_user(request: Request) -> TokenPayload:
    auth = request.headers.get("Authorization", "")
    if not auth.startswith("Bearer "):
        raise HTTPException(
            status_code=401,
            detail="Missing Authorization header",
        )
    token = auth[7:]
    payload = decode_token(token)
    return payload


def get_robot_identity(request: Request) -> Optional[str]:
    secret = request.headers.get("X-Robot-Secret")
    robot_id = request.headers.get("X-Robot-Id")
    if not secret or not robot_id:
        return None
    if secret != settings.robot_shared_secret:
        raise HTTPException(status_code=401, detail="Invalid robot secret")
    return robot_id


def get_caller(request: Request) -> dict:
    """
    返回 {"type": "user"|"robot", "id": str}
    """
    robot_id = get_robot_identity(request)
    if robot_id:
        return {"type": "robot", "id": robot_id}
    user = get_current_user(request)
    return {"type": "user", "id": user.sub}
