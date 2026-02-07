"""
QYH Jushen Control Plane - FastAPI 依赖注入
"""
from typing import Optional

from fastapi import Depends, HTTPException, status
from fastapi.security import HTTPBearer, HTTPAuthorizationCredentials
from sqlalchemy.orm import Session
from jose import JWTError

from app.database import get_db
from app.models.user import User
from app.core.security import decode_access_token
from app.config import settings


# HTTP Bearer 认证方案
security = HTTPBearer()

def _is_media_token(payload: dict) -> bool:
    """Check whether a JWT is intended for Media Plane usage."""
    aud = payload.get("aud")
    if isinstance(aud, list):
        aud = aud[0] if aud else None

    scope = payload.get("scope", "")
    if isinstance(scope, list):
        scope = " ".join([s for s in scope if isinstance(s, str)])
    elif not isinstance(scope, str):
        scope = ""

    media_audience = settings.MEDIA_TOKEN_AUDIENCE
    media_scope = settings.MEDIA_TOKEN_SCOPE
    scope_tokens = set(scope.replace(",", " ").split())

    return (
        bool(media_audience) and aud == media_audience
    ) or (
        bool(media_scope) and media_scope in scope_tokens
    )


async def get_current_user(
    credentials: HTTPAuthorizationCredentials = Depends(security),
    db: Session = Depends(get_db),
) -> User:
    """
    获取当前认证用户
    
    从 JWT Token 中解析用户信息
    """
    credentials_exception = HTTPException(
        status_code=status.HTTP_401_UNAUTHORIZED,
        detail="无法验证凭据",
        headers={"WWW-Authenticate": "Bearer"},
    )
    
    try:
        token = credentials.credentials
        payload = decode_access_token(token)
        user_id: str = payload.get("sub")

        if _is_media_token(payload):
            raise credentials_exception
        
        if user_id is None:
            raise credentials_exception
            
    except JWTError:
        raise credentials_exception
    
    user = db.query(User).filter(User.id == int(user_id)).first()
    
    if user is None:
        raise credentials_exception
    
    if not user.is_active:
        raise HTTPException(
            status_code=status.HTTP_403_FORBIDDEN,
            detail="用户已被禁用",
        )
    
    return user


async def get_current_admin(
    current_user: User = Depends(get_current_user),
) -> User:
    """
    获取当前管理员用户
    
    要求用户角色为 admin
    """
    if current_user.role != "admin":
        raise HTTPException(
            status_code=status.HTTP_403_FORBIDDEN,
            detail="需要管理员权限",
        )
    return current_user


async def get_current_operator(
    current_user: User = Depends(get_current_user),
) -> User:
    """
    获取当前操作员用户
    
    要求用户角色为 admin 或 operator
    """
    if current_user.role not in ("admin", "operator"):
        raise HTTPException(
            status_code=status.HTTP_403_FORBIDDEN,
            detail="需要操作员或管理员权限",
        )
    return current_user


def get_optional_user(
    credentials: Optional[HTTPAuthorizationCredentials] = Depends(
        HTTPBearer(auto_error=False)
    ),
    db: Session = Depends(get_db),
) -> Optional[User]:
    """
    获取可选的当前用户
    
    如果没有提供 Token 或 Token 无效，返回 None
    """
    if credentials is None:
        return None
    
    try:
        token = credentials.credentials
        payload = decode_access_token(token)
        user_id: str = payload.get("sub")

        if _is_media_token(payload):
            return None
        
        if user_id is None:
            return None
        
        user = db.query(User).filter(User.id == int(user_id)).first()
        
        if user and user.is_active:
            return user
        
    except JWTError:
        pass
    
    return None
