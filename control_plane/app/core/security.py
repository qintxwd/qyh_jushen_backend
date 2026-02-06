"""
QYH Jushen Control Plane - 安全模块

JWT 签发、校验、密码哈希
"""
from datetime import datetime, timedelta
import logging

logger = logging.getLogger(__name__)
from typing import Optional, Dict, Any

from jose import JWTError, jwt
from passlib.context import CryptContext

from app.config import settings


# 密码哈希上下文
pwd_context = CryptContext(schemes=["bcrypt"], deprecated="auto")


def verify_password(plain_password: str, hashed_password: str) -> bool:
    """验证密码"""
    return pwd_context.verify(plain_password, hashed_password)


def get_password_hash(password: str) -> str:
    """生成密码哈希"""
    return pwd_context.hash(password)


def create_access_token(
    data: Dict[str, Any],
    expires_delta: Optional[timedelta] = None,
) -> str:
    """
    创建 JWT Access Token
    
    Args:
        data: Token 载荷数据，应包含 sub(user_id), username, role
        expires_delta: 过期时间增量
    
    Returns:
        JWT Token 字符串
    """
    to_encode = data.copy()
    
    if expires_delta:
        expire = datetime.utcnow() + expires_delta
    else:
        expire = datetime.utcnow() + timedelta(minutes=settings.ACCESS_TOKEN_EXPIRE_MINUTES)
    
    to_encode.update({
        "exp": expire,
        "iat": datetime.utcnow(),
    })
    
    logger.debug("[AUTH] 🔐 创建Token:")
    logger.debug(f"[AUTH]   算法: {settings.ALGORITHM}")
    logger.debug(
        "[AUTH]   载荷: sub=%s, username=%s, role=%s",
        to_encode.get("sub"),
        to_encode.get("username"),
        to_encode.get("role"),
    )
    logger.debug(f"[AUTH]   过期时间: {expire}")
    
    encoded_jwt = jwt.encode(
        to_encode,
        settings.SECRET_KEY,
        algorithm=settings.ALGORITHM,
    )
    
    logger.debug(f"[AUTH]   Token长度: {len(encoded_jwt)}")
    
    return encoded_jwt


def decode_access_token(token: str) -> Dict[str, Any]:
    """
    解码并验证 JWT Token
    
    Args:
        token: JWT Token 字符串
    
    Returns:
        Token 载荷数据
    
    Raises:
        JWTError: Token 无效或已过期
    """
    logger.debug("[AUTH] 🔓 解码Token:")
    logger.debug(f"[AUTH]   Token长度: {len(token)}")
    logger.debug(f"[AUTH]   算法: {settings.ALGORITHM}")
    
    try:
        payload = jwt.decode(
            token,
            settings.SECRET_KEY,
            algorithms=[settings.ALGORITHM],
        )
        logger.debug(f"[AUTH]   ✅ 解码成功! sub={payload.get('sub')}, username={payload.get('username')}")
        return payload
    except JWTError as e:
        logger.debug(f"[AUTH]   ❌ 解码失败: {type(e).__name__}: {e}")
        raise


def should_refresh_token(token: str) -> bool:
    """
    检查 Token 是否需要刷新
    
    如果 Token 剩余有效期小于阈值，返回 True
    """
    try:
        payload = decode_access_token(token)
        exp = payload.get("exp")
        
        if exp:
            exp_datetime = datetime.fromtimestamp(exp)
            remaining = exp_datetime - datetime.utcnow()
            threshold = timedelta(minutes=settings.TOKEN_REFRESH_THRESHOLD_MINUTES)
            
            return remaining < threshold
        
        return False
    except JWTError:
        return False


def refresh_access_token(token: str) -> str:
    """
    刷新 JWT Token
    
    解析旧 Token，保留原有数据，生成新 Token
    """
    payload = decode_access_token(token)
    
    # 移除旧的时间戳
    payload.pop("exp", None)
    payload.pop("iat", None)
    
    return create_access_token(payload)
