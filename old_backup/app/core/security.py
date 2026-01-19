"""安全相关功能（JWT、密码加密）"""
from datetime import datetime, timedelta
from jose import JWTError, jwt
from passlib.context import CryptContext
from app.config import settings

# 密码加密上下文
pwd_context = CryptContext(schemes=["bcrypt"], deprecated="auto")


def verify_password(plain_password: str, hashed_password: str) -> bool:
    """验证密码"""
    return pwd_context.verify(plain_password, hashed_password)


def get_password_hash(password: str) -> str:
    """生成密码哈希"""
    return pwd_context.hash(password)


def create_access_token(data: dict, expires_delta: timedelta = None) -> str:
    """创建 JWT Token"""
    to_encode = data.copy()
    
    if expires_delta:
        expire = datetime.utcnow() + expires_delta
    else:
        expire = datetime.utcnow() + timedelta(minutes=settings.ACCESS_TOKEN_EXPIRE_MINUTES)
    
    to_encode.update({"exp": expire, "iat": datetime.utcnow()})
    
    encoded_jwt = jwt.encode(
        to_encode,
        settings.SECRET_KEY,
        algorithm=settings.ALGORITHM
    )
    
    return encoded_jwt


def decode_access_token(token: str) -> dict:
    """解码 JWT Token"""
    try:
        payload = jwt.decode(
            token,
            settings.SECRET_KEY,
            algorithms=[settings.ALGORITHM]
        )
        return payload
    except JWTError:
        raise ValueError("无效的 Token")


def should_refresh_token(token: str) -> bool:
    """检查是否应该刷新 Token
    
    如果 Token 剩余有效时间少于阈值，返回 True
    """
    try:
        payload = decode_access_token(token)
        exp = payload.get("exp")
        if not exp:
            return False
        
        # 计算剩余有效时间（秒）
        remaining_seconds = exp - datetime.utcnow().timestamp()
        remaining_minutes = remaining_seconds / 60
        
        # 如果剩余时间少于阈值，需要刷新
        return remaining_minutes < settings.TOKEN_REFRESH_THRESHOLD_MINUTES
    except:
        return False


def refresh_access_token(old_token: str) -> str:
    """刷新 Token（保留原有信息，延长过期时间）"""
    try:
        # 解码旧 Token
        payload = decode_access_token(old_token)
        
        # 移除过期时间和签发时间
        payload.pop("exp", None)
        payload.pop("iat", None)
        
        # 创建新 Token
        return create_access_token(data=payload)
    except:
        raise ValueError("无法刷新 Token")
