"""认证相关 API"""
from fastapi import APIRouter, Depends, HTTPException, status
from fastapi.security import HTTPBearer, HTTPAuthorizationCredentials
from sqlalchemy.orm import Session
from datetime import timedelta
from app.database import get_db
from app.models.user import User
from app.schemas.auth import LoginRequest, LoginResponse
from app.core.security import verify_password, create_access_token
from app.config import settings

router = APIRouter()


@router.post("/login", response_model=LoginResponse)
async def login(request: LoginRequest, db: Session = Depends(get_db)):
    """用户登录"""
    # 查询用户
    user = db.query(User).filter(User.username == request.username).first()
    
    if not user or not verify_password(request.password, user.hashed_password):
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="用户名或密码错误"
        )
    
    if not user.is_active:
        raise HTTPException(
            status_code=status.HTTP_403_FORBIDDEN,
            detail="用户已被禁用"
        )
    
    # 生成 Token
    access_token = create_access_token(
        data={"sub": str(user.id), "username": user.username, "role": user.role}
    )
    
    # 更新最后登录时间
    from datetime import datetime
    user.last_login = datetime.utcnow()
    db.commit()
    
    return {
        "access_token": access_token,
        "token_type": "bearer",
        "expires_in": settings.ACCESS_TOKEN_EXPIRE_MINUTES * 60,
        "user": {
            "id": user.id,
            "username": user.username,
            "role": user.role,
            "email": user.email
        }
    }


@router.post("/logout")
async def logout():
    """用户登出（JWT 无状态，前端删除 Token 即可）"""
    return {"message": "登出成功"}


@router.get("/me")
async def get_current_user_info(user: User = Depends(get_db)):
    """获取当前用户信息"""
    from app.dependencies import get_current_user
    return {
        "id": user.id,
        "username": user.username,
        "role": user.role,
        "email": user.email,
        "created_at": user.created_at
    }


@router.post("/refresh")
async def refresh_token(
    credentials: HTTPAuthorizationCredentials = Depends(HTTPBearer())
):
    """刷新 Token
    
    检查当前 Token，如果接近过期则返回新的 Token
    """
    from app.core.security import should_refresh_token, refresh_access_token
    
    old_token = credentials.credentials
    
    try:
        # 检查是否需要刷新
        if should_refresh_token(old_token):
            new_token = refresh_access_token(old_token)
            return {
                "refreshed": True,
                "access_token": new_token,
                "token_type": "bearer",
                "expires_in": settings.ACCESS_TOKEN_EXPIRE_MINUTES * 60
            }
        else:
            return {
                "refreshed": False,
                "message": "Token 仍然有效，无需刷新"
            }
    except Exception as e:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Token 无效或已过期"
        )


@router.post("/heartbeat")
async def heartbeat(
    credentials: HTTPAuthorizationCredentials = Depends(HTTPBearer())
):
    """心跳检测
    
    检查 Token 有效性，如果接近过期则自动刷新
    """
    from app.core.security import should_refresh_token, refresh_access_token, decode_access_token
    from fastapi.security import HTTPBearer
    
    old_token = credentials.credentials
    
    try:
        # 验证 Token 有效性
        payload = decode_access_token(old_token)
        
        # 检查是否需要刷新
        if should_refresh_token(old_token):
            new_token = refresh_access_token(old_token)
            return {
                "alive": True,
                "refreshed": True,
                "access_token": new_token,
                "token_type": "bearer",
                "expires_in": settings.ACCESS_TOKEN_EXPIRE_MINUTES * 60,
                "user": {
                    "username": payload.get("username"),
                    "role": payload.get("role")
                }
            }
        else:
            return {
                "alive": True,
                "refreshed": False,
                "user": {
                    "username": payload.get("username"),
                    "role": payload.get("role")
                }
            }
    except Exception:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="认证已过期，请重新登录"
        )
