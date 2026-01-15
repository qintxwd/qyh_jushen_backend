"""认证相关 API"""
from fastapi import APIRouter, Depends
from fastapi.security import HTTPBearer, HTTPAuthorizationCredentials
from sqlalchemy.orm import Session
from datetime import datetime
from app.database import get_db
from app.models.user import User
from app.schemas.auth import LoginRequest
from app.schemas.response import (
    ApiResponse, success_response, error_response, ErrorCodes
)
from app.core.security import verify_password, create_access_token
from app.config import settings

router = APIRouter()


@router.post("/login", response_model=ApiResponse)
async def login(request: LoginRequest, db: Session = Depends(get_db)):
    """用户登录
    
    Returns:
        ApiResponse: 统一响应格式，data 包含 access_token, token_type, expires_in, user
    """
    # 查询用户
    user = db.query(User).filter(User.username == request.username).first()
    
    if not user or not verify_password(request.password, user.hashed_password):
        return error_response(
            code=ErrorCodes.AUTH_INVALID_CREDENTIALS,
            message="用户名或密码错误"
        )
    
    if not user.is_active:
        return error_response(
            code=ErrorCodes.AUTH_USER_DISABLED,
            message="用户已被禁用"
        )
    
    # 生成 Token
    access_token = create_access_token(
        data={"sub": str(user.id), "username": user.username, "role": user.role}
    )
    
    # 更新最后登录时间
    user.last_login = datetime.utcnow()
    db.commit()
    
    return success_response(
        data={
            "access_token": access_token,
            "token_type": "bearer",
            "expires_in": settings.ACCESS_TOKEN_EXPIRE_MINUTES * 60,
            "user": {
                "id": user.id,
                "username": user.username,
                "role": user.role,
                "email": user.email
            }
        },
        message="登录成功"
    )


@router.post("/logout", response_model=ApiResponse)
async def logout():
    """用户登出（JWT 无状态，前端删除 Token 即可）
    
    Returns:
        ApiResponse: 统一响应格式
    """
    return success_response(message="登出成功")


@router.get("/me", response_model=ApiResponse)
async def get_current_user_info(db: Session = Depends(get_db)):
    """获取当前用户信息
    
    Returns:
        ApiResponse: 统一响应格式，data 包含用户信息
    """
    from app.dependencies import get_current_user
    # 注意：此处需要通过依赖注入获取当前用户
    # 这里先保留原逻辑，后续可优化
    return success_response(
        data={
            "message": "请使用 Authorization header 并通过依赖注入获取用户信息"
        }
    )


@router.post("/refresh", response_model=ApiResponse)
async def refresh_token(
    credentials: HTTPAuthorizationCredentials = Depends(HTTPBearer())
):
    """刷新 Token
    
    检查当前 Token，如果接近过期则返回新的 Token
    
    Returns:
        ApiResponse: 统一响应格式，data 包含 refreshed, access_token 等
    """
    from app.core.security import should_refresh_token, refresh_access_token
    
    old_token = credentials.credentials
    
    try:
        # 检查是否需要刷新
        if should_refresh_token(old_token):
            new_token = refresh_access_token(old_token)
            return success_response(
                data={
                    "refreshed": True,
                    "access_token": new_token,
                    "token_type": "bearer",
                    "expires_in": settings.ACCESS_TOKEN_EXPIRE_MINUTES * 60
                },
                message="Token 已刷新"
            )
        else:
            return success_response(
                data={"refreshed": False},
                message="Token 仍然有效，无需刷新"
            )
    except Exception as e:
        return error_response(
            code=ErrorCodes.AUTH_TOKEN_EXPIRED,
            message="Token 无效或已过期"
        )


@router.post("/heartbeat", response_model=ApiResponse)
async def heartbeat(
    credentials: HTTPAuthorizationCredentials = Depends(HTTPBearer())
):
    """心跳检测
    
    检查 Token 有效性，如果接近过期则自动刷新
    
    Returns:
        ApiResponse: 统一响应格式，data 包含 alive, refreshed, user 等
    """
    from app.core.security import should_refresh_token, refresh_access_token, decode_access_token
    
    old_token = credentials.credentials
    
    try:
        # 验证 Token 有效性
        payload = decode_access_token(old_token)
        
        # 检查是否需要刷新
        if should_refresh_token(old_token):
            new_token = refresh_access_token(old_token)
            return success_response(
                data={
                    "alive": True,
                    "refreshed": True,
                    "access_token": new_token,
                    "token_type": "bearer",
                    "expires_in": settings.ACCESS_TOKEN_EXPIRE_MINUTES * 60,
                    "user": {
                        "username": payload.get("username"),
                        "role": payload.get("role")
                    }
                },
                message="心跳正常，Token 已刷新"
            )
        else:
            return success_response(
                data={
                    "alive": True,
                    "refreshed": False,
                    "user": {
                        "username": payload.get("username"),
                        "role": payload.get("role")
                    }
                },
                message="心跳正常"
            )
    except Exception:
        return error_response(
            code=ErrorCodes.AUTH_TOKEN_EXPIRED,
            message="认证已过期，请重新登录"
        )
