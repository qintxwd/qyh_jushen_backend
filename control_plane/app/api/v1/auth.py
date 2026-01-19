"""
QYH Jushen Control Plane - 认证 API

提供用户登录、登出、Token 刷新等功能
"""
from datetime import datetime

from fastapi import APIRouter, Depends
from fastapi.security import HTTPBearer, HTTPAuthorizationCredentials
from sqlalchemy.orm import Session

from app.database import get_db
from app.models.user import User
from app.core.security import (
    verify_password,
    create_access_token,
    should_refresh_token,
    refresh_access_token,
)
from app.config import settings
from app.dependencies import get_current_user
from app.schemas.response import ApiResponse, success_response, error_response, ErrorCodes
from app.schemas.auth import LoginRequest, LoginResponse, UserInfo

router = APIRouter()


@router.post("/login", response_model=ApiResponse)
async def login(
    request: LoginRequest,
    db: Session = Depends(get_db),
):
    """
    用户登录
    
    验证用户名和密码，成功后返回 JWT Token
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
        data={
            "sub": str(user.id),
            "username": user.username,
            "role": user.role,
        }
    )
    
    # 更新最后登录时间
    user.last_login = datetime.utcnow()
    db.commit()
    
    return success_response(
        data=LoginResponse(
            access_token=access_token,
            token_type="bearer",
            expires_in=settings.ACCESS_TOKEN_EXPIRE_MINUTES * 60,
            user=UserInfo(
                id=user.id,
                username=user.username,
                email=user.email,
                role=user.role,
                is_active=user.is_active,
            ),
        ).model_dump(),
        message="登录成功"
    )


@router.post("/logout", response_model=ApiResponse)
async def logout():
    """
    用户登出
    
    JWT 无状态，前端删除 Token 即可
    """
    return success_response(message="登出成功")


@router.get("/me", response_model=ApiResponse)
async def get_current_user_info(
    current_user: User = Depends(get_current_user),
):
    """
    获取当前用户信息
    """
    return success_response(
        data=UserInfo(
            id=current_user.id,
            username=current_user.username,
            email=current_user.email,
            role=current_user.role,
            is_active=current_user.is_active,
        ).model_dump(),
        message="获取成功"
    )


@router.post("/refresh", response_model=ApiResponse)
async def refresh_token(
    credentials: HTTPAuthorizationCredentials = Depends(HTTPBearer()),
):
    """
    刷新 Token
    
    检查当前 Token，如果接近过期则返回新的 Token
    """
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
                    "expires_in": settings.ACCESS_TOKEN_EXPIRE_MINUTES * 60,
                },
                message="Token 已刷新"
            )
        else:
            return success_response(
                data={
                    "refreshed": False,
                    "message": "Token 仍然有效，无需刷新",
                },
                message="Token 有效"
            )
    except Exception as e:
        return error_response(
            code=ErrorCodes.AUTH_TOKEN_INVALID,
            message="Token 无效或已过期"
        )
