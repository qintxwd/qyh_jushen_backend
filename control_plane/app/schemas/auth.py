"""
QYH Jushen Control Plane - 认证 Schema
"""
from typing import Optional, List
from pydantic import BaseModel, Field


class LoginRequest(BaseModel):
    """登录请求"""
    username: str = Field(..., min_length=1, max_length=50, description="用户名")
    password: str = Field(..., min_length=1, description="密码")


class TokenResponse(BaseModel):
    """Token 响应"""
    access_token: str = Field(..., description="访问令牌")
    token_type: str = Field(default="bearer", description="令牌类型")
    expires_in: int = Field(..., description="过期时间（秒）")


class UserInfo(BaseModel):
    """用户信息"""
    id: int
    username: str
    email: Optional[str] = None
    role: str
    is_active: bool
    
    class Config:
        from_attributes = True


class LoginResponse(BaseModel):
    """登录响应数据"""
    access_token: str
    token_type: str = "bearer"
    expires_in: int
    user: UserInfo


class UserCreate(BaseModel):
    """创建用户请求"""
    username: str = Field(..., min_length=3, max_length=50)
    password: str = Field(..., min_length=6)
    email: Optional[str] = None
    role: str = Field(default="viewer", pattern="^(admin|operator|viewer)$")


class UserUpdate(BaseModel):
    """更新用户请求"""
    email: Optional[str] = None
    role: Optional[str] = Field(None, pattern="^(admin|operator|viewer)$")
    is_active: Optional[bool] = None


class PasswordChange(BaseModel):
    """修改密码请求"""
    old_password: str
    new_password: str = Field(..., min_length=6)
