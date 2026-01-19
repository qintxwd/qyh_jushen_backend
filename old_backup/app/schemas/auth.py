"""认证相关 Schema"""
from pydantic import BaseModel, EmailStr
from datetime import datetime


class LoginRequest(BaseModel):
    """登录请求"""
    username: str
    password: str


class UserInfo(BaseModel):
    """用户信息"""
    id: int
    username: str
    role: str
    email: EmailStr
    
    class Config:
        from_attributes = True


class LoginResponse(BaseModel):
    """登录响应"""
    access_token: str
    token_type: str = "bearer"
    expires_in: int
    user: UserInfo
