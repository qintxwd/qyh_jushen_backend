"""用户模型"""
from sqlalchemy import Column, Integer, String, Boolean, DateTime, Enum
from datetime import datetime
from app.database import Base
import enum


class UserRole(str, enum.Enum):
    """用户角色"""
    USER = "user"
    OPERATOR = "operator"
    ADMIN = "admin"


class User(Base):
    """用户表"""
    __tablename__ = "users"
    
    id = Column(Integer, primary_key=True, index=True)
    username = Column(String(50), unique=True, nullable=False, index=True)
    email = Column(String(100), unique=True, nullable=False)
    hashed_password = Column(String(255), nullable=False)
    role = Column(Enum(UserRole), default=UserRole.USER)
    is_active = Column(Boolean, default=True)
    created_at = Column(DateTime, default=datetime.utcnow)
    last_login = Column(DateTime, nullable=True)
