"""配置管理"""
from pydantic_settings import BaseSettings
from typing import List
import os


def get_ros_domain_id() -> int:
    """从配置文件读取 ROS_DOMAIN_ID，默认为 0"""
    config_file = os.path.expanduser("~/qyh-robot-system/persistent/ros/ROS_DOMAIN_ID")
    try:
        with open(config_file, 'r') as f:
            return int(f.read().strip())
    except (FileNotFoundError, ValueError):
        return 0


class Settings(BaseSettings):
    """应用配置"""
    
    # 应用信息
    APP_NAME: str = "QYH Jushen Web"
    APP_VERSION: str = "1.0.0"
    DEBUG: bool = False
    
    # 数据库
    DATABASE_URL: str = f"sqlite:///{os.path.expanduser('~/qyh-robot-system/persistent/web/web.db')}"
    
    # JWT 认证
    SECRET_KEY: str
    ALGORITHM: str = "HS256"
    ACCESS_TOKEN_EXPIRE_MINUTES: int = 30
    # Token 自动刷新阈值（剩余时间少于此值时自动刷新，单位：分钟）
    TOKEN_REFRESH_THRESHOLD_MINUTES: int = 10
    
    # ROS2 - 从文件读取
    MOCK_MODE: bool = False
    
    @property
    def ROS_DOMAIN_ID(self) -> int:
        return get_ros_domain_id()
    
    # CORS - 允许所有来源访问（支持远程访问）
    CORS_ORIGINS: str = "*"
    
    @property
    def cors_origins_list(self) -> List[str]:
        if self.CORS_ORIGINS == "*":
            return ["*"]
        return [origin.strip() for origin in self.CORS_ORIGINS.split(",")]
    
    # 日志
    LOG_LEVEL: str = "INFO"
    
    class Config:
        env_file = ".env"
        case_sensitive = True


settings = Settings()
