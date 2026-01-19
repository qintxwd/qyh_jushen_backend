"""
QYH Jushen Control Plane - 配置管理

使用 Pydantic Settings 管理所有配置项，支持环境变量和 .env 文件
"""
from functools import lru_cache
from typing import List
import os

from pydantic_settings import BaseSettings, SettingsConfigDict


class Settings(BaseSettings):
    """应用配置"""
    
    model_config = SettingsConfigDict(
        env_file=".env",
        env_file_encoding="utf-8",
        case_sensitive=True,
    )
    
    # ==================== 应用信息 ====================
    APP_NAME: str = "QYH Jushen Control Plane"
    APP_VERSION: str = "2.0.0"
    DEBUG: bool = False
    
    # ==================== 服务地址 ====================
    HOST: str = "0.0.0.0"
    PORT: int = 8000
    
    # ==================== 持久化存储 ====================
    PERSISTENT_DIR: str = "~/qyh-robot-system/persistent"
    
    @property
    def persistent_dir_expanded(self) -> str:
        """展开持久化目录中的 ~ 路径"""
        path = self.PERSISTENT_DIR
        if path.startswith("~"):
            return os.path.expanduser(path)
        return path

    # ==================== 数据库 ====================
    DATABASE_URL: str = "sqlite:///~/qyh-robot-system/persistent/web/web.db"
    
    @property
    def database_url_expanded(self) -> str:
        """展开数据库 URL 中的 ~ 路径"""
        url = self.DATABASE_URL
        if "~" in url:
            # 处理 sqlite:///~/path 格式
            if url.startswith("sqlite:///~"):
                path = url[10:]  # 去掉 sqlite:///
                expanded_path = os.path.expanduser(path)
                return f"sqlite:///{expanded_path}"
        return url
    
    # ==================== JWT 认证 ====================
    SECRET_KEY: str = "your-super-secret-key-change-this-in-production"
    ALGORITHM: str = "HS256"
    ACCESS_TOKEN_EXPIRE_MINUTES: int = 30
    TOKEN_REFRESH_THRESHOLD_MINUTES: int = 10
    
    # ==================== 其他服务地址 ====================
    WEBSOCKET_SERVER_URL: str = "ws://127.0.0.1:8765"
    WEBRTC_SIGNALING_URL: str = "http://127.0.0.1:8888"
    
    # ==================== ROS2 配置 ====================
    ROS_DOMAIN_ID: int = 0
    
    # ==================== CORS ====================
    CORS_ORIGINS: str = "*"
    
    @property
    def cors_origins_list(self) -> List[str]:
        """解析 CORS 源列表"""
        if self.CORS_ORIGINS == "*":
            return ["*"]
        return [origin.strip() for origin in self.CORS_ORIGINS.split(",")]
    
    # ==================== 日志 ====================
    LOG_LEVEL: str = "INFO"
    
    # ==================== 机器人信息 ====================
    ROBOT_ID: str = "jushen-001"
    ROBOT_NAME: str = "巨神-001"


@lru_cache()
def get_settings() -> Settings:
    """获取配置单例"""
    return Settings()


# 便捷访问
settings = get_settings()
