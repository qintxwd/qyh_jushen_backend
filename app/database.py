"""数据库配置"""
from sqlalchemy import create_engine
from sqlalchemy.ext.declarative import declarative_base
from sqlalchemy.orm import sessionmaker
from app.config import settings
import os
from pathlib import Path

# 展开 DATABASE_URL 中的 ~ 符号
database_url = settings.DATABASE_URL
if "sqlite:///" in database_url:
    # 提取路径部分
    db_path = database_url.replace("sqlite:///", "")
    # 展开用户目录
    db_path = os.path.expanduser(db_path)
    # 确保目录存在
    db_dir = os.path.dirname(db_path)
    Path(db_dir).mkdir(parents=True, exist_ok=True)
    # 重构 DATABASE_URL
    database_url = f"sqlite:///{db_path}"

# 创建引擎
connect_args = (
    {"check_same_thread": False} if "sqlite" in database_url else {}
)
engine = create_engine(database_url, connect_args=connect_args)

# 会话工厂
SessionLocal = sessionmaker(autocommit=False, autoflush=False, bind=engine)

# 基类
Base = declarative_base()


def get_db():
    """获取数据库会话（依赖注入）"""
    db = SessionLocal()
    try:
        yield db
    finally:
        db.close()
