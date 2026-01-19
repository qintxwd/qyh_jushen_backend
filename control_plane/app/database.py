"""
QYH Jushen Control Plane - 数据库连接

使用 SQLAlchemy 2.0 风格
"""
import os
from sqlalchemy import create_engine
from sqlalchemy.ext.asyncio import create_async_engine, AsyncSession, async_sessionmaker
from sqlalchemy.orm import sessionmaker, DeclarativeBase

from app.config import settings


# 确保数据库目录存在
def ensure_db_directory():
    """确保数据库目录存在"""
    db_url = settings.database_url_expanded
    if db_url.startswith("sqlite:///"):
        db_path = db_url[10:]  # 去掉 sqlite:///
        db_dir = os.path.dirname(db_path)
        if db_dir and not os.path.exists(db_dir):
            os.makedirs(db_dir, exist_ok=True)


ensure_db_directory()

# ==================== 同步连接 (用于依赖注入和 alembic) ====================
# 创建引擎
engine = create_engine(
    settings.database_url_expanded,
    connect_args={"check_same_thread": False} if "sqlite" in settings.DATABASE_URL else {},
    echo=settings.DEBUG,
)

# 创建会话工厂
SessionLocal = sessionmaker(autocommit=False, autoflush=False, bind=engine)

# ==================== 异步连接 (用于业务逻辑) ====================
# 获取异步数据库 URL
ASYNC_DATABASE_URL = settings.database_url_expanded.replace("sqlite:///", "sqlite+aiosqlite:///")

# 创建异步引擎
async_engine = create_async_engine(
    ASYNC_DATABASE_URL,
    connect_args={"check_same_thread": False} if "sqlite" in settings.DATABASE_URL else {},
    echo=settings.DEBUG,
)

# 创建异步会话工厂
AsyncSessionLocal = async_sessionmaker(
    bind=async_engine,
    class_=AsyncSession,
    autocommit=False,
    autoflush=False,
    expire_on_commit=False,
)


# 声明基类
class Base(DeclarativeBase):
    """SQLAlchemy 声明基类"""
    pass


def get_db():
    """获取数据库会话（同步，旧代码兼容）"""
    db = SessionLocal()
    try:
        yield db
    finally:
        db.close()

async def get_async_db():
    """获取异步数据库会话（推荐）"""
    async with AsyncSessionLocal() as session:
        yield session


def init_db():
    """初始化数据库（创建所有表）"""
    # 导入所有模型以注册到 Base.metadata
    from app.models import user, task, audit_log, preset, control_session  # noqa
    
    Base.metadata.create_all(bind=engine)
