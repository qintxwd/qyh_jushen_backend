"""
QYH Jushen Control Plane - FastAPI 应用入口
"""
from contextlib import asynccontextmanager

from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware

from app.config import settings
from app.database import init_db, SessionLocal
from app.api.v1.router import api_router
from app.api.health import router as health_router


@asynccontextmanager
async def lifespan(app: FastAPI):
    """应用生命周期管理"""
    # ==================== 启动 ====================
    print(f"🚀 启动 {settings.APP_NAME} v{settings.APP_VERSION}...")
    
    # 初始化数据库
    init_db()
    print("✅ 数据库初始化完成")
    
    # 创建默认管理员
    await create_default_admin()
    
    yield
    
    # ==================== 关闭 ====================
    print(f"🛑 关闭 {settings.APP_NAME}...")


async def create_default_admin():
    """创建默认管理员账户（如果不存在）"""
    from app.models.user import User
    from app.core.security import get_password_hash
    
    db = SessionLocal()
    try:
        admin = db.query(User).filter(User.username == "admin").first()
        if not admin:
            admin = User(
                username="admin",
                email="admin@example.com",
                hashed_password=get_password_hash("admin123"),
                role="admin",
            )
            db.add(admin)
            db.commit()
            print("✅ 创建默认管理员: admin / admin123")
        else:
            print("ℹ️  管理员账户已存在")
    finally:
        db.close()


# 创建 FastAPI 应用
app = FastAPI(
    title=settings.APP_NAME,
    version=settings.APP_VERSION,
    description="具身智能机器人后端 V2 - 管理平面 API",
    lifespan=lifespan,
    docs_url="/docs",
    redoc_url="/redoc",
    openapi_url="/openapi.json",
)

# ==================== 中间件 ====================

# CORS 中间件
app.add_middleware(
    CORSMiddleware,
    allow_origins=settings.cors_origins_list,
    allow_origin_regex=r"https?://.*",
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# ==================== 路由注册 ====================

# API v1 路由
app.include_router(api_router, prefix="/api/v1")

# 健康检查路由
app.include_router(health_router)


# ==================== 根路由 ====================

@app.get("/", tags=["Root"])
async def root():
    """根路径 - 返回 API 基本信息"""
    return {
        "name": settings.APP_NAME,
        "version": settings.APP_VERSION,
        "api_version": "v1",
        "docs": "/docs",
        "redoc": "/redoc",
        "health": "/health",
        "endpoints": {
            "api": "/api/v1",
            "auth": "/api/v1/auth",
            "system": "/api/v1/system",
            "control": "/api/v1/control",
            "mode": "/api/v1/mode",
            "tasks": "/api/v1/tasks",
        },
    }
