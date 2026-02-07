"""
QYH Jushen Control Plane - FastAPI 应用入口
"""
from contextlib import asynccontextmanager

from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware

from app.config import settings, DEFAULT_JWT_SECRET
from app.database import init_db, SessionLocal
from app.api.v1.router import api_router
from app.api.health import router as health_router


@asynccontextmanager
async def lifespan(app: FastAPI):
    """应用生命周期管理"""
    # ==================== 启动 ====================
    print(f"🚀 启动 {settings.APP_NAME} v{settings.APP_VERSION}...")

    if settings.SECRET_KEY == DEFAULT_JWT_SECRET and not settings.DEBUG:
        raise RuntimeError("JWT_SECRET must be set in production")
    
    # 初始化数据库
    init_db()
    print("✅ 数据库初始化完成")

    # 初始化 ROS2 客户端
    try:
        from app.services.ros2_client import get_ros2_client
        ros2 = get_ros2_client()
        await ros2.initialize()
        print("✅ ROS2 客户端初始化完成")
    except Exception as e:
        print(f"⚠️ ROS2 客户端初始化失败: {e}")
    
    # 创建默认管理员
    await create_default_admin()
    
    yield
    
    # ==================== 关闭 ====================
    print(f"🛑 关闭 {settings.APP_NAME}...")


async def create_default_admin():
    """创建默认管理员账户（如果不存在）"""
    from app.models.user import User
    from app.core.security import get_password_hash
    if not settings.AUTO_CREATE_ADMIN:
        return
    if settings.DEFAULT_ADMIN_PASSWORD == "admin123" and not settings.DEBUG:
        raise RuntimeError("DEFAULT_ADMIN_PASSWORD must be set in production")

    db = SessionLocal()
    try:
        admin = db.query(User).filter(
            User.username == settings.DEFAULT_ADMIN_USERNAME
        ).first()
        if not admin:
            admin = User(
                username=settings.DEFAULT_ADMIN_USERNAME,
                email="admin@example.com",
                hashed_password=get_password_hash(settings.DEFAULT_ADMIN_PASSWORD),
                role="admin",
            )
            db.add(admin)
            db.commit()
            print("✅ 创建默认管理员账户")
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
    allow_origin_regex=None,
    allow_credentials=settings.CORS_ORIGINS != "*",
    allow_methods=["*"],
    allow_headers=["*"],
)

# ==================== 路由注册 ====================

# API v1 路由
app.include_router(api_router, prefix="/api/v1")

# 健康检查路由
app.include_router(health_router)



# ==================== 公开路由 (地图图片) ====================
from fastapi.responses import FileResponse as PublicFileResponse
from pathlib import Path as PublicPath
import os as public_os

@app.get("/api/v1/chassis/map_image/{map_name}")
async def get_public_map_image(map_name: str):
    """获取地图图片 - 公开访问，不需要认证"""
    workspace_root = PublicPath(public_os.environ.get('QYH_WORKSPACE_ROOT', PublicPath.home() / 'qyh-robot-system'))
    maps_dir = workspace_root / "maps"
    
    # 安全检查
    if '..' in map_name or '/' in map_name or '\\' in map_name:
        raise HTTPException(status_code=400, detail="Invalid map name")
    
    map_image_file = maps_dir / map_name / f"{map_name}.png"
    if not map_image_file.exists():
        map_image_file = maps_dir / map_name / f"{map_name}.jpg"
        if not map_image_file.exists():
            raise HTTPException(status_code=404, detail="Map image not found")
    
    return PublicFileResponse(
        path=str(map_image_file),
        media_type="image/png" if map_image_file.suffix == '.png' else "image/jpeg",
        headers={
            "Access-Control-Allow-Origin": "*",
            "Access-Control-Allow-Methods": "GET",
            "Cache-Control": "public, max-age=3600"
        }
    )


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
