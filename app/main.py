"""FastAPI 主应用"""
from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from contextlib import asynccontextmanager

from app.config import settings
from app.database import engine, Base
from app.api import auth, control, robot, tasks, emergency, websocket, terminal, ros_gui, lift, waist, robot_model, head, arm, task_orchestration, preset, arm_points, chassis, gripper, vr_teleoperation, camera, recording, head_points, lift_points, waist_points, shutdown, actions, led
from app.ros2_bridge.bridge import ros2_bridge
from app.safety.watchdog import watchdog
from app.preset import preset_manager


@asynccontextmanager
async def lifespan(app: FastAPI):
    """应用生命周期管理"""
    # 启动时
    print("🚀 启动 FastAPI 应用...")
    
    # 初始化数据库
    Base.metadata.create_all(bind=engine)
    
    # 创建默认管理员（如果不存在）
    from app.database import SessionLocal
    from app.models.user import User
    from app.core.security import get_password_hash
    
    db = SessionLocal()
    admin = db.query(User).filter(User.username == "admin").first()
    if not admin:
        admin = User(
            username="admin",
            email="admin@example.com",
            hashed_password=get_password_hash("admin123"),
            role="admin"
        )
        db.add(admin)
        db.commit()
        print("✅ 创建默认管理员: admin / admin123")
    db.close()
    
    # 启动 ROS2 Bridge
    ros2_bridge.start()
    
    # 启动看门狗
    watchdog.start()
    
    yield
    
    # 关闭时
    print("🛑 关闭 FastAPI 应用...")
    watchdog.stop()
    ros2_bridge.shutdown()


app = FastAPI(
    title=settings.APP_NAME,
    version=settings.APP_VERSION,
    lifespan=lifespan
)

# CORS 中间件
app.add_middleware(
    CORSMiddleware,
    allow_origins=settings.cors_origins_list,
    allow_origin_regex=r"https?://.*",
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# 注册路由 - 统一使用 /api/v1 前缀
# 认证相关
app.include_router(auth.router, prefix="/api/v1/auth", tags=["认证"])
app.include_router(control.router, prefix="/api/v1/control", tags=["控制权"])

# 机器人核心
app.include_router(robot.router, prefix="/api/v1/robot", tags=["机器人状态"])
app.include_router(emergency.router, prefix="/api/v1/emergency", tags=["紧急控制"])

# 任务系统
app.include_router(tasks.router, prefix="/api/v1/tasks", tags=["任务管理"])
app.include_router(task_orchestration.router, prefix="/api/v1", tags=["任务编排"])

# 机械臂
app.include_router(arm.router, prefix="/api/v1", tags=["机械臂控制"])
app.include_router(arm_points.router, prefix="/api/v1", tags=["机械臂点位"])
app.include_router(gripper.router, prefix="/api/v1", tags=["夹爪控制"])

# 底盘
app.include_router(chassis.router, prefix="/api/v1", tags=["底盘控制"])

# 其他执行器
app.include_router(lift.router, prefix="/api/v1", tags=["升降电机控制"])
app.include_router(lift_points.router, prefix="/api/v1", tags=["升降点位"])
app.include_router(waist.router, prefix="/api/v1", tags=["腰部电机控制"])
app.include_router(waist_points.router, prefix="/api/v1", tags=["腰部点位"])
app.include_router(head.router, prefix="/api/v1", tags=["头部控制"])
app.include_router(head_points.router, prefix="/api/v1", tags=["头部点位"])
app.include_router(led.router, prefix="/api/v1", tags=["LED灯带控制"])

# 传感器 & 视觉
app.include_router(camera.router, prefix="/api/v1", tags=["相机视频流"])

# 预设 & 模型
app.include_router(preset.router, prefix="/api/v1", tags=["预设管理"])
app.include_router(robot_model.router, prefix="/api/v1/robot-model", tags=["机器人模型"])
app.include_router(actions.router, prefix="/api/v1", tags=["动作管理"])

# 数据采集
app.include_router(recording.router, prefix="/api/v1", tags=["数据录制"])

# VR & 遥操作
app.include_router(vr_teleoperation.router, prefix="/api/v1", tags=["VR遥操作"])

# 系统管理
app.include_router(ros_gui.router, prefix="/api/v1/ros-gui", tags=["ROS GUI 管理"])
app.include_router(terminal.router, prefix="/api/v1", tags=["终端（管理员）"])
app.include_router(shutdown.router, prefix="/api/v1", tags=["系统关机"])

# WebSocket (保持 /ws 前缀)
app.include_router(websocket.router, prefix="/ws", tags=["WebSocket"])

# ==================== 兼容旧路由 (deprecated, 将在 v2 移除) ====================
# 为了向后兼容，保留旧的路由前缀
app.include_router(auth.router, prefix="/api/auth", tags=["认证 (deprecated)"], deprecated=True)
app.include_router(control.router, prefix="/api/control", tags=["控制权 (deprecated)"], deprecated=True)
app.include_router(robot.router, prefix="/api/robot", tags=["机器人状态 (deprecated)"], deprecated=True)
app.include_router(tasks.router, prefix="/api/tasks", tags=["任务管理 (deprecated)"], deprecated=True)
app.include_router(emergency.router, prefix="/api/emergency", tags=["紧急控制 (deprecated)"], deprecated=True)


@app.get("/")
async def root():
    """根路径"""
    return {
        "message": "QYH Jushen Web API",
        "version": settings.APP_VERSION,
        "api_version": "v1",
        "docs": "/docs",
        "endpoints": {
            "api": "/api/v1",
            "websocket": "/ws",
            "health": "/health"
        }
    }


@app.get("/health")
async def health():
    """健康检查"""
    from datetime import datetime
    return {
        "success": True,
        "code": 0,
        "message": "服务正常",
        "data": {
            "status": "healthy",
            "ros2_connected": ros2_bridge.is_connected(),
            "database": "ok"
        },
        "timestamp": datetime.utcnow().isoformat() + "Z"
    }


@app.get("/api/v1/health")
async def health_v1():
    """健康检查 API v1"""
    from datetime import datetime
    return {
        "success": True,
        "code": 0,
        "message": "服务正常",
        "data": {
            "status": "healthy",
            "ros2_connected": ros2_bridge.is_connected(),
            "database": "ok"
        },
        "timestamp": datetime.utcnow().isoformat() + "Z"
    }


@app.get("/api/v1")
async def api_info():
    """API 版本信息"""
    return {
        "success": True,
        "code": 0,
        "message": "QYH Jushen Robot API v1",
        "data": {
            "version": "1.0.0",
            "deprecated_paths": [
                "/api/auth/* -> /api/v1/auth/*",
                "/api/control/* -> /api/v1/control/*",
                "/api/robot/* -> /api/v1/robot/*",
                "/api/tasks/* -> /api/v1/tasks/*",
                "/api/emergency/* -> /api/v1/emergency/*"
            ]
        }
    }
