"""
QYH Jushen Control Plane - API v1 路由聚合
"""
from fastapi import APIRouter

from app.api.v1 import auth, system, control, mode, tasks

api_router = APIRouter()

# 认证相关
api_router.include_router(auth.router, prefix="/auth", tags=["认证"])

# 系统配置
api_router.include_router(system.router, prefix="/system", tags=["系统配置"])

# 控制权管理
api_router.include_router(control.router, prefix="/control", tags=["控制权"])

# 模式管理
api_router.include_router(mode.router, prefix="/mode", tags=["工作模式"])

# 任务管理
api_router.include_router(tasks.router, prefix="/tasks", tags=["任务管理"])
