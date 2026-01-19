"""
QYH Jushen Control Plane - API v1 路由聚合
"""
from fastapi import APIRouter

from app.api.v1 import (
    auth, system, control, mode, tasks, presets, recording,
    actions, robot, audit, emergency, chassis, led, vr
)

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

# 预设管理
api_router.include_router(presets.router, prefix="/presets", tags=["预设管理"])

# 录制管理
api_router.include_router(recording.router, prefix="/recording", tags=["录制管理"])

# 动作管理
api_router.include_router(actions.router, prefix="/actions", tags=["动作管理"])

# 机器人信息
api_router.include_router(robot.router, prefix="/robot", tags=["机器人"])

# 审计日志
api_router.include_router(audit.router, prefix="/audit", tags=["审计日志"])

# 紧急停止
api_router.include_router(emergency.router, prefix="/emergency", tags=["紧急停止"])

# 底盘配置
api_router.include_router(chassis.router, prefix="/chassis", tags=["底盘配置"])

# LED灯带控制
api_router.include_router(led.router, prefix="/led", tags=["LED控制"])

# VR遥操作状态
api_router.include_router(vr.router, prefix="/vr", tags=["VR遥操作"])

