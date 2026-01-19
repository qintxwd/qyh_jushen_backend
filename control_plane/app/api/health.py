"""
QYH Jushen Control Plane - 健康检查 API
"""
from fastapi import APIRouter

from app.schemas.response import ApiResponse, success_response

router = APIRouter(tags=["健康检查"])


@router.get("/health", response_model=ApiResponse)
async def health_check():
    """
    健康检查端点
    
    用于负载均衡器和监控系统检查服务状态
    """
    return success_response(
        data={"status": "healthy"},
        message="服务正常"
    )


@router.get("/ready", response_model=ApiResponse)
async def readiness_check():
    """
    就绪检查端点
    
    检查服务是否准备好接收请求
    """
    # TODO: 检查数据库连接、ROS2 连接等
    return success_response(
        data={"status": "ready"},
        message="服务就绪"
    )
