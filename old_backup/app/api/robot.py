"""机器人状态 API"""
from fastapi import APIRouter
from fastapi.responses import Response
from app.ros2_bridge.bridge import ros2_bridge
from app.schemas.response import ApiResponse, success_response, error_response, ErrorCodes

router = APIRouter()


@router.get("/status", response_model=ApiResponse)
async def get_robot_status():
    """获取机器人当前状态
    
    Returns:
        ApiResponse: 统一响应格式，data 包含完整机器人状态
    """
    if not ros2_bridge.is_connected():
        return error_response(
            code=ErrorCodes.ROBOT_NOT_CONNECTED,
            message="ROS2 未连接"
        )
    
    state = ros2_bridge.get_robot_state()
    
    if state:
        # TODO: 补充完整的状态信息
        return success_response(
            data={
                "timestamp": state.get("timestamp"),
                "joints": state.get("joints"),
                "grippers": {
                    "left": {"position": 0.05, "force": 10.0},
                    "right": {"position": 0.08, "force": 12.0}
                },
                "base": {
                    "x": 0.0,
                    "y": 0.0,
                    "theta": 0.0,
                    "velocity": {"linear": 0.0, "angular": 0.0}
                },
                "system": {
                    "cpu_temp": 65.0,
                    "battery": 85.0,
                    "mode": "idle"
                }
            }
        )
    else:
        return success_response(
            data=None,
            message="暂无状态数据"
        )


@router.get("/urdf")
async def get_robot_urdf():
    """获取机器人 URDF 模型
    
    Returns:
        XML: URDF 文件内容 (非 JSON 响应)
    """
    # TODO: 读取实际的 URDF 文件
    urdf_content = """<?xml version="1.0"?>
<robot name="qyh_robot">
  <!-- URDF 内容 -->
  <link name="base_link"/>
</robot>"""
    
    return Response(content=urdf_content, media_type="application/xml")
