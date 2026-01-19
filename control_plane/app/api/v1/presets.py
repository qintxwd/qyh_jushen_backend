"""
QYH Jushen Control Plane - 预设管理 API

提供预设的 CRUD 操作和执行接口
整合了机械臂点位、头部点位、升降高度、腰部角度等预设管理
"""
from typing import Optional
from fastapi import APIRouter, Depends, Query

from app.dependencies import get_current_user, get_current_operator
from app.models.user import User
from app.services.preset_manager import preset_manager
from app.services.ros2_client import get_ros2_client, RobotSide, LiftCommand, WaistCommand
from app.schemas.preset import (
    PresetType,
    CreatePresetRequest,
    UpdatePresetRequest,
    CapturePresetRequest,
    ApplyPresetRequest,
    PresetInfo,
    PresetListResponse,
    PresetTypeInfo,
)
from app.schemas.response import (
    ApiResponse, success_response, error_response, ErrorCodes
)

router = APIRouter()


# ==================== 预设类型信息 ====================

PRESET_TYPE_INFO = {
    PresetType.ARM_POSE: {
        "name": "机械臂点位",
        "description": "机械臂关节角度或末端位姿预设",
        "data_fields": ["side", "pose_type", "left_joints", "right_joints", "velocity", "acceleration"],
    },
    PresetType.HEAD_POSITION: {
        "name": "头部点位",
        "description": "头部云台角度预设",
        "data_fields": ["pan", "tilt"],
    },
    PresetType.LIFT_HEIGHT: {
        "name": "升降高度",
        "description": "升降柱高度预设",
        "data_fields": ["height"],
    },
    PresetType.WAIST_ANGLE: {
        "name": "腰部角度",
        "description": "腰部前倾角度预设",
        "data_fields": ["angle"],
    },
    PresetType.LOCATION: {
        "name": "导航点位",
        "description": "底盘导航目标点预设",
        "data_fields": ["x", "y", "theta", "frame_id", "station_id"],
    },
    PresetType.GRIPPER_POSITION: {
        "name": "夹爪位置",
        "description": "夹爪开合位置预设",
        "data_fields": ["side", "left_position", "right_position", "force"],
    },
    PresetType.FULL_POSE: {
        "name": "完整姿态",
        "description": "机器人完整姿态预设（多部件组合）",
        "data_fields": ["arm", "head", "lift", "waist", "gripper"],
    },
}


@router.get("/types", response_model=ApiResponse)
async def get_preset_types():
    """
    获取所有预设类型
    """
    types = []
    for preset_type, info in PRESET_TYPE_INFO.items():
        types.append({
            "type": preset_type.value,
            "name": info["name"],
            "description": info["description"],
            "data_fields": info["data_fields"],
        })
    
    return success_response(
        data={"types": types},
        message="获取预设类型成功"
    )


@router.get("", response_model=ApiResponse)
async def list_presets(
    preset_type: PresetType = Query(..., description="预设类型"),
    category: Optional[str] = Query(None, description="分类过滤"),
    include_builtin: bool = Query(True, description="是否包含内置预设"),
    current_user: User = Depends(get_current_user),
):
    """
    列出指定类型的预设
    """
    # 特殊处理: location 类型从地图读取站点
    if preset_type == PresetType.LOCATION:
        items = preset_manager.get_locations_from_map()
        # 合并用户自定义的导航点
        user_items = preset_manager.list(preset_type, category, include_builtin)
        items.extend(user_items)
    else:
        items = preset_manager.list(preset_type, category, include_builtin)
    
    return success_response(
        data={
            "preset_type": preset_type.value,
            "total": len(items),
            "items": items,
        },
        message="获取预设列表成功"
    )


@router.post("", response_model=ApiResponse)
async def create_preset(
    request: CreatePresetRequest,
    current_user: User = Depends(get_current_operator),
):
    """
    创建新预设
    """
    try:
        preset = preset_manager.create(
            preset_type=request.preset_type,
            name=request.name,
            data=request.data,
            description=request.description,
            category=request.category,
        )
        
        return success_response(
            data=preset,
            message="预设创建成功"
        )
    except ValueError as e:
        return error_response(
            code=ErrorCodes.VALIDATION_ERROR,
            message=str(e)
        )
    except Exception as e:
        return error_response(
            code=ErrorCodes.INTERNAL_ERROR,
            message=f"创建预设失败: {str(e)}"
        )


@router.get("/{preset_type}/{preset_id}", response_model=ApiResponse)
async def get_preset(
    preset_type: PresetType,
    preset_id: str,
    current_user: User = Depends(get_current_user),
):
    """
    获取预设详情
    """
    preset = preset_manager.get(preset_type, preset_id)
    
    if not preset:
        # 尝试通过名称查找
        preset = preset_manager.get_by_name(preset_type, preset_id)
    
    if not preset:
        return error_response(
            code=ErrorCodes.NOT_FOUND,
            message=f"预设 '{preset_id}' 不存在"
        )
    
    return success_response(
        data=preset,
        message="获取预设成功"
    )


@router.put("/{preset_type}/{preset_id}", response_model=ApiResponse)
async def update_preset(
    preset_type: PresetType,
    preset_id: str,
    request: UpdatePresetRequest,
    current_user: User = Depends(get_current_operator),
):
    """
    更新预设
    """
    try:
        preset = preset_manager.update(
            preset_type=preset_type,
            preset_id=preset_id,
            name=request.name,
            description=request.description,
            category=request.category,
            data=request.data,
        )
        
        return success_response(
            data=preset,
            message="预设更新成功"
        )
    except ValueError as e:
        return error_response(
            code=ErrorCodes.VALIDATION_ERROR,
            message=str(e)
        )
    except Exception as e:
        return error_response(
            code=ErrorCodes.INTERNAL_ERROR,
            message=f"更新预设失败: {str(e)}"
        )


@router.delete("/{preset_type}/{preset_id}", response_model=ApiResponse)
async def delete_preset(
    preset_type: PresetType,
    preset_id: str,
    current_user: User = Depends(get_current_operator),
):
    """
    删除预设
    
    内置预设不可删除
    """
    try:
        success = preset_manager.delete(preset_type, preset_id)
        
        if not success:
            return error_response(
                code=ErrorCodes.NOT_FOUND,
                message=f"预设 '{preset_id}' 不存在"
            )
        
        return success_response(message="预设删除成功")
    except ValueError as e:
        return error_response(
            code=ErrorCodes.VALIDATION_ERROR,
            message=str(e)
        )


@router.post("/{preset_type}/{preset_id}/apply", response_model=ApiResponse)
async def apply_preset(
    preset_type: PresetType,
    preset_id: str,
    request: ApplyPresetRequest,
    current_user: User = Depends(get_current_operator),
):
    """
    应用预设
    
    通过 ROS2 Service 将机器人移动到预设位置
    
    ⚠️ 注意：这是一次性动作命令，不是持续控制
    """
    preset = preset_manager.get(preset_type, preset_id)
    
    if not preset:
        preset = preset_manager.get_by_name(preset_type, preset_id)
    
    if not preset:
        return error_response(
            code=ErrorCodes.NOT_FOUND,
            message=f"预设 '{preset_id}' 不存在"
        )
    
    # 获取 ROS2 客户端
    ros2_client = get_ros2_client()
    result = None
    
    try:
        if preset_type == PresetType.ARM_POSE:
            # 机械臂预设 - 使用 MoveJ 服务
            data = preset.get("data", {})
            left_joints = data.get("left_joints", [0.0] * 7)
            right_joints = data.get("right_joints", [0.0] * 7)
            velocity = data.get("velocity", 0.5)
            acceleration = data.get("acceleration", 0.3)
            
            # 合并为 14 个关节位置
            joint_positions = left_joints + right_joints
            
            # 确定使用哪只手
            side_str = data.get("side", "dual")
            if side_str == "left":
                robot_side = RobotSide.LEFT
            elif side_str == "right":
                robot_side = RobotSide.RIGHT
            else:
                robot_side = RobotSide.DUAL
            
            result = await ros2_client.arm_move_j(
                joint_positions=joint_positions,
                robot_side=robot_side,
                velocity=velocity,
                acceleration=acceleration,
                is_block=True  # 阻塞等待完成
            )
            
        elif preset_type == PresetType.LIFT_HEIGHT:
            # 升降预设 - 使用 LiftControl 服务
            data = preset.get("data", {})
            height = data.get("height", 0.0)
            
            result = await ros2_client.lift_go_position(
                position=height,
                hold=True
            )
            
        elif preset_type == PresetType.WAIST_ANGLE:
            # 腰部预设 - 使用 WaistControl 服务
            data = preset.get("data", {})
            angle = data.get("angle", 0.0)
            
            result = await ros2_client.waist_go_angle(
                angle=angle,
                hold=True
            )
            
        elif preset_type == PresetType.HEAD_POSITION:
            # 头部预设 - 暂时不支持完整的位置控制
            # 头部只有 enable_torque 服务，没有位置控制服务
            # TODO: 需要头部位置控制服务
            return success_response(
                data={
                    "preset_id": preset_id,
                    "preset_name": preset.get("name"),
                    "applied": False,
                    "message": "头部预设暂不支持自动应用（需要头部位置控制服务）"
                },
                message=f"头部预设 '{preset.get('name')}' 暂不支持自动应用"
            )
            
        else:
            return error_response(
                code=ErrorCodes.VALIDATION_ERROR,
                message=f"不支持应用 {preset_type.value} 类型的预设"
            )
        
        # 检查 ROS2 调用结果
        if result and result.success:
            return success_response(
                data={
                    "preset_id": preset_id,
                    "preset_name": preset.get("name"),
                    "applied": True,
                    "ros2_mock": ros2_client.is_mock_mode,
                    "message": result.message
                },
                message=f"预设 '{preset.get('name')}' 应用成功"
            )
        else:
            error_msg = result.message if result else "ROS2 服务调用失败"
            return error_response(
                code=ErrorCodes.OPERATION_FAILED,
                message=f"应用预设失败: {error_msg}"
            )
            
    except Exception as e:
        return error_response(
            code=ErrorCodes.INTERNAL_ERROR,
            message=f"应用预设时发生错误: {str(e)}"
        )


@router.post("/capture", response_model=ApiResponse)
async def capture_current_state(
    request: CapturePresetRequest,
    current_user: User = Depends(get_current_operator),
):
    """
    采集当前状态为新预设
    
    从 ROS2 读取机器人当前状态，创建新预设
    
    注意：当前状态采集需要 ROS2 Topic 订阅，
    在 ROS2 环境不可用时会返回模拟数据。
    """
    ros2_client = get_ros2_client()
    captured_data = None
    is_mock_data = ros2_client.is_mock_mode
    
    try:
        # 尝试从 ROS2 获取当前状态
        if request.preset_type == PresetType.ARM_POSE:
            # TODO: 需要订阅 /jaka/joint_states 或类似 topic
            # 暂时使用模拟数据
            captured_data = {
                "side": request.side or "dual",
                "pose_type": "joint",
                "left_joints": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                "right_joints": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                "velocity": 0.5,
                "acceleration": 0.3,
            }
            is_mock_data = True  # 强制标记为 mock，因为 topic 订阅未实现
            
        elif request.preset_type == PresetType.HEAD_POSITION:
            # TODO: 需要订阅头部状态 topic
            captured_data = {
                "pan": 0.0,
                "tilt": 0.0,
            }
            is_mock_data = True
            
        elif request.preset_type == PresetType.LIFT_HEIGHT:
            # TODO: 需要订阅 /lift/status 或类似 topic
            captured_data = {
                "height": 0.0,
            }
            is_mock_data = True
            
        elif request.preset_type == PresetType.WAIST_ANGLE:
            # TODO: 需要订阅 /waist/status 或类似 topic
            captured_data = {
                "angle": 0.0,
            }
            is_mock_data = True
            
        else:
            return error_response(
                code=ErrorCodes.VALIDATION_ERROR,
                message=f"不支持采集 {request.preset_type.value} 类型的预设"
            )
        
        # 创建预设
        preset = preset_manager.create(
            preset_type=request.preset_type,
            name=request.name,
            data=captured_data,
            description=request.description or "采集自当前状态",
            category="captured",
        )
        
        message_suffix = "（使用模拟数据，ROS2 Topic 订阅待实现）" if is_mock_data else ""
        
        return success_response(
            data={
                **preset,
                "is_mock_data": is_mock_data,
            },
            message=f"已采集当前状态为预设 '{request.name}'{message_suffix}"
        )
        
    except ValueError as e:
        return error_response(
            code=ErrorCodes.VALIDATION_ERROR,
            message=str(e)
        )
    except Exception as e:
        return error_response(
            code=ErrorCodes.INTERNAL_ERROR,
            message=f"采集预设时发生错误: {str(e)}"
        )


# ==================== 兼容旧 API 的快捷端点 ====================

@router.get("/arm/points", response_model=ApiResponse)
async def list_arm_points(
    category: Optional[str] = None,
    current_user: User = Depends(get_current_user),
):
    """
    列出机械臂点位（兼容旧 API）
    """
    items = preset_manager.list(PresetType.ARM_POSE, category, True)
    return success_response(
        data={"points": items, "total": len(items)},
        message="获取机械臂点位成功"
    )


@router.get("/head/points", response_model=ApiResponse)
async def list_head_points(
    category: Optional[str] = None,
    current_user: User = Depends(get_current_user),
):
    """
    列出头部点位（兼容旧 API）
    """
    items = preset_manager.list(PresetType.HEAD_POSITION, category, True)
    return success_response(
        data={"points": items, "total": len(items)},
        message="获取头部点位成功"
    )


@router.get("/lift/points", response_model=ApiResponse)
async def list_lift_points(
    category: Optional[str] = None,
    current_user: User = Depends(get_current_user),
):
    """
    列出升降点位（兼容旧 API）
    """
    items = preset_manager.list(PresetType.LIFT_HEIGHT, category, True)
    return success_response(
        data={"points": items, "total": len(items)},
        message="获取升降点位成功"
    )


@router.get("/waist/points", response_model=ApiResponse)
async def list_waist_points(
    category: Optional[str] = None,
    current_user: User = Depends(get_current_user),
):
    """
    列出腰部点位（兼容旧 API）
    """
    items = preset_manager.list(PresetType.WAIST_ANGLE, category, True)
    return success_response(
        data={"points": items, "total": len(items)},
        message="获取腰部点位成功"
    )
