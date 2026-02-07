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
from app.services.ros2_client import get_ros2_client, get_ros2_client_dependency, RobotSide, ROS2ServiceClient
from app.schemas.preset import (
    PresetType,
    CreatePresetRequest,
    UpdatePresetRequest,
    CapturePresetRequest,
    ApplyPresetRequest,
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
        "data_fields": [
            "side",
            "pose_type",
            "left_joints",
            "right_joints",
            "velocity",
            "acceleration",
        ],
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
        user_items = preset_manager.list(
            preset_type,
            category,
            include_builtin,
        )
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
    ros2_client: ROS2ServiceClient = Depends(get_ros2_client_dependency),
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
    
    result = None
    
    try:
        if preset_type == PresetType.ARM_POSE:
            # 机械臂预设 - 使用 MoveJ 服务
            # 兼容性处理：即支持根字段也支持 data 嵌套字段
            src = preset
            if "left_joints" not in preset and "data" in preset and isinstance(preset["data"], dict):
                src = preset.get("data", {})

            left_joints = src.get("left_joints", [0.0] * 7)
            right_joints = src.get("right_joints", [0.0] * 7)
            velocity = src.get("velocity", 0.5)
            acceleration = src.get("acceleration", 0.3)
            
            # 合并为 14 个关节位置
            joint_positions = left_joints + right_joints
            
            # 确定使用哪只手
            side_str = src.get("side", "dual")
            if side_str == "left":
                robot_side = RobotSide.LEFT
            elif side_str == "right":
                robot_side = RobotSide.RIGHT
            else:
                robot_side = RobotSide.DUAL
            
            result = await ros2_client.arm_move_j(
                joint_positions=joint_positions,
                robot_id=robot_side,
                velocity=velocity,
                acceleration=acceleration,
                is_block=True  # 阻塞等待完成
            )
            
        elif preset_type == PresetType.LIFT_HEIGHT:
            # 升降预设 - 使用 LiftControl 服务
            # 兼容性处理
            src = preset
            if "height" not in preset and "data" in preset and isinstance(preset["data"], dict):
                src = preset.get("data", {})
                
            height = src.get("height", 0.0)
            
            result = await ros2_client.lift_go_position(
                position=height,
                hold=True
            )
            
        elif preset_type == PresetType.WAIST_ANGLE:
            # 腰部预设 - 使用 WaistControl 服务
            # 兼容性处理
            src = preset
            if "angle" not in preset and "data" in preset and isinstance(preset["data"], dict):
                src = preset.get("data", {})
                
            angle = src.get("angle", 0.0)
            
            result = await ros2_client.waist_go_angle(
                angle=angle,
                hold=True
            )
            
        elif preset_type == PresetType.HEAD_POSITION:
            # 头部预设
            src = preset
            if "pan" not in preset and "data" in preset and isinstance(preset["data"], dict):
                src = preset.get("data", {})
                
            pan = src.get("pan", 0.0)
            tilt = src.get("tilt", 0.0)
            speed = src.get("speed", 50.0)
            
            result = await ros2_client.head_move(
                pan=float(pan),
                tilt=float(tilt),
                speed=float(speed)
            )

        elif preset_type == PresetType.GRIPPER_POSITION:
            # 夹爪预设 - 使用 MoveGripper 服务
            src = preset
            if "side" not in preset and "data" in preset and isinstance(preset["data"], dict):
                src = preset.get("data", {})
                
            side = src.get("side", "left")
            force = int(src.get("force", 50))
            
            error_msgs = []
            
            # 左手
            if side in ("left", "dual", "both"):
                pos = int(src.get("left_position", 0))
                res = await ros2_client.gripper_move("left", pos, 100, force)
                if not res.success:
                    error_msgs.append(f"Left: {res.message}")
            
            # 右手
            if side in ("right", "dual", "both"):
                pos = int(src.get("right_position", 0))
                res = await ros2_client.gripper_move("right", pos, 100, force)
                if not res.success:
                    error_msgs.append(f"Right: {res.message}")
            
            if error_msgs:
                # 构造一个失败对象
                class SimpleResult:
                    success = False
                    message = "; ".join(error_msgs)
                result = SimpleResult()
            else:
                # 构造一个成功对象
                class SimpleResult:
                    success = True
                    message = "夹爪动作已执行"
                result = SimpleResult()
            
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
    """
    ros2_client = get_ros2_client()
    captured_data = None
    
    if not ros2_client._initialized:
        await ros2_client.initialize()
    
    state = ros2_client.get_robot_state()
    if not state:
        return error_response(
            code=ErrorCodes.OPERATION_FAILED,
            message="未获取到 ROS2 状态，请检查话题订阅",
        )
    
    try:
        # 尝试从 ROS2 获取当前状态
        if request.preset_type == PresetType.ARM_POSE:
            side = request.side or "dual"
            left_joints = state.get("left_arm", {}).get("joints", [])
            right_joints = state.get("right_arm", {}).get("joints", [])

            def _positions(joints):
                return [j.get("position", 0.0) for j in joints]

            left_positions = _positions(left_joints) if left_joints else []
            right_positions = _positions(right_joints) if right_joints else []

            if side in ("left", "dual") and not left_positions:
                return error_response(
                    code=ErrorCodes.OPERATION_FAILED,
                    message="未获取到左臂关节状态",
                )
            if side in ("right", "dual") and not right_positions:
                return error_response(
                    code=ErrorCodes.OPERATION_FAILED,
                    message="未获取到右臂关节状态",
                )

            captured_data = {
                "side": side,
                "pose_type": "joint",
                "left_joints": left_positions,
                "right_joints": right_positions,
                "velocity": 0.5,
                "acceleration": 0.3,
            }
            
        elif request.preset_type == PresetType.HEAD_POSITION:
            head_joints = state.get("head", {}).get("joints", [])
            if not head_joints:
                return error_response(
                    code=ErrorCodes.OPERATION_FAILED,
                    message="未获取到头部关节状态",
                )

            def _find_joint(joints, keywords):
                for joint in joints:
                    name = str(joint.get("name", "")).lower()
                    if any(k in name for k in keywords):
                        return joint.get("position", 0.0)
                return None

            pan = _find_joint(head_joints, ["pan", "yaw"])
            tilt = _find_joint(head_joints, ["tilt", "pitch"])

            if pan is None or tilt is None:
                positions = [j.get("position", 0.0) for j in head_joints]
                if pan is None and positions:
                    pan = positions[0]
                if tilt is None:
                    tilt = positions[1] if len(positions) > 1 else 0.0

            captured_data = {
                "pan": pan or 0.0,
                "tilt": tilt or 0.0,
            }
            
        elif request.preset_type == PresetType.LIFT_HEIGHT:
            lift_state = state.get("lift", {})
            if not lift_state:
                return error_response(
                    code=ErrorCodes.OPERATION_FAILED,
                    message="未获取到升降状态",
                )
            captured_data = {
                "height": float(lift_state.get("current_position", 0.0)),
            }
            
        elif request.preset_type == PresetType.WAIST_ANGLE:
            waist_state = state.get("waist", {})
            if not waist_state:
                return error_response(
                    code=ErrorCodes.OPERATION_FAILED,
                    message="未获取到腰部状态",
                )
            angle = waist_state.get("current_angle")
            if angle is None:
                angle = waist_state.get("current_position", 0.0)
            captured_data = {
                "angle": float(angle),
            }

        elif request.preset_type == PresetType.GRIPPER_POSITION:
            side = request.side or "dual"
            
            left_state = state.get("left_gripper", {})
            right_state = state.get("right_gripper", {})
            
            captured_data = {
                "side": side,
                "force": 50, # 默认力度
            }
            
            # get_robot_state 返回 normalized position (0.0-1.0)
            if side in ("left", "dual"):
                pos = left_state.get("position", 0.0)
                captured_data["left_position"] = int(pos * 255)
                
            if side in ("right", "dual"):
                pos = right_state.get("position", 0.0)
                captured_data["right_position"] = int(pos * 255)
            
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
        
        return success_response(
            data={
                **preset,
            },
            message=f"已采集当前状态为预设 '{request.name}'"
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
