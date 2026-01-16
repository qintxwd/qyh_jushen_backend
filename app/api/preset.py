"""
预设管理 API (Preset API)

提供预设的 CRUD 操作和快捷执行接口
"""

from fastapi import APIRouter, Depends, Query, HTTPException
from typing import Optional, List, Dict, Any
from pydantic import BaseModel, Field

from app.dependencies import get_current_admin
from app.preset import (
    preset_manager, PresetType,
    Location, ArmPose, LiftHeight, HeadPosition, GripperPosition, TaskTemplate
)
from app.ros2_bridge.bridge import ros2_bridge
from app.safety.watchdog import watchdog
from app.schemas.response import (
    ApiResponse, success_response, error_response, ErrorCodes
)

router = APIRouter()


# ==================== 请求/响应模型 ====================

class PresetCreateRequest(BaseModel):
    """创建预设请求"""
    name: str = Field(..., description="预设名称")
    description: str = Field(default="", description="描述")
    category: str = Field(default="custom", description="分类")
    data: Dict[str, Any] = Field(..., description="预设数据")


class PresetUpdateRequest(BaseModel):
    """更新预设请求"""
    name: Optional[str] = None
    description: Optional[str] = None
    category: Optional[str] = None
    data: Optional[Dict[str, Any]] = None


class PresetResponse(BaseModel):
    """预设响应"""
    success: bool
    message: str
    data: Optional[Dict[str, Any]] = None


class CaptureRequest(BaseModel):
    """采集当前状态请求"""
    name: str = Field(..., description="预设名称")
    description: str = Field(default="", description="描述")
    category: str = Field(default="captured", description="分类")
    side: Optional[str] = Field(default=None, description="适用侧: left, right, both")


class ExecutePresetRequest(BaseModel):
    """执行预设请求"""
    preset_id: str = Field(..., description="预设ID或名称")
    override_params: Dict[str, Any] = Field(default_factory=dict, description="覆盖参数")


# ==================== 通用预设端点 ====================

@router.get("/presets/{preset_type}")
async def list_presets(
    preset_type: PresetType,
    category: Optional[str] = None,
    include_builtin: bool = True,
    current_user=Depends(get_current_admin)
):
    """获取预设列表"""
    import json
    from pathlib import Path
    
    # 特殊处理: location 从地图数据读取站点
    if preset_type == PresetType.LOCATION:
        workspace_root = Path(__file__).parent.parent.parent.parent.parent
        maps_dir = workspace_root / "maps"
        
        # 读取当前地图名
        current_map_file = maps_dir / "current_map.txt"
        if current_map_file.exists():
            current_map = current_map_file.read_text(encoding='utf-8').strip()
            if current_map:
                map_json_file = maps_dir / current_map / f"{current_map}.json"
                if map_json_file.exists():
                    try:
                        with open(map_json_file, 'r', encoding='utf-8') as f:
                            map_data = json.load(f)
                        stations = map_data.get('data', {}).get('station', [])
                        # 转换为预设格式
                        items = []
                        for station in stations:
                            items.append({
                                "id": f"station_{station['id']}",
                                "name": station.get('name', f"站点{station['id']}"),
                                "description": f"地图站点 (ID: {station['id']})",
                                "category": "map",
                                "x": station.get('pos.x', 0.0),
                                "y": station.get('pos.y', 0.0),
                                "theta": station.get('pos.yaw', 0.0) / 1000.0,  # 转换为弧度
                                "station_id": station['id']  # 保留原始站点ID用于导航
                            })
                        return {
                            "type": preset_type.value,
                            "total": len(items),
                            "items": items
                        }
                    except Exception as e:
                        print(f"Failed to load map stations: {e}")
        # 如果地图数据获取失败，返回空列表
        return {
            "type": preset_type.value,
            "total": 0,
            "items": []
        }
    
    # 特殊处理: arm_pose 从 arm_points.json 读取
    if preset_type == PresetType.ARM_POSE:
        arm_points_file = Path.home() / "qyh-robot-system" / "persistent" / "preset" / "arm_points.json"
        if arm_points_file.exists():
            try:
                with open(arm_points_file, 'r', encoding='utf-8') as f:
                    data = json.load(f)
                points = data.get('points', [])
                # 转换格式为预设格式
                items = []
                for point in points:
                    items.append({
                        "id": point["id"],
                        "name": point["name"],
                        "description": point.get("description", ""),
                        "category": "user" if not point.get("is_builtin") else "builtin",
                        "side": "both",
                        "left_joints": point.get("left_joints", [0.0] * 7),
                        "right_joints": point.get("right_joints", [0.0] * 7),
                    })
                return {
                    "type": preset_type.value,
                    "total": len(items),
                    "items": items
                }
            except Exception as e:
                print(f"Failed to load arm_points.json: {e}")
    
    items = preset_manager.list(preset_type, category, include_builtin)
    return {
        "type": preset_type.value,
        "total": len(items),
        "items": [item.model_dump() for item in items]
    }


@router.get("/presets/{preset_type}/{preset_id}")
async def get_preset(
    preset_type: PresetType,
    preset_id: str,
    current_user=Depends(get_current_admin)
):
    """获取单个预设"""
    preset = preset_manager.get(preset_type, preset_id)
    if not preset:
        # 尝试通过名称查找
        preset = preset_manager.get_by_name(preset_type, preset_id)
    
    if not preset:
        raise HTTPException(status_code=404, detail="预设不存在")
    
    return preset.model_dump()


@router.post("/presets/{preset_type}")
async def create_preset(
    preset_type: PresetType,
    request: PresetCreateRequest,
    current_user=Depends(get_current_admin)
):
    """创建预设"""
    try:
        data = {
            "name": request.name,
            "description": request.description,
            "category": request.category,
            **request.data
        }
        preset = preset_manager.create(preset_type, data)
        return PresetResponse(
            success=True,
            message=f"预设 '{request.name}' 创建成功",
            data=preset.model_dump()
        )
    except Exception as e:
        raise HTTPException(status_code=400, detail=str(e))


@router.put("/presets/{preset_type}/{preset_id}")
async def update_preset(
    preset_type: PresetType,
    preset_id: str,
    request: PresetUpdateRequest,
    current_user=Depends(get_current_admin)
):
    """更新预设"""
    try:
        update_data = {}
        if request.name is not None:
            update_data["name"] = request.name
        if request.description is not None:
            update_data["description"] = request.description
        if request.category is not None:
            update_data["category"] = request.category
        if request.data is not None:
            update_data.update(request.data)
        
        preset = preset_manager.update(preset_type, preset_id, update_data)
        if not preset:
            raise HTTPException(status_code=404, detail="预设不存在")
        
        return PresetResponse(
            success=True,
            message="预设更新成功",
            data=preset.model_dump()
        )
    except ValueError as e:
        raise HTTPException(status_code=400, detail=str(e))


@router.delete("/presets/{preset_type}/{preset_id}")
async def delete_preset(
    preset_type: PresetType,
    preset_id: str,
    current_user=Depends(get_current_admin)
):
    """删除预设"""
    try:
        success = preset_manager.delete(preset_type, preset_id)
        if not success:
            raise HTTPException(status_code=404, detail="预设不存在")
        
        return PresetResponse(success=True, message="预设删除成功")
    except ValueError as e:
        raise HTTPException(status_code=400, detail=str(e))


@router.get("/presets/{preset_type}/categories")
async def get_categories(
    preset_type: PresetType,
    current_user=Depends(get_current_admin)
):
    """获取分类列表"""
    categories = preset_manager.get_categories(preset_type)
    return {"categories": categories}


# ==================== 升降高度专用端点 ====================

@router.get("/lift/presets")
async def list_lift_presets(
    category: Optional[str] = None,
    current_user=Depends(get_current_admin)
):
    """获取升降高度预设列表"""
    items = preset_manager.list(PresetType.LIFT_HEIGHT, category)
    return {
        "total": len(items),
        "items": [item.model_dump() for item in items]
    }


@router.post("/lift/presets/capture")
async def capture_lift_height(
    request: CaptureRequest,
    current_user=Depends(get_current_admin)
):
    """采集当前升降高度为预设"""
    watchdog.heartbeat()
    
    # 获取当前升降机状态
    lift_state = ros2_bridge.get_lift_state()
    if not lift_state and not ros2_bridge.mock_mode:
        raise HTTPException(status_code=400, detail="无法获取升降机状态")
    
    # Mock 模式下使用模拟值
    current_height = lift_state.get("current_position", 100.0) if lift_state else 100.0
    
    data = {
        "name": request.name,
        "description": request.description,
        "category": request.category,
        "height": current_height
    }
    
    preset = preset_manager.create(PresetType.LIFT_HEIGHT, data)
    return PresetResponse(
        success=True,
        message=f"升降高度 '{request.name}' ({current_height}mm) 已保存",
        data=preset.model_dump()
    )


@router.post("/lift/presets/{preset_id}/execute")
async def execute_lift_preset(
    preset_id: str,
    current_user=Depends(get_current_admin)
):
    """执行升降高度预设"""
    watchdog.heartbeat()
    
    preset = preset_manager.get_lift_height(preset_id)
    if not preset:
        raise HTTPException(status_code=404, detail="预设不存在")
    
    # 发送升降命令
    if ros2_bridge.is_connected():
        result = await ros2_bridge.call_lift_control(
            command=2,  # 位置模式
            value=preset.height,
            hold=True
        )
        if result:
            return PresetResponse(
                success=result.get("success", False),
                message=result.get("message", ""),
                data={"target_height": preset.height}
            )
    
    # Mock 模式
    return PresetResponse(
        success=True,
        message=f"升降至 {preset.name} ({preset.height}mm) (mock mode)",
        data={"target_height": preset.height}
    )


# ==================== 头部位置专用端点 ====================

@router.get("/head/presets")
async def list_head_presets(
    category: Optional[str] = None,
    current_user=Depends(get_current_admin)
):
    """获取头部位置预设列表"""
    items = preset_manager.list(PresetType.HEAD_POSITION, category)
    return {
        "total": len(items),
        "items": [item.model_dump() for item in items]
    }


@router.post("/head/presets/capture")
async def capture_head_position(
    request: CaptureRequest,
    current_user=Depends(get_current_admin)
):
    """采集当前头部位置为预设"""
    watchdog.heartbeat()
    
    head_state = ros2_bridge.get_head_state()
    if not head_state and not ros2_bridge.mock_mode:
        raise HTTPException(status_code=400, detail="无法获取头部状态")
    
    pan = head_state.get("pan_normalized", 0.0) if head_state else 0.0
    tilt = head_state.get("tilt_normalized", 0.0) if head_state else 0.0
    
    data = {
        "name": request.name,
        "description": request.description,
        "category": request.category,
        "pan": pan,
        "tilt": tilt
    }
    
    preset = preset_manager.create(PresetType.HEAD_POSITION, data)
    return PresetResponse(
        success=True,
        message=f"头部位置 '{request.name}' 已保存",
        data=preset.model_dump()
    )


@router.post("/head/presets/{preset_id}/execute")
async def execute_head_preset(
    preset_id: str,
    current_user=Depends(get_current_admin)
):
    """执行头部位置预设"""
    watchdog.heartbeat()
    
    preset = preset_manager.get_head_position(preset_id)
    if not preset:
        raise HTTPException(status_code=404, detail="预设不存在")
    
    if ros2_bridge.is_connected():
        result = await ros2_bridge.send_head_command(
            pan=preset.pan,
            tilt=preset.tilt
        )
        if result:
            return PresetResponse(
                success=result.get("success", False),
                message=result.get("message", ""),
                data={"pan": preset.pan, "tilt": preset.tilt}
            )
    
    return PresetResponse(
        success=True,
        message=f"头部转向 {preset.name} (mock mode)",
        data={"pan": preset.pan, "tilt": preset.tilt}
    )


# ==================== 手臂姿态专用端点 ====================

class UpdateHomeRequest(BaseModel):
    """更新初始点请求"""
    side: str = Field(default="both", description="更新哪一侧: left, right, both")


@router.get("/arm/presets")
async def list_arm_presets(
    category: Optional[str] = None,
    side: Optional[str] = None,
    current_user=Depends(get_current_admin)
):
    """获取手臂姿态预设列表"""
    items = preset_manager.list(PresetType.ARM_POSE, category)
    
    # 按 side 过滤
    if side:
        items = [i for i in items if i.side == side or i.side == "both"]
    
    return {
        "total": len(items),
        "items": [item.model_dump() for item in items]
    }


@router.post("/arm/presets/capture")
async def capture_arm_pose(
    request: CaptureRequest,
    current_user=Depends(get_current_admin)
):
    """采集当前手臂姿态为预设"""
    watchdog.heartbeat()
    
    arm_state = ros2_bridge.get_arm_state()
    joint_positions = ros2_bridge.joint_positions
    
    side = request.side or "both"
    
    data = {
        "name": request.name,
        "description": request.description,
        "category": request.category,
        "side": side,
        "pose_type": "joint",
    }
    
    if side in ["left", "both"]:
        left_joints = joint_positions[:7] if len(joint_positions) >= 7 else [0.0] * 7
        data["left_joints"] = left_joints
    
    if side in ["right", "both"]:
        right_joints = joint_positions[7:14] if len(joint_positions) >= 14 else [0.0] * 7
        data["right_joints"] = right_joints
    
    preset = preset_manager.create(PresetType.ARM_POSE, data)
    return PresetResponse(
        success=True,
        message=f"手臂姿态 '{request.name}' 已保存",
        data=preset.model_dump()
    )


@router.post("/arm/presets/update_home")
async def update_home_pose(
    request: UpdateHomeRequest,
    current_user=Depends(get_current_admin)
):
    """
    更新初始点为当前位置
    
    支持:
    - side="both": 同时更新左右臂
    - side="left": 只更新左臂
    - side="right": 只更新右臂
    """
    watchdog.heartbeat()
    
    joint_positions = ros2_bridge.joint_positions
    
    if len(joint_positions) < 14:
        raise HTTPException(status_code=400, detail="无法获取完整的关节数据（需要14个关节）")
    
    left_joints = list(joint_positions[:7])
    right_joints = list(joint_positions[7:14])
    
    update_data = {}
    side = request.side
    
    if side in ["left", "both"]:
        update_data["left_joints"] = left_joints
    if side in ["right", "both"]:
        update_data["right_joints"] = right_joints
    
    try:
        # 使用特殊的更新内置预设方法
        preset = preset_manager.update_builtin(
            PresetType.ARM_POSE, 
            "pose_home",  # 初始点的固定ID
            update_data,
            side=side if side != "both" else None
        )
        
        if not preset:
            raise HTTPException(status_code=404, detail="初始点预设不存在")
        
        side_text = {"left": "左臂", "right": "右臂", "both": "双臂"}[side]
        return PresetResponse(
            success=True,
            message=f"初始点已更新（{side_text}）",
            data=preset.model_dump()
        )
    except ValueError as e:
        raise HTTPException(status_code=400, detail=str(e))


@router.post("/arm/presets/{preset_id}/execute")
async def execute_arm_preset(
    preset_id: str,
    override_side: Optional[str] = None,
    current_user=Depends(get_current_admin)
):
    """执行手臂姿态预设"""
    watchdog.heartbeat()
    
    preset = preset_manager.get_arm_pose(preset_id)
    if not preset:
        raise HTTPException(status_code=404, detail="预设不存在")
    
    side = override_side or preset.side
    results = []
    
    if side in ["left", "both"] and preset.left_joints:
        if ros2_bridge.is_connected():
            result = await ros2_bridge.call_move_j(
                robot_id=0,
                joint_positions=preset.left_joints,
                velocity=preset.velocity,
                acceleration=preset.acceleration,
                is_block=False
            )
            results.append({"left": result})
        else:
            results.append({"left": {"success": True, "message": "mock"}})
    
    if side in ["right", "both"] and preset.right_joints:
        if ros2_bridge.is_connected():
            result = await ros2_bridge.call_move_j(
                robot_id=1,
                joint_positions=preset.right_joints,
                velocity=preset.velocity,
                acceleration=preset.acceleration,
                is_block=False
            )
            results.append({"right": result})
        else:
            results.append({"right": {"success": True, "message": "mock"}})
    
    return PresetResponse(
        success=True,
        message=f"执行姿态 {preset.name}",
        data={"results": results}
    )


# ==================== 夹爪位置专用端点 ====================

@router.get("/gripper/presets")
async def list_gripper_presets(
    category: Optional[str] = None,
    side: Optional[str] = None,
    current_user=Depends(get_current_admin)
):
    """获取夹爪位置预设列表"""
    items = preset_manager.list(PresetType.GRIPPER_POSITION, category)
    
    if side:
        items = [i for i in items if i.side == side or i.side == "both"]
    
    return {
        "total": len(items),
        "items": [item.model_dump() for item in items]
    }


@router.post("/gripper/presets/{preset_id}/execute")
async def execute_gripper_preset(
    preset_id: str,
    override_side: Optional[str] = None,
    current_user=Depends(get_current_admin)
):
    """执行夹爪位置预设"""
    watchdog.heartbeat()
    
    preset = preset_manager.get_gripper_position(preset_id)
    if not preset:
        raise HTTPException(status_code=404, detail="预设不存在")
    
    side = override_side or preset.side
    
    # TODO: 调用夹爪控制服务
    return PresetResponse(
        success=True,
        message=f"执行夹爪预设 {preset.name} (side={side})",
        data=preset.model_dump()
    )


# ==================== 底盘点位专用端点 ====================

@router.get("/location/presets")
async def list_location_presets(
    category: Optional[str] = None,
    current_user=Depends(get_current_admin)
):
    """获取底盘点位预设列表"""
    items = preset_manager.list(PresetType.LOCATION, category)
    return {
        "total": len(items),
        "items": [item.model_dump() for item in items]
    }


@router.post("/location/presets/capture")
async def capture_location(
    request: CaptureRequest,
    current_user=Depends(get_current_admin)
):
    """采集当前位置为预设点位"""
    watchdog.heartbeat()
    
    # TODO: 从导航系统获取当前位置
    # 暂时使用模拟数据
    data = {
        "name": request.name,
        "description": request.description,
        "category": request.category,
        "x": 0.0,
        "y": 0.0,
        "theta": 0.0
    }
    
    preset = preset_manager.create(PresetType.LOCATION, data)
    return PresetResponse(
        success=True,
        message=f"点位 '{request.name}' 已保存",
        data=preset.model_dump()
    )


@router.post("/location/presets/{preset_id}/navigate")
async def navigate_to_location(
    preset_id: str,
    current_user=Depends(get_current_admin)
):
    """导航到预设点位"""
    watchdog.heartbeat()
    
    preset = preset_manager.get_location(preset_id)
    if not preset:
        raise HTTPException(status_code=404, detail="点位不存在")
    
    # TODO: 调用导航服务
    return PresetResponse(
        success=True,
        message=f"导航至 {preset.name} (x={preset.x}, y={preset.y})",
        data=preset.model_dump()
    )


# ==================== 任务模板端点 ====================

@router.get("/task/templates")
async def list_task_templates(
    category: Optional[str] = None,
    tags: Optional[List[str]] = Query(None),
    current_user=Depends(get_current_admin)
):
    """获取任务模板列表"""
    items = preset_manager.list(PresetType.TASK_TEMPLATE, category)
    
    # 按标签过滤
    if tags:
        items = [i for i in items if any(t in i.tags for t in tags)]
    
    return {
        "total": len(items),
        "items": [item.model_dump() for item in items]
    }


@router.post("/task/templates")
async def create_task_template(
    request: PresetCreateRequest,
    current_user=Depends(get_current_admin)
):
    """创建任务模板"""
    data = {
        "name": request.name,
        "description": request.description,
        "category": request.category,
        "task_tree": request.data.get("task_tree", {}),
        "input_params": request.data.get("input_params", []),
        "tags": request.data.get("tags", [])
    }
    
    preset = preset_manager.create(PresetType.TASK_TEMPLATE, data)
    return PresetResponse(
        success=True,
        message=f"任务模板 '{request.name}' 创建成功",
        data=preset.model_dump()
    )


# ==================== 导入导出 ====================

@router.get("/presets/export")
async def export_presets(current_user=Depends(get_current_admin)):
    """导出所有预设"""
    return preset_manager.export_all()


@router.post("/presets/import")
async def import_presets(
    data: Dict[str, Any],
    overwrite: bool = False,
    current_user=Depends(get_current_admin)
):
    """导入预设"""
    try:
        preset_manager.import_presets(data, overwrite)
        return PresetResponse(success=True, message="预设导入成功")
    except Exception as e:
        raise HTTPException(status_code=400, detail=str(e))
