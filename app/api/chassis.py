"""底盘控制 API - Standard Robot"""
import subprocess
import sys
from pathlib import Path
from fastapi import APIRouter, HTTPException
from pydantic import BaseModel
from typing import Optional, List
from app.ros2_bridge.bridge import ros2_bridge

router = APIRouter()

# ==================== 持久化配置 ====================

def _get_chassis_config_file() -> Path:
    """获取地盘配置文件路径"""
    workspace_root = Path(__file__).parent.parent.parent.parent.parent
    config_dir = workspace_root / "persistent" / "web"
    config_dir.mkdir(parents=True, exist_ok=True)
    return config_dir / "chassis_config.json"

def _load_chassis_config() -> dict:
    """加载地盘配置"""
    config_file = _get_chassis_config_file()
    if config_file.exists():
        try:
            with open(config_file, 'r', encoding='utf-8') as f:
                return json.load(f)
        except:
            pass
    # 默认配置
    return {
        "speed_level": 50,
        "volume": 50,
        "obstacle_strategy": 1
    }

def _save_chassis_config(config: dict):
    """保存地盘配置"""
    config_file = _get_chassis_config_file()
    with open(config_file, 'w', encoding='utf-8') as f:
        json.dump(config, f, indent=2, ensure_ascii=False)

# ==================== 数据模型 ====================

class VelocityCommand(BaseModel):
    """速度命令"""
    linear_x: float = 0.0
    linear_y: float = 0.0
    angular_z: float = 0.0


class NavigationTarget(BaseModel):
    """导航目标"""
    x: float
    y: float
    yaw: float
    is_localization: bool = False  # True=仅定位, False=导航移动


class SiteTarget(BaseModel):
    """站点目标"""
    site_id: int
    is_localization: bool = False  # True=仅定位, False=导航移动


class SiteNavigationRequest(BaseModel):
    """站点导航请求（不带任务ID）"""
    site_id: int


class SiteNavigationWithTaskRequest(BaseModel):
    """站点导航请求（带任务ID）"""
    site_id: int
    task_id: int


class ForceLocalizeRequest(BaseModel):
    """强制定位请求"""
    x: float
    y: float
    yaw: float


class SetSpeedLevelRequest(BaseModel):
    """设置速度级别"""
    level: int  # 1-100


class SetObstacleStrategyRequest(BaseModel):
    """设置避障策略"""
    strategy: int


class SetCurrentSiteRequest(BaseModel):
    """设置当前站点"""
    site_id: int


class SetVolumeRequest(BaseModel):
    """设置音量"""
    volume: int  # 1-100


class SetMapRequest(BaseModel):
    """设置当前地图"""
    map_name: str


class ManualControlCommand(BaseModel):
    """手动控制命令"""
    forward: bool = False
    backward: bool = False
    left: bool = False
    right: bool = False
    rotate_left: bool = False
    rotate_right: bool = False


# ==================== 状态获取 ====================

def _get_current_map_from_file() -> str:
    """从 maps/current_map.txt 读取当前地图名"""
    workspace_root = Path(__file__).parent.parent.parent.parent.parent
    current_map_file = workspace_root / "maps" / "current_map.txt"
    if current_map_file.exists():
        return current_map_file.read_text(encoding='utf-8').strip()
    return "未同步"


@router.get("/chassis/status")
async def get_chassis_status():
    """获取底盘完整状态"""
    status = ros2_bridge.get_chassis_status()
    
    # 从文件读取真正的地图名（HTTP/WebSocket 同步的）
    real_map_name = _get_current_map_from_file()
    
    # 加载持久化配置
    config = _load_chassis_config()
    
    if status is None:
        # 返回 Mock 数据
        return {
            "connected": False,
            "system_status": 0x02,
            "system_status_text": "系统空闲",
            "location_status": 0x03,
            "location_status_text": "定位成功",
            "operation_status": 0x01,
            "operation_status_text": "自动控制模式",
            "scheduling_mode": 0x02,
            "scheduling_mode_text": "自动模式",
            "motion_status": 0x01,
            "motion_status_text": "静止",
            "current_station_id": 0,
            "last_error_code": 0,
            "current_system_volume": 50,
            "ip_address": "192.168.1.100",
            "current_map_name": real_map_name,
            
            # 位姿信息
            "pose": {
                "x": 0.0,
                "y": 0.0,
                "yaw": 0.0,
                "confidence": 0.95
            },
            
            # 速度信息
            "velocity": {
                "linear_x": 0.0,
                "linear_y": 0.0,
                "angular_z": 0.0
            },
            
            # 电池信息
            "battery": {
                "percentage": 78,
                "voltage": 25.6,
                "current": -0.5,
                "temperature": 35,
                "estimated_time": 120,
                "status": 0x03,
                "status_text": "未充电",
                "cycle_count": 156,
                "nominal_capacity": 20000
            },
            
            # 统计信息
            "statistics": {
                "total_distance": 12500,
                "total_boot_time": 360000,
                "total_boot_count": 89
            },
            
            # 状态标志
            "flags": {
                "is_emergency_stopped": False,
                "is_emergency_recoverable": True,
                "is_brake_released": True,
                "is_charging": False,
                "is_low_power_mode": False,
                "obstacle_slowdown": False,
                "obstacle_paused": False,
                "can_run_motion_task": True,
                "is_auto_mode": True,
                "is_loaded": False,
                "has_wifi": True
            },
            
            # 避障传感器触发
            "obstacle_sensors": {
                "main_radar": False,
                "aux_radar": False,
                "depth_camera1": False,
                "depth_camera2": False,
                "depth_camera3": False,
                "depth_camera4": False,
                "obstacle_radar1": False,
                "obstacle_radar2": False,
                "obstacle_radar3": False,
                "obstacle_radar4": False
            },
            
            # 任务信息
            "mission": {
                "id": 0,
                "status": 0x00,
                "status_text": "无效状态",
                "result": 0x00,
                "result_text": "无效状态",
                "error_code": 0
            },
            
            # 移动任务信息
            "move_task": {
                "no": 0,
                "status": 0x00,
                "status_text": "无效状态",
                "result": 0x00,
                "result_text": "无效状态",
                "start_station": 0,
                "dest_station": 0,
                "path_no": 0
            }
        }
    
    # 覆盖 Modbus 返回的地图名（可能是 ID 编码），使用真实地图名
    if isinstance(status, dict):
        status["current_map_name"] = real_map_name
        # 添加持久化配置的值
        status['speed_level'] = config.get('speed_level', 50)
        status['current_system_volume'] = config.get('volume', 50)
        status['obstacle_strategy'] = config.get('obstacle_strategy', 1)
    
    return status


@router.get("/chassis/navigation_status")
async def get_navigation_status():
    """获取导航状态"""
    status = ros2_bridge.get_navigation_status()
    
    if status is None:
        return {
            "communication_pose": {"x": 0.0, "y": 0.0, "yaw": 0.0},
            "autonomous_nav_pose": {"x": 0.0, "y": 0.0, "yaw": 0.0},
            "speed_level": 50,
            "obstacle_strategy": 0,
            "current_site": 0,
            "speaker_volume": 50
        }
    
    return status


# ==================== 控制服务 ====================

@router.post("/chassis/velocity")
async def send_velocity(cmd: VelocityCommand):
    """发送速度命令"""
    result = await ros2_bridge.send_chassis_velocity(
        cmd.linear_x, cmd.linear_y, cmd.angular_z
    )
    
    if result is None:
        return {"success": True, "message": "速度命令已发送 (Mock)"}
    
    return result


@router.post("/chassis/stop")
async def stop_chassis():
    """停止底盘"""
    result = await ros2_bridge.call_chassis_service("stop_move")
    
    if result is None:
        return {"success": True, "message": "停止命令已发送 (Mock)"}
    
    return result


class ManualModeRequest(BaseModel):
    """手动模式请求"""
    enable: bool = True
    mode: str = "velocity"  # 'coil' or 'velocity'


@router.post("/chassis/manual_mode")
async def set_manual_mode(request: ManualModeRequest):
    """设置手动控制模式（开启/关闭）"""
    if request.enable:
        # 开启手动控制
        result = await ros2_bridge.call_chassis_service("start_manual")
        if result is None:
            return {
                "success": True,
                "message": f"手动{request.mode}模式已启动 (Mock)"
            }
        return result
    else:
        # 关闭手动控制
        result = await ros2_bridge.call_chassis_service("stop_manual")
        if result is None:
            return {"success": True, "message": "手动控制已关闭 (Mock)"}
        return result


@router.post("/chassis/manual/start")
async def start_manual_control():
    """启动手动控制"""
    result = await ros2_bridge.call_chassis_service("start_manual")
    
    if result is None:
        return {"success": True, "message": "手动控制已启动 (Mock)"}
    
    return result


@router.post("/chassis/manual/stop")
async def stop_manual_control():
    """停止手动控制"""
    result = await ros2_bridge.call_chassis_service("stop_manual")
    
    if result is None:
        return {"success": True, "message": "手动控制已停止 (Mock)"}
    
    return result


@router.post("/chassis/manual/command")
async def send_manual_command(cmd: ManualControlCommand):
    """发送手动控制命令"""
    result = await ros2_bridge.send_manual_motion_command(
        forward=cmd.forward,
        backward=cmd.backward,
        left=cmd.left,
        right=cmd.right,
        rotate_left=cmd.rotate_left,
        rotate_right=cmd.rotate_right
    )
    
    if result is None:
        return {"success": True, "message": "手动命令已发送 (Mock)"}
    
    return result


# ==================== 移动控制 ====================

@router.post("/chassis/pause_move")
async def pause_move():
    """暂停移动"""
    result = await ros2_bridge.call_chassis_service("pause_move")
    
    if result is None:
        return {"success": True, "message": "暂停移动 (Mock)"}
    
    return result


@router.post("/chassis/resume_move")
async def resume_move():
    """恢复移动"""
    result = await ros2_bridge.call_chassis_service("resume_move")
    
    if result is None:
        return {"success": True, "message": "恢复移动 (Mock)"}
    
    return result


@router.post("/chassis/stop_move")
async def stop_move():
    """停止移动"""
    result = await ros2_bridge.call_chassis_service("stop_move")
    
    if result is None:
        return {"success": True, "message": "停止移动 (Mock)"}
    
    return result


@router.post("/chassis/stop_localization")
async def stop_localization():
    """停止定位"""
    result = await ros2_bridge.call_chassis_service("stop_localization")
    
    if result is None:
        return {"success": True, "message": "停止定位 (Mock)"}
    
    return result


# ==================== 急停控制 ====================

@router.post("/chassis/emergency_stop")
async def emergency_stop():
    """紧急停止"""
    result = await ros2_bridge.call_chassis_service("emergency_stop")
    
    if result is None:
        return {"success": True, "message": "紧急停止已触发 (Mock)"}
    
    return result


@router.post("/chassis/release_emergency_stop")
async def release_emergency_stop():
    """解除急停"""
    result = await ros2_bridge.call_chassis_service("release_emergency_stop")
    
    if result is None:
        return {"success": True, "message": "急停已解除 (Mock)"}
    
    return result


# ==================== 充电控制 ====================

@router.post("/chassis/start_charging")
async def start_charging():
    """开始充电"""
    result = await ros2_bridge.call_chassis_service("start_charging")
    
    if result is None:
        return {"success": True, "message": "开始充电 (Mock)"}
    
    return result


@router.post("/chassis/stop_charging")
async def stop_charging():
    """停止充电"""
    result = await ros2_bridge.call_chassis_service("stop_charging")
    
    if result is None:
        return {"success": True, "message": "停止充电 (Mock)"}
    
    return result


# ==================== 功耗控制 ====================

@router.post("/chassis/enter_low_power")
async def enter_low_power():
    """进入低功耗模式"""
    result = await ros2_bridge.call_chassis_service("enter_low_power")
    
    if result is None:
        return {"success": True, "message": "进入低功耗模式 (Mock)"}
    
    return result


@router.post("/chassis/exit_low_power")
async def exit_low_power():
    """退出低功耗模式"""
    result = await ros2_bridge.call_chassis_service("exit_low_power")
    
    if result is None:
        return {"success": True, "message": "退出低功耗模式 (Mock)"}
    
    return result


# ==================== 系统控制 ====================

@router.post("/chassis/system_reset")
async def system_reset():
    """系统复位"""
    result = await ros2_bridge.call_chassis_service("system_reset")
    
    if result is None:
        return {"success": True, "message": "系统复位 (Mock)"}
    
    return result


# ==================== 任务控制 ====================

@router.post("/chassis/mission/pause")
async def pause_mission():
    """暂停任务"""
    result = await ros2_bridge.call_chassis_service("pause_mission")
    
    if result is None:
        return {"success": True, "message": "任务已暂停 (Mock)"}
    
    return result


@router.post("/chassis/mission/resume")
async def resume_mission():
    """恢复任务"""
    result = await ros2_bridge.call_chassis_service("resume_mission")
    
    if result is None:
        return {"success": True, "message": "任务已恢复 (Mock)"}
    
    return result


@router.post("/chassis/mission/cancel")
async def cancel_mission():
    """取消任务"""
    result = await ros2_bridge.call_chassis_service("cancel_mission")
    
    if result is None:
        return {"success": True, "message": "任务已取消 (Mock)"}
    
    return result


# ==================== 导航控制 ====================

@router.post("/chassis/navigate/coordinate")
async def navigate_to_coordinate(target: NavigationTarget):
    """导航到坐标点"""
    result = await ros2_bridge.navigate_to_coordinate(
        x=target.x,
        y=target.y,
        yaw=target.yaw,
        is_localization=target.is_localization
    )
    
    if result is None:
        action = "定位" if target.is_localization else "导航"
        return {"success": True, "message": f"{action}到 ({target.x}, {target.y}) (Mock)"}
    
    return result


@router.post("/chassis/navigate/site")
async def navigate_to_site(target: SiteTarget):
    """导航到站点（兼容旧接口）"""
    result = await ros2_bridge.navigate_to_site(
        site_id=target.site_id,
        is_localization=target.is_localization
    )
    
    if result is None:
        action = "定位" if target.is_localization else "导航"
        return {"success": True, "message": f"{action}到站点 {target.site_id} (Mock)"}
    
    return result


@router.post("/chassis/navigate/site_simple")
async def navigate_to_site_simple(req: SiteNavigationRequest):
    """导航到站点（不带任务ID）"""
    result = await ros2_bridge.navigate_to_site_simple(
        site_id=req.site_id
    )
    
    if result is None:
        return {"success": True, "message": f"导航到站点 {req.site_id} (Mock)"}
    
    return result


@router.post("/chassis/navigate/site_with_task")
async def navigate_to_site_with_task(req: SiteNavigationWithTaskRequest):
    """导航到站点（带任务ID）"""
    result = await ros2_bridge.navigate_to_site_with_task(
        site_id=req.site_id,
        task_id=req.task_id
    )
    
    if result is None:
        return {"success": True, "message": f"导航到站点 {req.site_id} 任务 {req.task_id} (Mock)"}
    
    return result


@router.post("/chassis/force_localize")
async def force_localize(req: ForceLocalizeRequest):
    """强制定位"""
    result = await ros2_bridge.force_localize(
        x=req.x, y=req.y, yaw=req.yaw
    )
    
    if result is None:
        return {"success": True, "message": f"强制定位到 ({req.x}, {req.y}) (Mock)"}
    
    return result


# ==================== 参数设置 ====================

@router.post("/chassis/set_speed_level")
async def set_speed_level(req: SetSpeedLevelRequest):
    """设置速度级别"""
    result = await ros2_bridge.set_chassis_speed_level(req.level)
    
    if result is None:
        result = {"success": True, "message": f"速度级别设置为 {req.level} (Mock)"}
    
    # 成功后保存到持久化配置
    if result.get('success', False):
        config = _load_chassis_config()
        config['speed_level'] = req.level
        _save_chassis_config(config)
    
    return result


@router.post("/chassis/set_obstacle_strategy")
async def set_obstacle_strategy(req: SetObstacleStrategyRequest):
    """设置避障策略"""
    result = await ros2_bridge.set_chassis_obstacle_strategy(req.strategy)
    
    if result is None:
        result = {"success": True, "message": f"避障策略设置为 {req.strategy} (Mock)"}
    
    # 成功后保存到持久化配置
    if result.get('success', False):
        config = _load_chassis_config()
        config['obstacle_strategy'] = req.strategy
        _save_chassis_config(config)
    
    return result


@router.post("/chassis/set_current_site")
async def set_current_site(req: SetCurrentSiteRequest):
    """设置当前站点"""
    result = await ros2_bridge.set_chassis_current_site(req.site_id)
    
    if result is None:
        return {"success": True, "message": f"当前站点设置为 {req.site_id} (Mock)"}
    
    return result


@router.post("/chassis/set_volume")
async def set_volume(req: SetVolumeRequest):
    """设置音量"""
    result = await ros2_bridge.set_chassis_volume(req.volume)
    
    if result is None:
        result = {"success": True, "message": f"音量设置为 {req.volume} (Mock)"}
    
    # 成功后保存到持久化配置
    if result.get('success', False):
        config = _load_chassis_config()
        config['volume'] = req.volume
        _save_chassis_config(config)
    
    return result


@router.post("/chassis/set_map")
async def set_map(req: SetMapRequest):
    """设置当前地图"""
    result = await ros2_bridge.set_chassis_map(req.map_name)
    
    if result is None:
        return {"success": True, "message": f"地图设置为 {req.map_name} (Mock)"}
    
    return result


# ==================== 预设站点管理 ====================

@router.get("/chassis/stations")
async def get_stations():
    """获取预设站点列表"""
    # TODO: 从配置文件或 ROS2 获取
    return {
        "stations": [
            {"id": 1, "name": "充电站", "x": 0.0, "y": 0.0, "yaw": 0.0},
            {"id": 2, "name": "工作台 A", "x": 1.5, "y": 2.0, "yaw": 1.57},
            {"id": 3, "name": "工作台 B", "x": 3.0, "y": 1.0, "yaw": 3.14},
            {"id": 4, "name": "物料区", "x": -1.0, "y": 2.5, "yaw": 0.78}
        ]
    }


# ==================== 地图同步 ====================

@router.post("/chassis/sync_maps")
async def sync_maps():
    """
    从底盘同步地图数据
    执行 qyh_standard_api/get_map.py 脚本
    """
    # 获取 get_map.py 路径
    workspace_root = Path(__file__).parent.parent.parent.parent.parent
    get_map_script = workspace_root / "qyh_standard_api" / "get_map.py"
    
    if not get_map_script.exists():
        raise HTTPException(
            status_code=500, 
            detail=f"get_map.py 脚本不存在: {get_map_script}"
        )
    
    try:
        # 使用后端的 venv Python（已安装所需依赖 requests, websocket-client, pyyaml）
        backend_venv = workspace_root / "qyh_jushen_web" / "backend" / "venv" / "bin" / "python3"
        if not backend_venv.exists():
            backend_venv = Path("/usr/bin/python3")
        
        script_dir = get_map_script.parent
        
        # 设置环境变量，确保能找到依赖
        env = {
            **dict(__import__('os').environ),
            "PYTHONPATH": str(script_dir)
        }
        
        # 执行脚本
        result = subprocess.run(
            [str(backend_venv), str(get_map_script)],
            capture_output=True,
            text=True,
            timeout=60,
            cwd=str(script_dir),
            env=env
        )
        
        if result.returncode != 0:
            raise HTTPException(
                status_code=500,
                detail=f"同步失败: {result.stderr or result.stdout}"
            )
        
        # 读取当前地图名
        maps_dir = workspace_root / "maps"
        current_map_file = maps_dir / "current_map.txt"
        current_map = ""
        if current_map_file.exists():
            current_map = current_map_file.read_text(encoding='utf-8').strip()
        
        # 获取地图列表
        map_list = []
        if maps_dir.exists():
            for d in maps_dir.iterdir():
                if d.is_dir():
                    map_list.append(d.name)
        
        return {
            "success": True,
            "message": "地图同步完成",
            "current_map": current_map,
            "maps": map_list,
            "output": result.stdout
        }
        
    except subprocess.TimeoutExpired:
        raise HTTPException(status_code=500, detail="同步超时")
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"同步失败: {str(e)}")


@router.get("/chassis/map_data")
async def get_map_data():
    """
    获取当前地图的完整数据，用于3D场景渲染
    返回: meta, nodes, edges, stations, image_url
    """
    import json
    
    workspace_root = Path(__file__).parent.parent.parent.parent.parent
    maps_dir = workspace_root / "maps"
    
    # 读取当前地图名
    current_map_file = maps_dir / "current_map.txt"
    if not current_map_file.exists():
        raise HTTPException(status_code=404, detail="未找到当前地图信息，请先同步地图")
    
    current_map = current_map_file.read_text(encoding='utf-8').strip()
    if not current_map:
        raise HTTPException(status_code=404, detail="当前地图名称为空")
    
    # 读取地图JSON数据
    map_json_file = maps_dir / current_map / f"{current_map}.json"
    if not map_json_file.exists():
        raise HTTPException(
            status_code=404, 
            detail=f"地图数据文件不存在: {current_map}"
        )
    
    with open(map_json_file, 'r', encoding='utf-8') as f:
        map_data = json.load(f)
    
    meta = map_data.get('meta', {})
    data = map_data.get('data', {})
    
    # 检查地图图片
    map_image_file = maps_dir / current_map / f"{current_map}.png"
    has_image = map_image_file.exists()
    
    return {
        "success": True,
        "map_name": current_map,
        "meta": meta,
        "nodes": data.get('node', []),
        "edges": data.get('edge', []),
        "stations": data.get('station', []),
        "has_image": has_image,
        "image_url": f"/api/v1/chassis/map_image/{current_map}" if has_image else None
    }


@router.get("/chassis/map_image/{map_name}")
async def get_map_image(map_name: str):
    """获取地图图片"""
    from fastapi.responses import FileResponse
    
    workspace_root = Path(__file__).parent.parent.parent.parent.parent
    maps_dir = workspace_root / "maps"
    
    # 安全检查：防止路径遍历
    if '..' in map_name or '/' in map_name or '\\' in map_name:
        raise HTTPException(status_code=400, detail="无效的地图名称")
    
    map_image_file = maps_dir / map_name / f"{map_name}.png"
    if not map_image_file.exists():
        # 尝试 jpg
        map_image_file = maps_dir / map_name / f"{map_name}.jpg"
        if not map_image_file.exists():
            raise HTTPException(status_code=404, detail="地图图片不存在")
    
    return FileResponse(
        path=str(map_image_file),
        media_type="image/png" if map_image_file.suffix == '.png' else "image/jpeg"
    )


@router.get("/chassis/maps")
async def get_maps_list():
    """获取所有地图列表"""
    workspace_root = Path(__file__).parent.parent.parent.parent.parent
    maps_dir = workspace_root / "maps"
    
    if not maps_dir.exists():
        return {"maps": [], "current_map": None}
    
    # 读取当前地图
    current_map_file = maps_dir / "current_map.txt"
    current_map = ""
    if current_map_file.exists():
        current_map = current_map_file.read_text(encoding='utf-8').strip()
    
    # 获取地图列表
    map_list = []
    for d in maps_dir.iterdir():
        if d.is_dir():
            map_list.append(d.name)
    
    return {
        "maps": map_list,
        "current_map": current_map
    }
