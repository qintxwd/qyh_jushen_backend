"""
QYH Jushen Control Plane - 预设管理器

负责预设的持久化存储和管理
存储位置: ~/qyh-robot-system/persistent/preset/
"""
import os
import json
import uuid
import shutil
from pathlib import Path
from typing import Dict, List, Optional, Any
from datetime import datetime
from threading import Lock
import logging

from app.schemas.preset import PresetType, PresetInfo
from app.config import settings

logger = logging.getLogger(__name__)


class PresetManager:
    """
    预设管理器
    
    功能:
    - 加载/保存预设到 JSON 文件
    - 增删改查操作
    - 内置预设管理
    - 线程安全
    """
    
    # 文件名映射
    FILE_MAP: Dict[PresetType, str] = {
        PresetType.ARM_POSE: "arm_points.json",
        PresetType.HEAD_POSITION: "head_points.json",
        PresetType.LIFT_HEIGHT: "lift_points.json",
        PresetType.WAIST_ANGLE: "waist_points.json",
        PresetType.LOCATION: "locations.json",
        PresetType.GRIPPER_POSITION: "gripper_positions.json",
        PresetType.FULL_POSE: "full_poses.json",
    }
    
    def __init__(self, storage_path: str = None):
        """
        初始化预设管理器
        
        Args:
            storage_path: 存储路径，默认为 configured PERSISTENT_DIR/preset/
        """
        if storage_path:
            self.storage_path = Path(storage_path)
        else:
            self.storage_path = Path(settings.persistent_dir_expanded) / "preset"
        
        self.storage_path.mkdir(parents=True, exist_ok=True)
        
        # 数据缓存: {preset_type: {preset_id: preset_data}}
        self._cache: Dict[PresetType, Dict[str, dict]] = {
            t: {} for t in PresetType
        }
        
        # 线程锁
        self._lock = Lock()
        
        # 初始化内置预设
        self._init_builtin_presets()
        
        # 加载用户预设
        self._load_all()
        
        logger.info(f"预设管理器初始化完成，存储路径: {self.storage_path}")
    
    def _load_all(self):
        """加载所有预设文件"""
        for preset_type, filename in self.FILE_MAP.items():
            filepath = self.storage_path / filename
            if filepath.exists():
                try:
                    with open(filepath, 'r', encoding='utf-8') as f:
                        data = json.load(f)
                    
                    items = []
                    if isinstance(data, list):
                        items = data
                    elif isinstance(data, dict):
                        items = data.get('points', data.get('items', []))
                    
                    for item in items:
                        preset_id = item.get('id')
                        if preset_id:
                            # 不覆盖内置预设
                            if preset_id not in self._cache[preset_type] or not self._cache[preset_type][preset_id].get('is_builtin'):
                                self._cache[preset_type][preset_id] = item
                    
                    logger.info(f"加载 {len(self._cache[preset_type])} 个 {preset_type.value} 预设")
                except Exception as e:
                    logger.error(f"加载预设文件失败 [{filename}]: {e}")
    
    def _save(self, preset_type: PresetType):
        """保存指定类型的预设到文件"""
        filename = self.FILE_MAP.get(preset_type)
        if not filename:
            return
        
        filepath = self.storage_path / filename
        
        # 获取所有预设（包括内置）
        items = list(self._cache[preset_type].values())
        
        data = {
            "type": preset_type.value,
            "version": "1.0",
            "updated_at": datetime.now().isoformat(),
            "points": items  # 兼容旧格式
        }
        
        # 原子写入
        temp_filepath = filepath.with_suffix('.tmp')
        try:
            with open(temp_filepath, 'w', encoding='utf-8') as f:
                json.dump(data, f, ensure_ascii=False, indent=2)
            shutil.move(str(temp_filepath), str(filepath))
        except Exception as e:
            logger.error(f"保存预设文件失败 [{filename}]: {e}")
            if temp_filepath.exists():
                temp_filepath.unlink()
            raise
    
    def _init_builtin_presets(self):
        """初始化内置预设"""
        now = datetime.now().isoformat()
        
        # 机械臂内置点位
        arm_builtins = [
            {
                "id": "zero",
                "name": "零位",
                "description": "机械臂零位（各关节角度为0）",
                "category": "builtin",
                "is_builtin": True,
                "side": "both",
                "pose_type": "joint",
                "left_joints": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                "right_joints": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                "velocity": 0.5,
                "acceleration": 0.3,
                "created_at": now,
                "updated_at": now,
            },
            {
                "id": "home",
                "name": "初始点",
                "description": "机械臂初始工作位置",
                "category": "builtin",
                "is_builtin": True,
                "side": "both",
                "pose_type": "joint",
                "left_joints": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                "right_joints": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                "velocity": 0.5,
                "acceleration": 0.3,
                "created_at": now,
                "updated_at": now,
            },
        ]
        for item in arm_builtins:
            self._cache[PresetType.ARM_POSE][item["id"]] = item
        
        # 头部内置点位
        head_builtins = [
            {
                "id": "zero",
                "name": "零位",
                "description": "头部零位",
                "category": "builtin",
                "is_builtin": True,
                "pan": 0.0,
                "tilt": 0.0,
                "created_at": now,
                "updated_at": now,
            },
            {
                "id": "forward",
                "name": "前方",
                "description": "头部看向正前方",
                "category": "builtin",
                "is_builtin": True,
                "pan": 0.0,
                "tilt": 0.0,
                "created_at": now,
                "updated_at": now,
            },
        ]
        for item in head_builtins:
            self._cache[PresetType.HEAD_POSITION][item["id"]] = item
        
        # 升降内置高度
        lift_builtins = [
            {
                "id": "lowest",
                "name": "最低",
                "description": "升降最低位置",
                "category": "builtin",
                "is_builtin": True,
                "height": 0.0,
                "created_at": now,
                "updated_at": now,
            },
            {
                "id": "middle",
                "name": "中间",
                "description": "升降中间位置",
                "category": "builtin",
                "is_builtin": True,
                "height": 0.25,
                "created_at": now,
                "updated_at": now,
            },
            {
                "id": "highest",
                "name": "最高",
                "description": "升降最高位置",
                "category": "builtin",
                "is_builtin": True,
                "height": 0.5,
                "created_at": now,
                "updated_at": now,
            },
        ]
        for item in lift_builtins:
            self._cache[PresetType.LIFT_HEIGHT][item["id"]] = item
        
        # 腰部内置角度
        waist_builtins = [
            {
                "id": "upright",
                "name": "竖直",
                "description": "腰部竖直站立",
                "category": "builtin",
                "is_builtin": True,
                "angle": 0.0,
                "created_at": now,
                "updated_at": now,
            },
            {
                "id": "lean_15",
                "name": "前倾15度",
                "description": "腰部前倾15度",
                "category": "builtin",
                "is_builtin": True,
                "angle": 15.0,
                "created_at": now,
                "updated_at": now,
            },
            {
                "id": "lean_30",
                "name": "前倾30度",
                "description": "腰部前倾30度",
                "category": "builtin",
                "is_builtin": True,
                "angle": 30.0,
                "created_at": now,
                "updated_at": now,
            },
        ]
        for item in waist_builtins:
            self._cache[PresetType.WAIST_ANGLE][item["id"]] = item
    
    def list(
        self,
        preset_type: PresetType,
        category: Optional[str] = None,
        include_builtin: bool = True,
    ) -> List[dict]:
        """
        列出预设
        
        Args:
            preset_type: 预设类型
            category: 分类过滤
            include_builtin: 是否包含内置预设
        
        Returns:
            预设列表
        """
        with self._lock:
            items = list(self._cache[preset_type].values())
            
            if not include_builtin:
                items = [i for i in items if not i.get('is_builtin', False)]
            
            if category:
                items = [i for i in items if i.get('category') == category]
            
            # 按名称排序，内置在前
            items.sort(key=lambda x: (not x.get('is_builtin', False), x.get('name', '')))
            
            return items
    
    def get(self, preset_type: PresetType, preset_id: str) -> Optional[dict]:
        """
        获取单个预设
        
        Args:
            preset_type: 预设类型
            preset_id: 预设 ID
        
        Returns:
            预设数据，不存在返回 None
        """
        with self._lock:
            return self._cache[preset_type].get(preset_id)
    
    def create(
        self,
        preset_type: PresetType,
        name: str,
        data: Dict[str, Any],
        description: str = "",
        category: str = "custom",
    ) -> dict:
        """
        创建新预设
        
        Args:
            preset_type: 预设类型
            name: 预设名称
            data: 预设数据
            description: 描述
            category: 分类
        
        Returns:
            创建的预设
        """
        with self._lock:
            # 检查名称是否重复
            for existing in self._cache[preset_type].values():
                if existing.get('name') == name:
                    raise ValueError(f"预设名称 '{name}' 已存在")
            
            # 生成 ID
            preset_id = str(uuid.uuid4())[:8]
            now = datetime.now().isoformat()
            
            preset = {
                "id": preset_id,
                "name": name,
                "description": description,
                "category": category,
                "is_builtin": False,
                "created_at": now,
                "updated_at": now,
                **data,
            }
            
            self._cache[preset_type][preset_id] = preset
            self._save(preset_type)
            
            return preset
    
    def update(
        self,
        preset_type: PresetType,
        preset_id: str,
        name: Optional[str] = None,
        description: Optional[str] = None,
        category: Optional[str] = None,
        data: Optional[Dict[str, Any]] = None,
    ) -> dict:
        """
        更新预设
        
        Args:
            preset_type: 预设类型
            preset_id: 预设 ID
            name: 新名称
            description: 新描述
            category: 新分类
            data: 新数据
        
        Returns:
            更新后的预设
        """
        with self._lock:
            preset = self._cache[preset_type].get(preset_id)
            if not preset:
                raise ValueError(f"预设 '{preset_id}' 不存在")
            
            # 检查名称是否与其他预设重复
            if name and name != preset.get('name'):
                for pid, existing in self._cache[preset_type].items():
                    if pid != preset_id and existing.get('name') == name:
                        raise ValueError(f"预设名称 '{name}' 已存在")
            
            # 更新字段
            if name is not None:
                preset['name'] = name
            if description is not None:
                preset['description'] = description
            if category is not None:
                preset['category'] = category
            if data is not None:
                # 合并数据，保留 id, name 等元数据
                for key, value in data.items():
                    if key not in ('id', 'name', 'description', 'category', 'is_builtin', 'created_at'):
                        preset[key] = value
            
            preset['updated_at'] = datetime.now().isoformat()
            
            self._save(preset_type)
            
            return preset
    
    def delete(self, preset_type: PresetType, preset_id: str) -> bool:
        """
        删除预设
        
        Args:
            preset_type: 预设类型
            preset_id: 预设 ID
        
        Returns:
            是否成功删除
        """
        with self._lock:
            preset = self._cache[preset_type].get(preset_id)
            if not preset:
                return False
            
            if preset.get('is_builtin', False):
                raise ValueError("内置预设不可删除")
            
            del self._cache[preset_type][preset_id]
            self._save(preset_type)
            
            return True
    
    def get_by_name(self, preset_type: PresetType, name: str) -> Optional[dict]:
        """
        通过名称获取预设
        
        Args:
            preset_type: 预设类型
            name: 预设名称
        
        Returns:
            预设数据，不存在返回 None
        """
        with self._lock:
            for preset in self._cache[preset_type].values():
                if preset.get('name') == name:
                    return preset
            return None
    
    def get_locations_from_map(self) -> List[dict]:
        """
        从当前地图读取导航站点
        
        Returns:
            站点列表
        """
        try:
            # 地图文件路径
            workspace_root = Path(__file__).parent.parent.parent.parent.parent
            maps_dir = workspace_root / "maps"
            
            # 读取当前地图名
            current_map_file = maps_dir / "current_map.txt"
            if not current_map_file.exists():
                return []
            
            current_map = current_map_file.read_text(encoding='utf-8').strip()
            if not current_map:
                return []
            
            map_json_file = maps_dir / current_map / f"{current_map}.json"
            if not map_json_file.exists():
                return []
            
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
                    "is_builtin": True,
                    "x": station.get('pos.x', 0.0),
                    "y": station.get('pos.y', 0.0),
                    "theta": station.get('pos.yaw', 0.0) / 1000.0,  # 转换为弧度
                    "frame_id": "map",
                    "station_id": station['id'],
                    "created_at": datetime.now().isoformat(),
                    "updated_at": datetime.now().isoformat(),
                })
            
            return items
        except Exception as e:
            logger.error(f"从地图读取站点失败: {e}")
            return []


# 全局单例
preset_manager = PresetManager()
