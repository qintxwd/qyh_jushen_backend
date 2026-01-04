"""
预设管理器 (Preset Manager)

负责预设的持久化存储和管理
"""

import os
import json
import uuid
import shutil
from pathlib import Path
from typing import Dict, List, Optional, Any, Type
from datetime import datetime
from threading import Lock

from .models import (
    PresetType, PresetBase,
    Location, ArmPose, LiftHeight, HeadPosition, GripperPosition, TaskTemplate
)


class PresetManager:
    """
    预设管理器
    
    功能:
    - 加载/保存预设到 JSON 文件
    - 增删改查操作
    - 内置预设管理
    - 线程安全
    """
    
    # 预设类型到模型类的映射
    MODEL_MAP: Dict[PresetType, Type[PresetBase]] = {
        PresetType.LOCATION: Location,
        PresetType.ARM_POSE: ArmPose,
        PresetType.LIFT_HEIGHT: LiftHeight,
        PresetType.HEAD_POSITION: HeadPosition,
        PresetType.GRIPPER_POSITION: GripperPosition,
        PresetType.TASK_TEMPLATE: TaskTemplate,
    }
    
    # 文件名映射
    FILE_MAP: Dict[PresetType, str] = {
        PresetType.LOCATION: "locations.json",
        PresetType.ARM_POSE: "arm_poses.json",
        PresetType.LIFT_HEIGHT: "lift_heights.json",
        PresetType.HEAD_POSITION: "head_positions.json",
        PresetType.GRIPPER_POSITION: "gripper_positions.json",
        PresetType.TASK_TEMPLATE: "task_templates.json",
    }
    
    def __init__(self, storage_path: str = None):
        """
        初始化预设管理器
        
        Args:
            storage_path: 存储路径，默认为 ~/qyh_jushen_ws/persistent/preset/
        """
        if storage_path is None:
            # 默认路径
            home = Path.home()
            storage_path = home / "qyh_jushen_ws" / "persistent" / "preset"
        
        self.storage_path = Path(storage_path)
        self.storage_path.mkdir(parents=True, exist_ok=True)
        
        # 数据缓存
        self._cache: Dict[PresetType, Dict[str, PresetBase]] = {
            t: {} for t in PresetType
        }
        
        # 线程锁
        self._lock = Lock()
        
        # 初始化内置预设（先初始化默认值）
        self._init_builtin_presets()
        
        # 加载用户预设
        self._load_all()
        
        # 加载内置预设的用户更新（覆盖默认值）
        self._load_builtin_updates()
    
    def _load_all(self):
        """加载所有用户预设文件"""
        for preset_type, filename in self.FILE_MAP.items():
            filepath = self.storage_path / filename
            if filepath.exists():
                try:
                    with open(filepath, 'r', encoding='utf-8') as f:
                        data = json.load(f)
                    
                    model_class = self.MODEL_MAP[preset_type]
                    for item in data.get('items', []):
                        try:
                            preset = model_class(**item)
                            self._cache[preset_type][preset.id] = preset
                        except Exception as e:
                            print(f"⚠️  加载预设失败 [{preset_type}]: {e}")
                    
                    print(f"✅ 加载 {len(self._cache[preset_type])} 个 {preset_type.value} 用户预设")
                except Exception as e:
                    print(f"❌ 加载预设文件失败 [{filename}]: {e}")
    
    def _load_builtin_updates(self):
        """加载内置预设的用户更新"""
        for preset_type, filename in self.FILE_MAP.items():
            builtin_filepath = self.storage_path / f"builtin_{filename}"
            if builtin_filepath.exists():
                try:
                    with open(builtin_filepath, 'r', encoding='utf-8') as f:
                        data = json.load(f)
                    
                    model_class = self.MODEL_MAP[preset_type]
                    for item in data.get('items', []):
                        try:
                            preset = model_class(**item)
                            # 只覆盖内置预设
                            if preset.is_builtin:
                                self._cache[preset_type][preset.id] = preset
                        except Exception as e:
                            print(f"⚠️  加载内置预设更新失败 [{preset_type}]: {e}")
                    
                    print(f"✅ 加载内置预设更新 [{preset_type.value}]")
                except Exception as e:
                    print(f"❌ 加载内置预设更新文件失败 [{builtin_filepath}]: {e}")
    
    def _save(self, preset_type: PresetType):
        """保存指定类型的预设到文件"""
        filename = self.FILE_MAP[preset_type]
        filepath = self.storage_path / filename
        
        # 过滤掉内置预设（内置预设不保存到文件）
        items = [
            p.model_dump() for p in self._cache[preset_type].values()
            if not p.is_builtin
        ]
        
        data = {
            "type": preset_type.value,
            "version": "1.0",
            "updated_at": datetime.now().isoformat(),
            "items": items
        }
        
        # 先写入临时文件，再重命名（原子操作）
        temp_filepath = filepath.with_suffix('.tmp')
        with open(temp_filepath, 'w', encoding='utf-8') as f:
            json.dump(data, f, ensure_ascii=False, indent=2)
        
        shutil.move(str(temp_filepath), str(filepath))
    
    def _init_builtin_presets(self):
        """初始化内置预设"""
        # 内置底盘点位
        builtin_locations = [
            Location(id="loc_origin", name="原点", description="坐标原点", 
                    x=0.0, y=0.0, theta=0.0, is_builtin=True, category="builtin"),
            Location(id="loc_charging", name="充电桩", description="充电位置",
                    x=0.0, y=0.0, theta=0.0, is_builtin=True, category="builtin"),
        ]
        for loc in builtin_locations:
            if loc.id not in self._cache[PresetType.LOCATION]:
                self._cache[PresetType.LOCATION][loc.id] = loc
        
        # 内置手臂姿态 - 只有零位和初始点两个
        builtin_arm_poses = [
            ArmPose(
                id="pose_zero", name="零位", description="机械臂零位（各关节角度为0）",
                side="both", pose_type="joint",
                left_joints=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                right_joints=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                is_builtin=True, category="builtin"
            ),
            ArmPose(
                id="pose_home", name="初始点", description="机械臂初始工作位置（可更新）",
                side="both", pose_type="joint",
                left_joints=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                right_joints=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                is_builtin=True, category="builtin"
            ),
        ]
        for pose in builtin_arm_poses:
            # 对于内置预设，总是更新（从文件加载可能有用户更新的值）
            self._cache[PresetType.ARM_POSE][pose.id] = pose
        
        # 内置升降高度
        builtin_lift_heights = [
            LiftHeight(id="lift_bottom", name="底部", description="最低位置",
                      height=0.0, is_builtin=True, category="builtin"),
            LiftHeight(id="lift_middle", name="中间", description="中间位置",
                      height=250.0, is_builtin=True, category="builtin"),
            LiftHeight(id="lift_top", name="顶部", description="最高位置",
                      height=500.0, is_builtin=True, category="builtin"),
            LiftHeight(id="lift_desk", name="桌面平齐", description="标准桌面高度",
                      height=180.0, is_builtin=True, category="builtin"),
        ]
        for height in builtin_lift_heights:
            if height.id not in self._cache[PresetType.LIFT_HEIGHT]:
                self._cache[PresetType.LIFT_HEIGHT][height.id] = height
        
        # 内置头部位置
        builtin_head_positions = [
            HeadPosition(id="head_center", name="正前方", description="看向正前方",
                        pan=0.0, tilt=0.0, is_builtin=True, category="builtin"),
            HeadPosition(id="head_left", name="左侧", description="看向左侧",
                        pan=-0.8, tilt=0.0, is_builtin=True, category="builtin"),
            HeadPosition(id="head_right", name="右侧", description="看向右侧",
                        pan=0.8, tilt=0.0, is_builtin=True, category="builtin"),
            HeadPosition(id="head_up", name="抬头", description="向上看",
                        pan=0.0, tilt=-0.5, is_builtin=True, category="builtin"),
            HeadPosition(id="head_down", name="低头", description="向下看",
                        pan=0.0, tilt=0.5, is_builtin=True, category="builtin"),
        ]
        for pos in builtin_head_positions:
            if pos.id not in self._cache[PresetType.HEAD_POSITION]:
                self._cache[PresetType.HEAD_POSITION][pos.id] = pos
        
        # 内置夹爪位置
        builtin_gripper_positions = [
            GripperPosition(id="gripper_open", name="完全打开", description="夹爪完全张开",
                           side="both", left_position=1.0, right_position=1.0,
                           is_builtin=True, category="builtin"),
            GripperPosition(id="gripper_close", name="完全关闭", description="夹爪完全闭合",
                           side="both", left_position=0.0, right_position=0.0,
                           is_builtin=True, category="builtin"),
            GripperPosition(id="gripper_half", name="半开", description="夹爪半开状态",
                           side="both", left_position=0.5, right_position=0.5,
                           is_builtin=True, category="builtin"),
            GripperPosition(id="gripper_left_open", name="左夹爪打开", description="仅左夹爪打开",
                           side="left", left_position=1.0,
                           is_builtin=True, category="builtin"),
            GripperPosition(id="gripper_right_open", name="右夹爪打开", description="仅右夹爪打开",
                           side="right", right_position=1.0,
                           is_builtin=True, category="builtin"),
        ]
        for pos in builtin_gripper_positions:
            if pos.id not in self._cache[PresetType.GRIPPER_POSITION]:
                self._cache[PresetType.GRIPPER_POSITION][pos.id] = pos
        
        print("✅ 内置预设初始化完成")
    
    # ==================== CRUD 操作 ====================
    
    def list(
        self, 
        preset_type: PresetType,
        category: str = None,
        include_builtin: bool = True
    ) -> List[PresetBase]:
        """
        获取预设列表
        
        Args:
            preset_type: 预设类型
            category: 筛选分类
            include_builtin: 是否包含内置预设
        
        Returns:
            预设列表
        """
        with self._lock:
            items = list(self._cache[preset_type].values())
            
            if not include_builtin:
                items = [i for i in items if not i.is_builtin]
            
            if category:
                items = [i for i in items if i.category == category]
            
            return items
    
    def get(self, preset_type: PresetType, preset_id: str) -> Optional[PresetBase]:
        """
        获取单个预设
        
        Args:
            preset_type: 预设类型
            preset_id: 预设 ID
        
        Returns:
            预设对象或 None
        """
        with self._lock:
            return self._cache[preset_type].get(preset_id)
    
    def get_by_name(self, preset_type: PresetType, name: str) -> Optional[PresetBase]:
        """
        通过名称获取预设
        
        Args:
            preset_type: 预设类型
            name: 预设名称
        
        Returns:
            预设对象或 None
        """
        with self._lock:
            for preset in self._cache[preset_type].values():
                if preset.name == name:
                    return preset
            return None
    
    def create(self, preset_type: PresetType, data: Dict[str, Any]) -> PresetBase:
        """
        创建新预设
        
        Args:
            preset_type: 预设类型
            data: 预设数据
        
        Returns:
            创建的预设对象
        """
        with self._lock:
            # 生成 ID
            if 'id' not in data:
                prefix = preset_type.value[:3]
                data['id'] = f"{prefix}_{uuid.uuid4().hex[:8]}"
            
            # 设置时间戳
            now = datetime.now().isoformat()
            data['created_at'] = now
            data['updated_at'] = now
            data['is_builtin'] = False
            
            # 创建对象
            model_class = self.MODEL_MAP[preset_type]
            preset = model_class(**data)
            
            # 保存到缓存
            self._cache[preset_type][preset.id] = preset
            
            # 持久化
            self._save(preset_type)
            
            return preset
    
    def update(
        self, 
        preset_type: PresetType, 
        preset_id: str, 
        data: Dict[str, Any]
    ) -> Optional[PresetBase]:
        """
        更新预设
        
        Args:
            preset_type: 预设类型
            preset_id: 预设 ID
            data: 更新数据
        
        Returns:
            更新后的预设对象或 None
        """
        with self._lock:
            existing = self._cache[preset_type].get(preset_id)
            if not existing:
                return None
            
            # 内置预设不能通过普通update修改（使用update_builtin）
            if existing.is_builtin:
                raise ValueError("内置预设不能修改，请使用 update_builtin 方法")
            
            # 合并数据
            existing_data = existing.model_dump()
            existing_data.update(data)
            existing_data['updated_at'] = datetime.now().isoformat()
            
            # 创建新对象
            model_class = self.MODEL_MAP[preset_type]
            updated = model_class(**existing_data)
            
            # 更新缓存
            self._cache[preset_type][preset_id] = updated
            
            # 持久化
            self._save(preset_type)
            
            return updated
    
    def update_builtin(
        self,
        preset_type: PresetType,
        preset_id: str,
        data: Dict[str, Any],
        side: str = None
    ) -> Optional[PresetBase]:
        """
        更新内置预设的数据（只更新数据，不改变名称等元信息）
        
        特别用于"初始点"这样允许用户更新数据的内置预设
        支持只更新左/右臂
        
        Args:
            preset_type: 预设类型
            preset_id: 预设 ID
            data: 更新数据
            side: 可选，only 'left' or 'right' 单独更新左/右
        
        Returns:
            更新后的预设对象或 None
        """
        with self._lock:
            existing = self._cache[preset_type].get(preset_id)
            if not existing:
                return None
            
            if not existing.is_builtin:
                raise ValueError("此方法仅用于更新内置预设")
            
            # 只允许更新特定的可更新内置预设（如初始点）
            updatable_builtin_ids = ['pose_home']  # 初始点可更新
            if preset_id not in updatable_builtin_ids:
                raise ValueError(f"内置预设 '{preset_id}' 不允许修改")
            
            existing_data = existing.model_dump()
            
            # 对于手臂姿态，支持单独更新左/右
            if preset_type == PresetType.ARM_POSE and side:
                if side == 'left' and 'left_joints' in data:
                    existing_data['left_joints'] = data['left_joints']
                elif side == 'right' and 'right_joints' in data:
                    existing_data['right_joints'] = data['right_joints']
            else:
                # 更新所有允许的字段
                for key in ['left_joints', 'right_joints', 'left_cartesian', 'right_cartesian',
                           'velocity', 'acceleration']:
                    if key in data:
                        existing_data[key] = data[key]
            
            existing_data['updated_at'] = datetime.now().isoformat()
            
            # 创建新对象
            model_class = self.MODEL_MAP[preset_type]
            updated = model_class(**existing_data)
            
            # 更新缓存
            self._cache[preset_type][preset_id] = updated
            
            # 持久化内置预设更新到特殊文件
            self._save_builtin_updates(preset_type)
            
            return updated
    
    def _save_builtin_updates(self, preset_type: PresetType):
        """保存内置预设的更新到单独文件"""
        filename = f"builtin_{self.FILE_MAP[preset_type]}"
        filepath = self.storage_path / filename
        
        # 只保存被修改过的内置预设
        items = [
            p.model_dump() for p in self._cache[preset_type].values()
            if p.is_builtin
        ]
        
        data = {
            "type": preset_type.value,
            "version": "1.0",
            "updated_at": datetime.now().isoformat(),
            "items": items
        }
        
        temp_filepath = filepath.with_suffix('.tmp')
        with open(temp_filepath, 'w', encoding='utf-8') as f:
            json.dump(data, f, ensure_ascii=False, indent=2)
        
        shutil.move(str(temp_filepath), str(filepath))
    
    def delete(self, preset_type: PresetType, preset_id: str) -> bool:
        """
        删除预设
        
        Args:
            preset_type: 预设类型
            preset_id: 预设 ID
        
        Returns:
            是否删除成功
        """
        with self._lock:
            existing = self._cache[preset_type].get(preset_id)
            if not existing:
                return False
            
            # 内置预设不能删除
            if existing.is_builtin:
                raise ValueError("内置预设不能删除")
            
            # 从缓存删除
            del self._cache[preset_type][preset_id]
            
            # 持久化
            self._save(preset_type)
            
            return True
    
    # ==================== 便捷方法 ====================
    
    def get_location(self, name_or_id: str) -> Optional[Location]:
        """获取点位（支持名称或ID）"""
        preset = self.get(PresetType.LOCATION, name_or_id)
        if not preset:
            preset = self.get_by_name(PresetType.LOCATION, name_or_id)
        return preset
    
    def get_arm_pose(self, name_or_id: str) -> Optional[ArmPose]:
        """获取手臂姿态（支持名称或ID）"""
        preset = self.get(PresetType.ARM_POSE, name_or_id)
        if not preset:
            preset = self.get_by_name(PresetType.ARM_POSE, name_or_id)
        return preset
    
    def get_lift_height(self, name_or_id: str) -> Optional[LiftHeight]:
        """获取升降高度（支持名称或ID）"""
        preset = self.get(PresetType.LIFT_HEIGHT, name_or_id)
        if not preset:
            preset = self.get_by_name(PresetType.LIFT_HEIGHT, name_or_id)
        return preset
    
    def get_head_position(self, name_or_id: str) -> Optional[HeadPosition]:
        """获取头部位置（支持名称或ID）"""
        preset = self.get(PresetType.HEAD_POSITION, name_or_id)
        if not preset:
            preset = self.get_by_name(PresetType.HEAD_POSITION, name_or_id)
        return preset
    
    def get_gripper_position(self, name_or_id: str) -> Optional[GripperPosition]:
        """获取夹爪位置（支持名称或ID）"""
        preset = self.get(PresetType.GRIPPER_POSITION, name_or_id)
        if not preset:
            preset = self.get_by_name(PresetType.GRIPPER_POSITION, name_or_id)
        return preset
    
    def get_task_template(self, name_or_id: str) -> Optional[TaskTemplate]:
        """获取任务模板（支持名称或ID）"""
        preset = self.get(PresetType.TASK_TEMPLATE, name_or_id)
        if not preset:
            preset = self.get_by_name(PresetType.TASK_TEMPLATE, name_or_id)
        return preset
    
    def get_categories(self, preset_type: PresetType) -> List[str]:
        """获取所有分类"""
        with self._lock:
            categories = set()
            for preset in self._cache[preset_type].values():
                categories.add(preset.category)
            return sorted(list(categories))
    
    def export_all(self) -> Dict[str, Any]:
        """导出所有预设"""
        with self._lock:
            result = {}
            for preset_type in PresetType:
                result[preset_type.value] = [
                    p.model_dump() for p in self._cache[preset_type].values()
                    if not p.is_builtin
                ]
            return result
    
    def import_presets(self, data: Dict[str, Any], overwrite: bool = False):
        """导入预设"""
        for type_str, items in data.items():
            try:
                preset_type = PresetType(type_str)
                for item in items:
                    preset_id = item.get('id')
                    if preset_id and preset_id in self._cache[preset_type]:
                        if overwrite:
                            self.update(preset_type, preset_id, item)
                    else:
                        self.create(preset_type, item)
            except ValueError:
                print(f"⚠️  未知预设类型: {type_str}")


# 全局单例
preset_manager = PresetManager()
