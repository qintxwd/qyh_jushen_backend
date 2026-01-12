"""
模型动作管理 API

提供动作定义的查询、同步和管理功能。
所有动作统一存储在 ~/qyh-robot-system/model_actions/ 目录。
动作配置从云端下载后保存到此目录。
"""

from fastapi import APIRouter, HTTPException
from pydantic import BaseModel, Field
from typing import List, Optional, Dict, Any, Literal
import logging
import os
import sys
from pathlib import Path

logger = logging.getLogger(__name__)

router = APIRouter(prefix="/actions", tags=["actions"])

# 统一的模型动作目录
MODEL_ACTIONS_DIR = Path(os.path.expanduser("~/qyh-robot-system/model_actions"))


# ==================== 响应模型 ====================

# 动作状态类型
ActionStatus = Literal["collecting", "trained"]


class ActionSummary(BaseModel):
    """动作摘要"""
    id: str
    name: str
    description: str = ""
    version: str = "1.0.0"
    tags: List[str] = []
    status: ActionStatus = "collecting"  # 动作状态
    has_model: bool = False              # 是否有训练好的模型
    episode_count: int = 0               # 已采集轨迹数
    topics: List[str] = []
    camera_count: int = 0
    created_at: str = ""
    updated_at: str = ""


class ActionDetail(BaseModel):
    """动作详情"""
    id: str
    name: str
    description: str = ""
    version: str = "1.0.0"
    tags: List[str] = []
    status: ActionStatus = "collecting"
    has_model: bool = False
    episode_count: int = 0
    last_training: Optional[str] = None
    model_version: Optional[str] = None
    
    # 采集配置
    collection: Dict[str, Any] = {}
    
    # 训练配置
    training: Dict[str, Any] = {}
    
    # 推理配置
    inference: Dict[str, Any] = {}


class ActionListResponse(BaseModel):
    """动作列表响应"""
    success: bool
    actions: List[ActionSummary] = []
    total: int = 0


class ActionDetailResponse(BaseModel):
    """动作详情响应"""
    success: bool
    action: Optional[ActionDetail] = None
    message: str = ""


class CreateActionRequest(BaseModel):
    """创建动作请求"""
    id: str = Field(..., description="动作ID（英文，如 pickup_cube）")
    name: str = Field(..., description="显示名称（如 夹取方块）")
    description: str = Field("", description="描述")
    template: Optional[str] = Field(None, description="模板动作ID")


class CreateActionResponse(BaseModel):
    """创建动作响应"""
    success: bool
    message: str = ""
    action: Optional[ActionSummary] = None


class TopicsForActionResponse(BaseModel):
    """获取动作话题响应"""
    success: bool
    topics: List[str] = []
    action_id: str = ""


# ==================== 辅助函数 ====================

def _get_registry():
    """获取动作注册表"""
    try:
        training_path = Path(__file__).parent.parent.parent.parent / "qyh_act_training"
        if str(training_path) not in sys.path:
            sys.path.insert(0, str(training_path))
        
        from qyh_act_training.action_registry import ActionRegistry
        return ActionRegistry(str(MODEL_ACTIONS_DIR))
    except ImportError as e:
        logger.warning(f"Cannot import ActionRegistry: {e}")
        return None


def _determine_status(action_dir: Path, meta: dict) -> ActionStatus:
    """
    确定动作状态
    
    优先使用配置中的 status 字段，否则根据模型文件自动判断
    """
    # 检查是否有模型文件
    model_path = action_dir / "model" / "policy.pt"
    has_model = model_path.exists()
    
    # 配置中指定的状态
    config_status = meta.get('status')
    
    if config_status == 'trained':
        return 'trained'
    elif config_status == 'collecting':
        return 'collecting'
    else:
        # 自动判断：有模型就是 trained
        return 'trained' if has_model else 'collecting'


def _count_episodes(action_dir: Path) -> int:
    """统计已采集的轨迹数量"""
    episodes_dir = action_dir / "data" / "episodes"
    if not episodes_dir.exists():
        return 0
    # 统计 HDF5 文件数量
    return len(list(episodes_dir.glob("*.hdf5")))


# ==================== API 路由 ====================

@router.get("/list", response_model=ActionListResponse)
async def list_actions(status: Optional[ActionStatus] = None):
    """
    列出所有可用动作
    
    Args:
        status: 可选，过滤特定状态的动作（collecting/trained）
    
    返回 model_actions 目录中已下载的所有动作
    """
    actions = []
    
    if not MODEL_ACTIONS_DIR.exists():
        MODEL_ACTIONS_DIR.mkdir(parents=True, exist_ok=True)
        return ActionListResponse(success=True, actions=[], total=0)
    
    for action_dir in MODEL_ACTIONS_DIR.iterdir():
        if not action_dir.is_dir():
            continue
            
        action_file = action_dir / "action.yaml"
        if not action_file.exists():
            continue
            
        try:
            import yaml
            with open(action_file, 'r', encoding='utf-8') as f:
                data = yaml.safe_load(f)
            
            meta = data.get('metadata', {})
            coll = data.get('collection', {})
            
            # 检查模型和状态
            model_path = action_dir / "model" / "policy.pt"
            has_model = model_path.exists()
            action_status = _determine_status(action_dir, meta)
            
            # 状态过滤
            if status and action_status != status:
                continue
            
            # 获取话题列表
            topics = []
            for cam in coll.get('cameras', []):
                topic = cam.get('topic')
                if topic:
                    topics.append(topic)
            joints = coll.get('joints')
            if joints and joints.get('topic'):
                topics.append(joints['topic'])
            for g in coll.get('grippers', []):
                topic = g.get('topic')
                if topic:
                    topics.append(topic)
            
            # 统计轨迹数量
            episode_count = meta.get('episode_count', 0) or _count_episodes(action_dir)
            
            actions.append(ActionSummary(
                id=meta.get('id', action_dir.name),
                name=meta.get('name', action_dir.name),
                description=meta.get('description', ''),
                version=meta.get('version', '1.0.0'),
                tags=meta.get('tags', []),
                status=action_status,
                has_model=has_model,
                episode_count=episode_count,
                topics=topics,
                camera_count=len(coll.get('cameras', [])),
                created_at=meta.get('created_at', ''),
                updated_at=meta.get('updated_at', ''),
            ))
        except Exception as e:
            logger.error(f"Failed to read {action_file}: {e}")
    
    return ActionListResponse(
        success=True,
        actions=actions,
        total=len(actions)
    )


@router.get("/trained", response_model=ActionListResponse)
async def list_trained_actions():
    """
    列出所有已训练的动作（可用于执行推理）
    """
    return await list_actions(status="trained")


@router.get("/collecting", response_model=ActionListResponse)
async def list_collecting_actions():
    """
    列出所有数据采集中的动作
    """
    return await list_actions(status="collecting")


@router.get("/{action_id}", response_model=ActionDetailResponse)
async def get_action(action_id: str):
    """
    获取动作详情
    """
    action_file = MODEL_ACTIONS_DIR / action_id / "action.yaml"
    
    if not action_file.exists():
        raise HTTPException(
            status_code=404, 
            detail=f"Action '{action_id}' not found"
        )
    
    try:
        import yaml
        with open(action_file, 'r', encoding='utf-8') as f:
            data = yaml.safe_load(f)
        
        action_dir = MODEL_ACTIONS_DIR / action_id
        meta = data.get('metadata', {})
        model_path = action_dir / "model" / "policy.pt"
        has_model = model_path.exists()
        
        action = ActionDetail(
            id=meta.get('id', action_id),
            name=meta.get('name', action_id),
            description=meta.get('description', ''),
            version=meta.get('version', '1.0.0'),
            tags=meta.get('tags', []),
            status=_determine_status(action_dir, meta),
            has_model=has_model,
            episode_count=meta.get('episode_count', 0) or _count_episodes(action_dir),
            last_training=meta.get('last_training'),
            model_version=meta.get('model_version'),
            collection=data.get('collection', {}),
            training=data.get('training', {}),
            inference=data.get('inference', {}),
        )
        
        return ActionDetailResponse(success=True, action=action)
        
    except Exception as e:
        logger.error(f"Failed to load action {action_id}: {e}")
        raise HTTPException(status_code=500, detail=str(e))


@router.get("/{action_id}/topics", response_model=TopicsForActionResponse)
async def get_action_topics(action_id: str):
    """
    获取动作需要录制的话题列表
    """
    action_file = MODEL_ACTIONS_DIR / action_id / "action.yaml"
    
    if not action_file.exists():
        raise HTTPException(
            status_code=404, 
            detail=f"Action '{action_id}' not found"
        )
    
    try:
        import yaml
        with open(action_file, 'r', encoding='utf-8') as f:
            data = yaml.safe_load(f)
        
        coll = data.get('collection', {})
        topics = []
        
        # 相机话题
        for cam in coll.get('cameras', []):
            topic = cam.get('topic')
            if topic:
                topics.append(topic)
        
        # 关节话题
        joints = coll.get('joints')
        if joints and joints.get('topic'):
            topics.append(joints['topic'])
        
        # 夹爪话题
        for g in coll.get('grippers', []):
            topic = g.get('topic')
            if topic:
                topics.append(topic)
        
        return TopicsForActionResponse(
            success=True,
            topics=topics,
            action_id=action_id
        )
        
    except Exception as e:
        logger.error(f"Failed to get topics for {action_id}: {e}")
        raise HTTPException(status_code=500, detail=str(e))


@router.post("/create", response_model=CreateActionResponse)
async def create_action(request: CreateActionRequest):
    """
    创建新动作（初始状态为 collecting）
    """
    action_dir = MODEL_ACTIONS_DIR / request.id
    
    if action_dir.exists():
        raise HTTPException(
            status_code=400, 
            detail=f"Action '{request.id}' already exists"
        )
    
    try:
        # 创建目录结构
        action_dir.mkdir(parents=True)
        (action_dir / "model").mkdir()
        (action_dir / "data" / "episodes").mkdir(parents=True)
        
        # 复制模板或创建默认配置
        if request.template:
            template_file = MODEL_ACTIONS_DIR / request.template / "action.yaml"
            if template_file.exists():
                import shutil
                shutil.copy(template_file, action_dir / "action.yaml")
                
                # 更新元数据
                import yaml
                with open(action_dir / "action.yaml", 'r', encoding='utf-8') as f:
                    data = yaml.safe_load(f)
                
                data['metadata']['id'] = request.id
                data['metadata']['name'] = request.name
                data['metadata']['description'] = request.description
                data['metadata']['status'] = 'collecting'
                data['metadata']['episode_count'] = 0
                data['metadata']['last_training'] = None
                data['metadata']['model_version'] = None
                
                with open(action_dir / "action.yaml", 'w', encoding='utf-8') as f:
                    yaml.dump(data, f, allow_unicode=True, default_flow_style=False)
        
        return CreateActionResponse(
            success=True,
            message=f"Created action: {request.id}",
        )
        
    except Exception as e:
        logger.error(f"Failed to create action: {e}")
        # 清理失败的创建
        if action_dir.exists():
            import shutil
            shutil.rmtree(action_dir)
        raise HTTPException(status_code=500, detail=str(e))


@router.delete("/{action_id}")
async def delete_action(action_id: str, delete_data: bool = False):
    """
    删除动作
    """
    action_dir = MODEL_ACTIONS_DIR / action_id
    
    if not action_dir.exists():
        raise HTTPException(
            status_code=404, 
            detail=f"Action '{action_id}' not found"
        )
    
    try:
        import shutil
        
        if delete_data:
            shutil.rmtree(action_dir)
        else:
            # 只删除定义文件
            action_file = action_dir / "action.yaml"
            if action_file.exists():
                action_file.unlink()
        
        return {"success": True, "message": f"Deleted action: {action_id}"}
        
    except Exception as e:
        logger.error(f"Failed to delete action: {e}")
        raise HTTPException(status_code=500, detail=str(e))


@router.post("/{action_id}/update-episode-count")
async def update_episode_count(action_id: str):
    """
    更新动作的轨迹数量统计
    """
    action_dir = MODEL_ACTIONS_DIR / action_id
    action_file = action_dir / "action.yaml"
    
    if not action_file.exists():
        raise HTTPException(
            status_code=404, 
            detail=f"Action '{action_id}' not found"
        )
    
    try:
        import yaml
        
        # 统计轨迹数量
        count = _count_episodes(action_dir)
        
        # 更新配置文件
        with open(action_file, 'r', encoding='utf-8') as f:
            data = yaml.safe_load(f)
        
        if 'metadata' not in data:
            data['metadata'] = {}
        data['metadata']['episode_count'] = count
        
        with open(action_file, 'w', encoding='utf-8') as f:
            yaml.dump(data, f, allow_unicode=True, default_flow_style=False)
        
        return {
            "success": True, 
            "action_id": action_id,
            "episode_count": count
        }
        
    except Exception as e:
        logger.error(f"Failed to update episode count: {e}")
        raise HTTPException(status_code=500, detail=str(e))


@router.post("/{action_id}/mark-trained")
async def mark_action_trained(action_id: str, model_version: str = "1.0.0"):
    """
    标记动作为已训练状态
    """
    action_dir = MODEL_ACTIONS_DIR / action_id
    action_file = action_dir / "action.yaml"
    
    if not action_file.exists():
        raise HTTPException(
            status_code=404, 
            detail=f"Action '{action_id}' not found"
        )
    
    # 检查是否有模型文件
    model_path = action_dir / "model" / "policy.pt"
    if not model_path.exists():
        raise HTTPException(
            status_code=400, 
            detail=f"No model file found for action '{action_id}'"
        )
    
    try:
        import yaml
        from datetime import datetime
        
        with open(action_file, 'r', encoding='utf-8') as f:
            data = yaml.safe_load(f)
        
        if 'metadata' not in data:
            data['metadata'] = {}
        
        data['metadata']['status'] = 'trained'
        data['metadata']['last_training'] = datetime.now().isoformat()
        data['metadata']['model_version'] = model_version
        
        with open(action_file, 'w', encoding='utf-8') as f:
            yaml.dump(data, f, allow_unicode=True, default_flow_style=False)
        
        return {
            "success": True, 
            "action_id": action_id,
            "status": "trained",
            "model_version": model_version
        }
        
    except Exception as e:
        logger.error(f"Failed to mark action as trained: {e}")
        raise HTTPException(status_code=500, detail=str(e))


@router.get("/{action_id}/inference-config")
async def get_inference_config(action_id: str):
    """
    获取推理配置（供 act_inference 节点使用）
    
    注意：只有 trained 状态的动作才能获取推理配置
    """
    action_dir = MODEL_ACTIONS_DIR / action_id
    action_file = action_dir / "action.yaml"
    
    if not action_file.exists():
        raise HTTPException(
            status_code=404, 
            detail=f"Action '{action_id}' not found"
        )
    
    try:
        import yaml
        with open(action_file, 'r', encoding='utf-8') as f:
            data = yaml.safe_load(f)
        
        meta = data.get('metadata', {})
        coll = data.get('collection', {})
        training = data.get('training', {})
        inference = data.get('inference', {})
        
        # 检查状态
        status = _determine_status(action_dir, meta)
        model_path = action_dir / "model" / "policy.pt"
        
        if status != 'trained' or not model_path.exists():
            raise HTTPException(
                status_code=400,
                detail=f"Action '{action_id}' is not trained yet"
            )
        
        norm_path = action_dir / "model" / "normalization.npz"
        
        # 查找 RGB 和深度话题
        head_rgb_topic = '/camera/head/color/image_raw'
        head_depth_topic = '/camera/head/depth/image_raw'
        for cam in coll.get('cameras', []):
            if cam.get('type') == 'rgb' and 'head' in cam.get('name', ''):
                head_rgb_topic = cam.get('topic', head_rgb_topic)
            elif cam.get('type') == 'depth' and 'head' in cam.get('name', ''):
                head_depth_topic = cam.get('topic', head_depth_topic)
        
        # 关节话题
        joints_topic = '/right_arm/joint_states'
        joints = coll.get('joints')
        if joints:
            joints_topic = joints.get('topic', joints_topic)
        
        config = {
            'action_id': action_id,
            'action_name': meta.get('name', action_id),
            'model_path': str(model_path),
            'normalization_path': str(norm_path) if norm_path.exists() else '',
            
            'observation_horizon': training.get('observation_horizon', 10),
            'action_horizon': training.get('action_horizon', 20),
            'action_steps': training.get('action_exec_horizon', 3),
            
            'control_frequency': inference.get('control_frequency', 20.0),
            'inference_frequency': inference.get('inference_frequency', 10.0),
            'action_scale': inference.get('action_scale', 0.4),
            'smoothing_alpha': inference.get('smoothing_alpha', 0.3),
            'max_joint_velocity': inference.get('max_joint_velocity', 1.0),
            'max_joint_delta': inference.get('max_joint_delta', 0.05),
            'gripper_threshold': inference.get('gripper_threshold', 0.6),
            
            'action_type': coll.get('action_type', 'absolute'),
            
            'head_camera_topic': head_rgb_topic,
            'head_camera_depth_topic': head_depth_topic,
            'right_arm_state_topic': joints_topic,
            
            'use_depth': any(c.get('type') == 'depth' for c in coll.get('cameras', [])),
            
            'camera_names': [c.get('name') for c in coll.get('cameras', [])],
        }
        
        return {"success": True, "config": config}
        
    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Failed to get inference config: {e}")
        raise HTTPException(status_code=500, detail=str(e))
