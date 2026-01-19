"""
模型动作管理 API

提供动作定义的查询、同步和管理功能。
所有动作统一存储在 ~/qyh-robot-system/model_actions/{robot_name}/{version}/ 目录。
动作配置从云端下载后保存到此目录。

目录结构：
  model_actions/
    general/
      1.0/
        pickup_cube/
          action.yaml
          data/
          model/
"""

from fastapi import APIRouter, Query, HTTPException
from pydantic import BaseModel, Field
from typing import List, Optional, Dict, Any, Literal
import logging
import os
import sys
from pathlib import Path

from app.schemas.response import (
    ApiResponse, success_response, error_response, ErrorCodes
)

logger = logging.getLogger(__name__)

router = APIRouter(prefix="/actions", tags=["actions"])

# 基础模型动作目录
MODEL_ACTIONS_BASE = Path(os.path.expanduser("~/qyh-robot-system/model_actions"))

# 默认机器人类型和版本（从环境变量获取）
DEFAULT_ROBOT_NAME = os.environ.get('GLOBAL_ROBOT_NAME', 'general')
DEFAULT_ROBOT_VERSION = os.environ.get('GLOBAL_ROBOT_VERSION', '1.0')


def get_model_actions_dir(robot_name: str = None, robot_version: str = None) -> Path:
    """获取指定机器人和版本的模型动作目录"""
    name = robot_name or DEFAULT_ROBOT_NAME
    version = robot_version or DEFAULT_ROBOT_VERSION
    return MODEL_ACTIONS_BASE / name / version


# ==================== 响应模型 ====================

# 动作状态类型
ActionStatus = Literal["collecting", "trained"]


class RobotInfo(BaseModel):
    """机器人信息"""
    name: str = DEFAULT_ROBOT_NAME
    version: str = DEFAULT_ROBOT_VERSION


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
    model_version: Optional[int] = 0               # 模型版本号
    topics: List[str] = []
    camera_count: int = 0
    robot_name: str = DEFAULT_ROBOT_NAME      # 机器人类型
    robot_version: str = DEFAULT_ROBOT_VERSION  # 机器人版本
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
    robot_name: str = DEFAULT_ROBOT_NAME      # 机器人类型
    robot_version: str = DEFAULT_ROBOT_VERSION  # 机器人版本
    
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
    robot_name: str = DEFAULT_ROBOT_NAME
    robot_version: str = DEFAULT_ROBOT_VERSION


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


class RobotInfoResponse(BaseModel):
    """机器人信息响应"""
    success: bool
    robot_name: str
    robot_version: str
    actions_dir: str


# ==================== 辅助函数 ====================

def _get_registry(robot_name: str = None, robot_version: str = None):
    """获取动作注册表"""
    try:
        training_path = Path(__file__).parent.parent.parent.parent / "qyh_act_training"
        if str(training_path) not in sys.path:
            sys.path.insert(0, str(training_path))
        
        from qyh_act_training.action_registry import ActionRegistry
        actions_dir = get_model_actions_dir(robot_name, robot_version)
        return ActionRegistry(str(actions_dir))
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
    """
    统计已采集的轨迹数量
    
    统计两个位置：
    1. data/episodes/ - 转换后的 HDF5 文件
    2. data/bags/ - 原始 rosbag 文件（尚未转换）
    """
    count = 0
    
    # 统计 HDF5 文件数量
    episodes_dir = action_dir / "data" / "episodes"
    if episodes_dir.exists():
        count += len(list(episodes_dir.glob("*.hdf5")))
    
    # 统计 bag 目录数量（每个 bag 是一个目录）
    bags_dir = action_dir / "data" / "bags"
    if bags_dir.exists():
        # rosbag2 的每个录制是一个目录
        for item in bags_dir.iterdir():
            if item.is_dir() and (item / "metadata.yaml").exists():
                count += 1
    
    return count


# ==================== API 路由 ====================

@router.get("/robot-info", response_model=RobotInfoResponse)
async def get_robot_info():
    """获取当前机器人信息"""
    actions_dir = get_model_actions_dir()
    return RobotInfoResponse(
        success=True,
        robot_name=DEFAULT_ROBOT_NAME,
        robot_version=DEFAULT_ROBOT_VERSION,
        actions_dir=str(actions_dir)
    )


@router.get("/list", response_model=ActionListResponse)
async def list_actions(
    status: Optional[ActionStatus] = None,
    robot_name: Optional[str] = Query(None, description="机器人类型"),
    robot_version: Optional[str] = Query(None, description="机器人版本")
):
    """
    列出所有可用动作
    
    Args:
        status: 可选，过滤特定状态的动作（collecting/trained）
        robot_name: 机器人类型，默认使用环境变量 GLOBAL_ROBOT_NAME
        robot_version: 机器人版本，默认使用环境变量 GLOBAL_ROBOT_VERSION
    
    返回 model_actions/{robot_name}/{version}/ 目录中的所有动作
    """
    actions = []
    
    rn = robot_name or DEFAULT_ROBOT_NAME
    rv = robot_version or DEFAULT_ROBOT_VERSION
    model_actions_dir = get_model_actions_dir(rn, rv)
    
    if not model_actions_dir.exists():
        model_actions_dir.mkdir(parents=True, exist_ok=True)
        return ActionListResponse(
            success=True, actions=[], total=0,
            robot_name=rn, robot_version=rv
        )
    
    for action_dir in model_actions_dir.iterdir():
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
            model_ckpt_path = action_dir / "model" / "policy_best.ckpt"
            has_model = model_path.exists() or model_ckpt_path.exists()
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
                model_version=meta.get('model_version', 0),
                topics=topics,
                camera_count=len(coll.get('cameras', [])),
                robot_name=rn,
                robot_version=rv,
                created_at=meta.get('created_at', ''),
                updated_at=meta.get('updated_at', ''),
            ))
        except Exception as e:
            logger.error(f"Failed to read {action_file}: {e}")
    
    return ActionListResponse(
        success=True,
        actions=actions,
        total=len(actions),
        robot_name=rn,
        robot_version=rv
    )


@router.get("/trained", response_model=ActionListResponse)
async def list_trained_actions(
    robot_name: Optional[str] = Query(None, description="机器人类型"),
    robot_version: Optional[str] = Query(None, description="机器人版本")
):
    """
    列出所有已训练的动作（可用于执行推理）
    """
    return await list_actions(status="trained", robot_name=robot_name, robot_version=robot_version)


@router.get("/collecting", response_model=ActionListResponse)
async def list_collecting_actions(
    robot_name: Optional[str] = Query(None, description="机器人类型"),
    robot_version: Optional[str] = Query(None, description="机器人版本")
):
    """
    列出所有数据采集中的动作
    """
    return await list_actions(status="collecting", robot_name=robot_name, robot_version=robot_version)


@router.get("/{action_id}", response_model=ActionDetailResponse)
async def get_action(
    action_id: str,
    robot_name: Optional[str] = Query(None, description="机器人类型"),
    robot_version: Optional[str] = Query(None, description="机器人版本")
):
    """
    获取动作详情
    """
    rn = robot_name or DEFAULT_ROBOT_NAME
    rv = robot_version or DEFAULT_ROBOT_VERSION
    model_actions_dir = get_model_actions_dir(rn, rv)
    
    action_file = model_actions_dir / action_id / "action.yaml"
    
    if not action_file.exists():
        raise HTTPException(
            status_code=404, 
            detail=f"Action '{action_id}' not found for robot {rn}/{rv}"
        )
    
    try:
        import yaml
        with open(action_file, 'r', encoding='utf-8') as f:
            data = yaml.safe_load(f)
        
        action_dir = model_actions_dir / action_id
        meta = data.get('metadata', {})
        model_path = action_dir / "model" / "policy.pt"
        model_ckpt_path = action_dir / "model" / "policy_best.ckpt"
        has_model = model_path.exists() or model_ckpt_path.exists()
        
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
            robot_name=rn,
            robot_version=rv,
            collection=data.get('collection', {}),
            training=data.get('training', {}),
            inference=data.get('inference', {}),
        )
        
        return ActionDetailResponse(success=True, action=action)
        
    except Exception as e:
        logger.error(f"Failed to load action {action_id}: {e}")
        raise HTTPException(status_code=500, detail=str(e))


@router.get("/{action_id}/topics", response_model=TopicsForActionResponse)
async def get_action_topics(
    action_id: str,
    robot_name: Optional[str] = Query(None, description="机器人类型"),
    robot_version: Optional[str] = Query(None, description="机器人版本")
):
    """
    获取动作需要录制的话题列表
    """
    model_actions_dir = get_model_actions_dir(robot_name, robot_version)
    action_file = model_actions_dir / action_id / "action.yaml"
    
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
async def create_action(
    request: CreateActionRequest,
    robot_name: Optional[str] = Query(None, description="机器人类型"),
    robot_version: Optional[str] = Query(None, description="机器人版本")
):
    """
    创建新动作（初始状态为 collecting）
    """
    model_actions_dir = get_model_actions_dir(robot_name, robot_version)
    action_dir = model_actions_dir / request.id
    
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
        (action_dir / "data" / "bags").mkdir(parents=True)
        
        # 复制模板或创建默认配置
        if request.template:
            template_file = model_actions_dir / request.template / "action.yaml"
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
async def delete_action(
    action_id: str, 
    delete_data: bool = False,
    robot_name: Optional[str] = Query(None, description="机器人类型"),
    robot_version: Optional[str] = Query(None, description="机器人版本")
):
    """
    删除动作
    """
    model_actions_dir = get_model_actions_dir(robot_name, robot_version)
    action_dir = model_actions_dir / action_id
    
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
async def update_episode_count(
    action_id: str,
    robot_name: Optional[str] = Query(None, description="机器人类型"),
    robot_version: Optional[str] = Query(None, description="机器人版本")
):
    """
    更新动作的轨迹数量统计
    """
    model_actions_dir = get_model_actions_dir(robot_name, robot_version)
    action_dir = model_actions_dir / action_id
    action_file = action_dir / "action.yaml"
    
    if not action_file.exists():
        raise HTTPException(
            status_code=404, 
            detail=f"Action '{action_id}' not found"
        )
    
    try:
        import re as regex
        
        # 统计轨迹数量
        count = _count_episodes(action_dir)
        
        # 使用正则表达式只更新 episode_count 字段，保留文件原有格式和注释
        with open(action_file, 'r', encoding='utf-8') as f:
            content = f.read()
        
        # 匹配 episode_count: 数字 的模式（支持带注释的情况）
        pattern = r'(episode_count:\s*)(\d+)'
        replacement = f'\g<1>{count}'
        
        if regex.search(pattern, content):
            new_content = regex.sub(pattern, replacement, content, count=1)
        else:
            # 如果没有 episode_count 字段，在 metadata 部分添加
            # 这种情况下才需要用 yaml 库
            import yaml
            with open(action_file, 'r', encoding='utf-8') as f:
                data = yaml.safe_load(f)
            if 'metadata' not in data:
                data['metadata'] = {}
            data['metadata']['episode_count'] = count
            with open(action_file, 'w', encoding='utf-8') as f:
                yaml.dump(data, f, allow_unicode=True, default_flow_style=False, sort_keys=False)
            return {
                "success": True, 
                "action_id": action_id,
                "episode_count": count
            }
        
        with open(action_file, 'w', encoding='utf-8') as f:
            f.write(new_content)
        
        return {
            "success": True, 
            "action_id": action_id,
            "episode_count": count
        }
        
    except Exception as e:
        logger.error(f"Failed to update episode count: {e}")
        raise HTTPException(status_code=500, detail=str(e))


@router.post("/{action_id}/mark-trained")
async def mark_action_trained(
    action_id: str, 
    model_version: str = "1.0.0",
    robot_name: Optional[str] = Query(None, description="机器人类型"),
    robot_version: Optional[str] = Query(None, description="机器人版本")
):
    """
    标记动作为已训练状态
    """
    model_actions_dir = get_model_actions_dir(robot_name, robot_version)
    action_dir = model_actions_dir / action_id
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
async def get_inference_config(
    action_id: str,
    robot_name: Optional[str] = Query(None, description="机器人类型"),
    robot_version: Optional[str] = Query(None, description="机器人版本")
):
    """
    获取推理配置（供 act_inference 节点使用）
    
    注意：只有 trained 状态的动作才能获取推理配置
    """
    model_actions_dir = get_model_actions_dir(robot_name, robot_version)
    action_dir = model_actions_dir / action_id
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
