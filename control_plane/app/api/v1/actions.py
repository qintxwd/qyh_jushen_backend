"""
QYH Jushen Control Plane - 动作管理 API

提供训练动作的元数据管理:
- 动作 CRUD
- 轨迹列表查询
- 推理配置获取
- 动作状态管理

动作存储位置: ~/qyh-robot-system/model_actions/{robot_name}/{version}/{action_id}/
目录结构:
  action.yaml     - 动作配置
  data/
    episodes/     - HDF5 轨迹文件
    bags/         - 原始 rosbag
  model/
    policy.pt     - 训练模型
"""
import os
import re
import hashlib
import logging
from pathlib import Path
from datetime import datetime
from typing import Optional, List, Literal

from fastapi import APIRouter, Depends, HTTPException, Query

from app.dependencies import get_current_user, get_current_operator
from app.models.user import User
from app.schemas.response import ApiResponse, ErrorCodes, success_response, error_response
from app.schemas.action import (
    ActionSummary,
    ActionDetail,
    CreateActionRequest,
    UpdateActionRequest,
    MarkTrainedRequest,
    EpisodeInfo,
    InferenceConfig,
    ActionStatus,
    DEFAULT_ROBOT_NAME,
    DEFAULT_ROBOT_VERSION,
)

logger = logging.getLogger(__name__)
router = APIRouter()

# 基础模型动作目录
MODEL_ACTIONS_BASE = Path(os.path.expanduser("~/qyh-robot-system/model_actions"))
ACTION_ID_PATTERN = re.compile(r"^[a-z0-9][a-z0-9_-]{0,63}$")


# ============================================================================
# 辅助函数
# ============================================================================

def get_model_actions_dir(robot_name: str = None, robot_version: str = None) -> Path:
    """获取指定机器人和版本的模型动作目录"""
    name = robot_name or DEFAULT_ROBOT_NAME
    version = robot_version or DEFAULT_ROBOT_VERSION
    return MODEL_ACTIONS_BASE / name / version


def _determine_status(action_dir: Path, meta: dict) -> ActionStatus:
    """确定动作状态"""
    model_path = action_dir / "model" / "policy.pt"
    model_ckpt_path = action_dir / "model" / "policy_best.ckpt"
    has_model = model_path.exists() or model_ckpt_path.exists()
    
    config_status = meta.get('status')
    
    if config_status == 'trained':
        return 'trained'
    elif config_status == 'collecting':
        return 'collecting'
    else:
        return 'trained' if has_model else 'collecting'


def _count_episodes(action_dir: Path) -> int:
    """统计已采集的轨迹数量"""
    count = 0
    
    # 统计 HDF5 文件
    episodes_dir = action_dir / "data" / "episodes"
    if episodes_dir.exists():
        count += len(list(episodes_dir.glob("*.hdf5")))
    
    # 统计 bag 目录
    bags_dir = action_dir / "data" / "bags"
    if bags_dir.exists():
        for item in bags_dir.iterdir():
            if item.is_dir() and (item / "metadata.yaml").exists():
                count += 1
    
    return count


def _load_action_yaml(action_file: Path) -> dict:
    """加载 action.yaml"""
    try:
        import yaml
        with open(action_file, 'r', encoding='utf-8') as f:
            return yaml.safe_load(f) or {}
    except Exception as e:
        logger.error(f"Failed to load {action_file}: {e}")
        return {}


def _save_action_yaml(action_file: Path, data: dict):
    """保存 action.yaml"""
    import yaml
    with open(action_file, 'w', encoding='utf-8') as f:
        yaml.dump(data, f, allow_unicode=True, default_flow_style=False, sort_keys=False)


def _extract_topics(collection: dict) -> List[str]:
    """从采集配置提取话题列表"""
    topics = []
    
    for cam in collection.get('cameras', []):
        if topic := cam.get('topic'):
            topics.append(topic)
    
    if joints := collection.get('joints'):
        if topic := joints.get('topic'):
            topics.append(topic)
    
    for g in collection.get('grippers', []):
        if topic := g.get('topic'):
            topics.append(topic)
    
    return topics


def _validate_action_id(action_id: str):
    if not ACTION_ID_PATTERN.fullmatch(action_id):
        return error_response(
            code=ErrorCodes.INVALID_PARAMS,
            message="动作 ID 仅允许小写字母、数字、下划线和短横线",
        )
    return None


def _build_action_summary(
    action_dir: Path,
    data: dict,
    robot_name: str,
    robot_version: str
) -> ActionSummary:
    """构建动作摘要"""
    meta = data.get('metadata', {})
    coll = data.get('collection', {})
    
    model_path = action_dir / "model" / "policy.pt"
    model_ckpt_path = action_dir / "model" / "policy_best.ckpt"
    has_model = model_path.exists() or model_ckpt_path.exists()
    
    return ActionSummary(
        id=meta.get('id', action_dir.name),
        name=meta.get('name', action_dir.name),
        description=meta.get('description', ''),
        version=meta.get('version', '1.0.0'),
        tags=meta.get('tags', []),
        status=_determine_status(action_dir, meta),
        has_model=has_model,
        episode_count=meta.get('episode_count', 0) or _count_episodes(action_dir),
        model_version=meta.get('model_version', 0),
        topics=_extract_topics(coll),
        camera_count=len(coll.get('cameras', [])),
        robot_name=robot_name,
        robot_version=robot_version,
        created_at=meta.get('created_at', ''),
        updated_at=meta.get('updated_at', ''),
    )


# ============================================================================
# API 端点
# ============================================================================

@router.get("/robot-info", response_model=ApiResponse, summary="获取机器人信息")
async def get_robot_info(
    current_user: User = Depends(get_current_user),
):
    """获取当前机器人类型和版本信息"""
    actions_dir = get_model_actions_dir()
    return success_response(
        data={
            "robot_name": DEFAULT_ROBOT_NAME,
            "robot_version": DEFAULT_ROBOT_VERSION,
            "actions_dir": str(actions_dir),
        },
        message="获取机器人信息成功"
    )


@router.get("", response_model=ApiResponse, summary="列出所有动作")
async def list_actions(
    status: Optional[ActionStatus] = Query(None, description="状态过滤"),
    robot_name: Optional[str] = Query(None, description="机器人类型"),
    robot_version: Optional[str] = Query(None, description="机器人版本"),
    current_user: User = Depends(get_current_user),
):
    """
    列出所有可用动作
    
    支持按状态过滤: collecting（采集中）/ trained（已训练）
    """
    rn = robot_name or DEFAULT_ROBOT_NAME
    rv = robot_version or DEFAULT_ROBOT_VERSION
    model_actions_dir = get_model_actions_dir(rn, rv)
    
    if not model_actions_dir.exists():
        model_actions_dir.mkdir(parents=True, exist_ok=True)
        return success_response(
            data={
                "robot_name": rn,
                "robot_version": rv,
                "total": 0,
                "items": [],
            },
            message="获取动作列表成功"
        )
    
    actions = []
    
    for action_dir in model_actions_dir.iterdir():
        if not action_dir.is_dir():
            continue
        
        action_file = action_dir / "action.yaml"
        if not action_file.exists():
            continue
        
        try:
            data = _load_action_yaml(action_file)
            meta = data.get('metadata', {})
            
            action_status = _determine_status(action_dir, meta)
            if status and action_status != status:
                continue
            
            actions.append(_build_action_summary(action_dir, data, rn, rv).model_dump())
        except Exception as e:
            logger.error(f"Failed to read {action_file}: {e}")
    
    return success_response(
        data={
            "robot_name": rn,
            "robot_version": rv,
            "total": len(actions),
            "items": actions,
            "actions": actions,
        },
        message="获取动作列表成功"
    )


@router.get("/trained", response_model=ApiResponse, summary="列出已训练动作")
async def list_trained_actions(
    robot_name: Optional[str] = Query(None, description="机器人类型"),
    robot_version: Optional[str] = Query(None, description="机器人版本"),
    current_user: User = Depends(get_current_user),
):
    """列出所有已训练的动作（可用于推理执行）"""
    return await list_actions(
        status="trained",
        robot_name=robot_name,
        robot_version=robot_version,
        current_user=current_user
    )


@router.get("/collecting", response_model=ApiResponse, summary="列出采集中动作")
async def list_collecting_actions(
    robot_name: Optional[str] = Query(None, description="机器人类型"),
    robot_version: Optional[str] = Query(None, description="机器人版本"),
    current_user: User = Depends(get_current_user),
):
    """列出所有数据采集中的动作"""
    return await list_actions(
        status="collecting",
        robot_name=robot_name,
        robot_version=robot_version,
        current_user=current_user
    )


@router.post("", response_model=ApiResponse, summary="创建新动作")
async def create_action(
    request: CreateActionRequest,
    robot_name: Optional[str] = Query(None, description="机器人类型"),
    robot_version: Optional[str] = Query(None, description="机器人版本"),
    current_user: User = Depends(get_current_operator),
):
    """
    创建新动作（初始状态为 collecting）
    
    可选择基于模板创建
    """
    invalid = _validate_action_id(request.id)
    if invalid:
        return invalid
    if request.template:
        invalid_template = _validate_action_id(request.template)
        if invalid_template:
            return invalid_template

    model_actions_dir = get_model_actions_dir(robot_name, robot_version)
    action_dir = model_actions_dir / request.id
    
    if action_dir.exists():
        return error_response(
            code=ErrorCodes.VALIDATION_ERROR,
            message=f"动作 '{request.id}' 已存在"
        )
    
    try:
        import yaml
        import shutil
        
        # 创建目录结构
        action_dir.mkdir(parents=True)
        (action_dir / "model").mkdir()
        (action_dir / "data" / "episodes").mkdir(parents=True)
        (action_dir / "data" / "bags").mkdir(parents=True)
        
        # 从模板复制或创建默认配置
        if request.template:
            template_file = model_actions_dir / request.template / "action.yaml"
            if template_file.exists():
                shutil.copy(template_file, action_dir / "action.yaml")
                data = _load_action_yaml(action_dir / "action.yaml")
            else:
                return error_response(
                    code=ErrorCodes.NOT_FOUND,
                    message=f"模板动作 '{request.template}' 不存在"
                )
        else:
            data = {
                'metadata': {},
                'collection': {
                    'cameras': [],
                    'joints': {},
                    'grippers': [],
                },
                'training': {},
                'inference': {},
            }
        
        # 更新元数据
        now = datetime.now().isoformat()
        data['metadata'] = {
            **data.get('metadata', {}),
            'id': request.id,
            'name': request.name,
            'description': request.description,
            'status': 'collecting',
            'episode_count': 0,
            'last_training': None,
            'model_version': None,
            'created_at': now,
            'updated_at': now,
        }
        
        _save_action_yaml(action_dir / "action.yaml", data)
        
        return success_response(
            data=_build_action_summary(
                action_dir, data,
                robot_name or DEFAULT_ROBOT_NAME,
                robot_version or DEFAULT_ROBOT_VERSION
            ).model_dump(),
            message=f"动作 '{request.id}' 创建成功"
        )
        
    except Exception as e:
        logger.error(f"Failed to create action: {e}")
        # 清理失败的创建
        if action_dir.exists():
            import shutil
            shutil.rmtree(action_dir)
        return error_response(
            code=ErrorCodes.INTERNAL_ERROR,
            message=f"创建动作失败: {str(e)}"
        )


@router.get("/{action_id}", response_model=ApiResponse, summary="获取动作详情")
async def get_action(
    action_id: str,
    robot_name: Optional[str] = Query(None, description="机器人类型"),
    robot_version: Optional[str] = Query(None, description="机器人版本"),
    current_user: User = Depends(get_current_user),
):
    """获取动作详细信息"""
    invalid = _validate_action_id(action_id)
    if invalid:
        return invalid
    rn = robot_name or DEFAULT_ROBOT_NAME
    rv = robot_version or DEFAULT_ROBOT_VERSION
    model_actions_dir = get_model_actions_dir(rn, rv)
    
    action_dir = model_actions_dir / action_id
    action_file = action_dir / "action.yaml"
    
    if not action_file.exists():
        return error_response(
            code=ErrorCodes.NOT_FOUND,
            message=f"动作 '{action_id}' 不存在"
        )
    
    try:
        data = _load_action_yaml(action_file)
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
        
        return success_response(
            data=action.model_dump(),
            message="获取动作详情成功"
        )
        
    except Exception as e:
        logger.error(f"Failed to load action {action_id}: {e}")
        return error_response(
            code=ErrorCodes.INTERNAL_ERROR,
            message=f"获取动作详情失败: {str(e)}"
        )


@router.put("/{action_id}", response_model=ApiResponse, summary="更新动作")
async def update_action(
    action_id: str,
    request: UpdateActionRequest,
    robot_name: Optional[str] = Query(None, description="机器人类型"),
    robot_version: Optional[str] = Query(None, description="机器人版本"),
    current_user: User = Depends(get_current_operator),
):
    """更新动作配置"""
    invalid = _validate_action_id(action_id)
    if invalid:
        return invalid
    model_actions_dir = get_model_actions_dir(robot_name, robot_version)
    action_dir = model_actions_dir / action_id
    action_file = action_dir / "action.yaml"
    
    if not action_file.exists():
        return error_response(
            code=ErrorCodes.NOT_FOUND,
            message=f"动作 '{action_id}' 不存在"
        )
    
    try:
        data = _load_action_yaml(action_file)
        if 'metadata' not in data or not isinstance(data['metadata'], dict):
            data['metadata'] = {}
        
        # 更新元数据
        if request.name is not None:
            data['metadata']['name'] = request.name
        if request.description is not None:
            data['metadata']['description'] = request.description
        if request.tags is not None:
            data['metadata']['tags'] = request.tags
        
        # 更新配置
        if request.collection is not None:
            data['collection'] = request.collection
        if request.training is not None:
            data['training'] = request.training
        if request.inference is not None:
            data['inference'] = request.inference
        
        data['metadata']['updated_at'] = datetime.now().isoformat()
        
        _save_action_yaml(action_file, data)
        
        return success_response(
            data=_build_action_summary(
                action_dir, data,
                robot_name or DEFAULT_ROBOT_NAME,
                robot_version or DEFAULT_ROBOT_VERSION
            ).model_dump(),
            message="动作更新成功"
        )
        
    except Exception as e:
        logger.error(f"Failed to update action {action_id}: {e}")
        return error_response(
            code=ErrorCodes.INTERNAL_ERROR,
            message=f"更新动作失败: {str(e)}"
        )


@router.delete("/{action_id}", response_model=ApiResponse, summary="删除动作")
async def delete_action(
    action_id: str,
    delete_data: bool = Query(False, description="是否删除数据文件"),
    robot_name: Optional[str] = Query(None, description="机器人类型"),
    robot_version: Optional[str] = Query(None, description="机器人版本"),
    current_user: User = Depends(get_current_operator),
):
    """
    删除动作
    
    - delete_data=False: 仅删除 action.yaml
    - delete_data=True: 删除整个目录（包括数据和模型）
    """
    invalid = _validate_action_id(action_id)
    if invalid:
        return invalid
    model_actions_dir = get_model_actions_dir(robot_name, robot_version)
    action_dir = model_actions_dir / action_id
    
    if not action_dir.exists():
        return error_response(
            code=ErrorCodes.NOT_FOUND,
            message=f"动作 '{action_id}' 不存在"
        )
    
    try:
        import shutil
        
        if delete_data:
            shutil.rmtree(action_dir)
        else:
            action_file = action_dir / "action.yaml"
            if action_file.exists():
                action_file.unlink()
        
        return success_response(
            data={"deleted_id": action_id, "delete_data": delete_data},
            message=f"动作 '{action_id}' 删除成功"
        )
        
    except Exception as e:
        logger.error(f"Failed to delete action {action_id}: {e}")
        return error_response(
            code=ErrorCodes.INTERNAL_ERROR,
            message=f"删除动作失败: {str(e)}"
        )


@router.get("/{action_id}/topics", response_model=ApiResponse, summary="获取录制话题")
async def get_action_topics(
    action_id: str,
    robot_name: Optional[str] = Query(None, description="机器人类型"),
    robot_version: Optional[str] = Query(None, description="机器人版本"),
    current_user: User = Depends(get_current_user),
):
    """获取动作需要录制的话题列表"""
    invalid = _validate_action_id(action_id)
    if invalid:
        return invalid
    model_actions_dir = get_model_actions_dir(robot_name, robot_version)
    action_file = model_actions_dir / action_id / "action.yaml"
    
    if not action_file.exists():
        return error_response(
            code=ErrorCodes.NOT_FOUND,
            message=f"动作 '{action_id}' 不存在"
        )
    
    try:
        data = _load_action_yaml(action_file)
        topics = _extract_topics(data.get('collection', {}))
        
        return success_response(
            data={
                "action_id": action_id,
                "topics": topics,
            },
            message="获取话题列表成功"
        )
        
    except Exception as e:
        logger.error(f"Failed to get topics for {action_id}: {e}")
        return error_response(
            code=ErrorCodes.INTERNAL_ERROR,
            message=f"获取话题列表失败: {str(e)}"
        )


@router.get("/{action_id}/episodes", response_model=ApiResponse, summary="获取轨迹列表")
async def get_action_episodes(
    action_id: str,
    robot_name: Optional[str] = Query(None, description="机器人类型"),
    robot_version: Optional[str] = Query(None, description="机器人版本"),
    current_user: User = Depends(get_current_user),
):
    """获取动作的轨迹文件列表"""
    invalid = _validate_action_id(action_id)
    if invalid:
        return invalid
    model_actions_dir = get_model_actions_dir(robot_name, robot_version)
    action_dir = model_actions_dir / action_id
    
    if not action_dir.exists():
        return error_response(
            code=ErrorCodes.NOT_FOUND,
            message=f"动作 '{action_id}' 不存在"
        )
    
    episodes = []
    base_dir = model_actions_dir
    
    # 扫描 HDF5 文件
    episodes_dir = action_dir / "data" / "episodes"
    if episodes_dir.exists():
        for hdf5_file in episodes_dir.glob("*.hdf5"):
            try:
                stat = hdf5_file.stat()
                rel_path = None
                try:
                    rel_path = str(hdf5_file.relative_to(base_dir))
                except ValueError:
                    rel_path = hdf5_file.name
                episodes.append({
                    "id": hashlib.md5(str(hdf5_file).encode()).hexdigest()[:12],
                    "name": hdf5_file.name,
                    "path": rel_path,
                    "type": "hdf5",
                    "size_mb": round(stat.st_size / (1024 * 1024), 2),
                    "created_at": datetime.fromtimestamp(stat.st_ctime).isoformat(),
                })
            except Exception as e:
                logger.warning(f"Failed to read {hdf5_file}: {e}")
    
    # 扫描 bag 目录
    bags_dir = action_dir / "data" / "bags"
    if bags_dir.exists():
        for bag_dir in bags_dir.iterdir():
            if bag_dir.is_dir() and (bag_dir / "metadata.yaml").exists():
                try:
                    # 计算目录大小
                    total_size = sum(
                        f.stat().st_size for f in bag_dir.rglob("*") if f.is_file()
                    )
                    stat = bag_dir.stat()
                    rel_path = None
                    try:
                        rel_path = str(bag_dir.relative_to(base_dir))
                    except ValueError:
                        rel_path = bag_dir.name
                    episodes.append({
                        "id": hashlib.md5(str(bag_dir).encode()).hexdigest()[:12],
                        "name": bag_dir.name,
                        "path": rel_path,
                        "type": "bag",
                        "size_mb": round(total_size / (1024 * 1024), 2),
                        "created_at": datetime.fromtimestamp(stat.st_ctime).isoformat(),
                    })
                except Exception as e:
                    logger.warning(f"Failed to read {bag_dir}: {e}")
    
    return success_response(
        data={
            "action_id": action_id,
            "total": len(episodes),
            "items": episodes,
        },
        message="获取轨迹列表成功"
    )


@router.delete("/{action_id}/episodes/{episode_id}", response_model=ApiResponse, summary="删除轨迹")
async def delete_episode(
    action_id: str,
    episode_id: str,
    robot_name: Optional[str] = Query(None, description="机器人类型"),
    robot_version: Optional[str] = Query(None, description="机器人版本"),
    current_user: User = Depends(get_current_operator),
):
    """删除指定的轨迹文件"""
    invalid = _validate_action_id(action_id)
    if invalid:
        return invalid
    import shutil
    
    # 先获取轨迹列表
    model_actions_dir = get_model_actions_dir(robot_name, robot_version)
    action_dir = model_actions_dir / action_id
    
    if not action_dir.exists():
        return error_response(
            code=ErrorCodes.NOT_FOUND,
            message=f"动作 '{action_id}' 不存在"
        )
    
    # 查找匹配的轨迹
    target_path = None
    
    episodes_dir = action_dir / "data" / "episodes"
    if episodes_dir.exists():
        for hdf5_file in episodes_dir.glob("*.hdf5"):
            if hashlib.md5(str(hdf5_file).encode()).hexdigest()[:12] == episode_id:
                target_path = hdf5_file
                break
    
    if not target_path:
        bags_dir = action_dir / "data" / "bags"
        if bags_dir.exists():
            for bag_dir in bags_dir.iterdir():
                if bag_dir.is_dir():
                    if hashlib.md5(str(bag_dir).encode()).hexdigest()[:12] == episode_id:
                        target_path = bag_dir
                        break
    
    if not target_path:
        return error_response(
            code=ErrorCodes.NOT_FOUND,
            message=f"轨迹 '{episode_id}' 不存在"
        )
    
    try:
        if target_path.is_file():
            target_path.unlink()
        else:
            shutil.rmtree(target_path)
        
        return success_response(
            data={"deleted_id": episode_id},
            message="轨迹删除成功"
        )
    except Exception as e:
        return error_response(
            code=ErrorCodes.INTERNAL_ERROR,
            message=f"删除轨迹失败: {str(e)}"
        )


@router.post("/{action_id}/update-episode-count", response_model=ApiResponse, summary="更新轨迹数量")
async def update_episode_count(
    action_id: str,
    robot_name: Optional[str] = Query(None, description="机器人类型"),
    robot_version: Optional[str] = Query(None, description="机器人版本"),
    current_user: User = Depends(get_current_operator),
):
    """更新动作的轨迹数量统计"""
    invalid = _validate_action_id(action_id)
    if invalid:
        return invalid
    model_actions_dir = get_model_actions_dir(robot_name, robot_version)
    action_dir = model_actions_dir / action_id
    action_file = action_dir / "action.yaml"
    
    if not action_file.exists():
        return error_response(
            code=ErrorCodes.NOT_FOUND,
            message=f"动作 '{action_id}' 不存在"
        )
    
    try:
        count = _count_episodes(action_dir)
        
        # 使用正则表达式更新，保留文件格式和注释
        with open(action_file, 'r', encoding='utf-8') as f:
            content = f.read()
        
        pattern = r'(episode_count:\s*)(\d+)'
        
        if re.search(pattern, content):
            new_content = re.sub(pattern, f'\\g<1>{count}', content, count=1)
            with open(action_file, 'w', encoding='utf-8') as f:
                f.write(new_content)
        else:
            # 没有 episode_count 字段，用 yaml 添加
            data = _load_action_yaml(action_file)
            if 'metadata' not in data:
                data['metadata'] = {}
            data['metadata']['episode_count'] = count
            _save_action_yaml(action_file, data)
        
        return success_response(
            data={
                "action_id": action_id,
                "episode_count": count,
            },
            message="轨迹数量更新成功"
        )
        
    except Exception as e:
        logger.error(f"Failed to update episode count: {e}")
        return error_response(
            code=ErrorCodes.INTERNAL_ERROR,
            message=f"更新轨迹数量失败: {str(e)}"
        )


@router.post("/{action_id}/mark-trained", response_model=ApiResponse, summary="标记为已训练")
async def mark_action_trained(
    action_id: str,
    request: MarkTrainedRequest,
    robot_name: Optional[str] = Query(None, description="机器人类型"),
    robot_version: Optional[str] = Query(None, description="机器人版本"),
    current_user: User = Depends(get_current_operator),
):
    """标记动作为已训练状态"""
    invalid = _validate_action_id(action_id)
    if invalid:
        return invalid
    model_actions_dir = get_model_actions_dir(robot_name, robot_version)
    action_dir = model_actions_dir / action_id
    action_file = action_dir / "action.yaml"
    
    if not action_file.exists():
        return error_response(
            code=ErrorCodes.NOT_FOUND,
            message=f"动作 '{action_id}' 不存在"
        )
    
    # 检查模型文件
    model_path = action_dir / "model" / "policy.pt"
    model_ckpt_path = action_dir / "model" / "policy_best.ckpt"
    if not model_path.exists() and not model_ckpt_path.exists():
        return error_response(
            code=ErrorCodes.VALIDATION_ERROR,
            message=f"动作 '{action_id}' 没有模型文件"
        )
    
    try:
        data = _load_action_yaml(action_file)
        
        if 'metadata' not in data:
            data['metadata'] = {}
        
        data['metadata']['status'] = 'trained'
        data['metadata']['last_training'] = datetime.now().isoformat()
        data['metadata']['model_version'] = request.model_version
        data['metadata']['updated_at'] = datetime.now().isoformat()
        
        _save_action_yaml(action_file, data)
        
        return success_response(
            data={
                "action_id": action_id,
                "status": "trained",
                "model_version": request.model_version,
            },
            message="动作已标记为已训练"
        )
        
    except Exception as e:
        logger.error(f"Failed to mark action as trained: {e}")
        return error_response(
            code=ErrorCodes.INTERNAL_ERROR,
            message=f"标记失败: {str(e)}"
        )


@router.get("/{action_id}/inference-config", response_model=ApiResponse, summary="获取推理配置")
async def get_inference_config(
    action_id: str,
    robot_name: Optional[str] = Query(None, description="机器人类型"),
    robot_version: Optional[str] = Query(None, description="机器人版本"),
    current_user: User = Depends(get_current_user),
):
    """
    获取推理配置（供 act_inference 节点使用）
    
    只有 trained 状态的动作才能获取推理配置
    """
    invalid = _validate_action_id(action_id)
    if invalid:
        return invalid
    model_actions_dir = get_model_actions_dir(robot_name, robot_version)
    action_dir = model_actions_dir / action_id
    action_file = action_dir / "action.yaml"
    
    if not action_file.exists():
        return error_response(
            code=ErrorCodes.NOT_FOUND,
            message=f"动作 '{action_id}' 不存在"
        )
    
    try:
        data = _load_action_yaml(action_file)
        meta = data.get('metadata', {})
        coll = data.get('collection', {})
        training = data.get('training', {})
        inference = data.get('inference', {})
        
        # 检查状态
        status = _determine_status(action_dir, meta)
        model_path = action_dir / "model" / "policy.pt"
        model_ckpt_path = action_dir / "model" / "policy_best.ckpt"
        
        actual_model_path = model_path if model_path.exists() else model_ckpt_path
        
        if status != 'trained' or not actual_model_path.exists():
            return error_response(
                code=ErrorCodes.VALIDATION_ERROR,
                message=f"动作 '{action_id}' 尚未训练"
            )
        
        norm_path = action_dir / "model" / "normalization.npz"
        
        # 查找话题
        head_rgb_topic = '/camera/head/color/image_raw'
        head_depth_topic = '/camera/head/depth/image_raw'
        for cam in coll.get('cameras', []):
            if cam.get('type') == 'rgb' and 'head' in cam.get('name', ''):
                head_rgb_topic = cam.get('topic', head_rgb_topic)
            elif cam.get('type') == 'depth' and 'head' in cam.get('name', ''):
                head_depth_topic = cam.get('topic', head_depth_topic)
        
        joints_topic = '/right_arm/joint_states'
        if joints := coll.get('joints'):
            joints_topic = joints.get('topic', joints_topic)
        
        config = InferenceConfig(
            action_id=action_id,
            action_name=meta.get('name', action_id),
            model_path=str(actual_model_path),
            normalization_path=str(norm_path) if norm_path.exists() else '',
            
            observation_horizon=training.get('observation_horizon', 10),
            action_horizon=training.get('action_horizon', 20),
            action_steps=training.get('action_exec_horizon', 3),
            
            control_frequency=inference.get('control_frequency', 20.0),
            inference_frequency=inference.get('inference_frequency', 10.0),
            action_scale=inference.get('action_scale', 0.4),
            smoothing_alpha=inference.get('smoothing_alpha', 0.3),
            max_joint_velocity=inference.get('max_joint_velocity', 1.0),
            max_joint_delta=inference.get('max_joint_delta', 0.05),
            gripper_threshold=inference.get('gripper_threshold', 0.6),
            
            action_type=coll.get('action_type', 'absolute'),
            
            head_camera_topic=head_rgb_topic,
            head_camera_depth_topic=head_depth_topic,
            right_arm_state_topic=joints_topic,
            
            use_depth=any(c.get('type') == 'depth' for c in coll.get('cameras', [])),
            camera_names=[c.get('name') for c in coll.get('cameras', []) if c.get('name')],
        )
        
        return success_response(
            data=config.model_dump(),
            message="获取推理配置成功"
        )
        
    except Exception as e:
        logger.error(f"Failed to get inference config: {e}")
        return error_response(
            code=ErrorCodes.INTERNAL_ERROR,
            message=f"获取推理配置失败: {str(e)}"
        )
