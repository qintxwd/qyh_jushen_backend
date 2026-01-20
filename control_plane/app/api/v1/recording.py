"""
QYH Jushen Control Plane - 录制管理 API

提供 rosbag 录制的生命周期管理:
- 开始/停止/放弃录制
- 查询录制状态
- 获取录制文件列表
- 获取可用话题

注意: 实际的录制由 ROS2 节点执行，本 API 通过 ROS2 服务调用控制录制
"""
import os
import hashlib
from pathlib import Path
from datetime import datetime
from typing import Optional, List

from fastapi import APIRouter, Depends, HTTPException, Query

from app.dependencies import get_current_user, get_current_operator
from app.models.user import User
from app.core.control_lock import control_lock
from app.schemas.response import ApiResponse, ErrorCodes, success_response, error_response
from app.schemas.recording import (
    StartRecordingRequest,
    RecordingStatus,
    RecordingResult,
    RecordingFile,
    DefaultTopicsConfig,
)

router = APIRouter()


# ============================================================================
# 内部状态管理（实际应由 ROS2 节点管理）
# ============================================================================

class RecordingState:
    """录制状态管理（临时实现，待 ROS2 集成后替换）"""
    
    def __init__(self):
        self._is_recording = False
        self._action_name = ""
        self._user_name = ""
        self._version = ""
        self._topics: List[str] = []
        self._bag_path = ""
        self._started_at: Optional[datetime] = None
    
    def start(
        self,
        action_name: str,
        user_name: str,
        version: str,
        topics: List[str],
        bag_path: str
    ):
        self._is_recording = True
        self._action_name = action_name
        self._user_name = user_name
        self._version = version
        self._topics = topics
        self._bag_path = bag_path
        self._started_at = datetime.now()
    
    def stop(self) -> tuple[str, float]:
        """停止录制，返回 (bag_path, duration_seconds)"""
        bag_path = self._bag_path
        duration = 0.0
        if self._started_at:
            duration = (datetime.now() - self._started_at).total_seconds()
        self.reset()
        return bag_path, duration
    
    def reset(self):
        self._is_recording = False
        self._action_name = ""
        self._user_name = ""
        self._version = ""
        self._topics = []
        self._bag_path = ""
        self._started_at = None
    
    def get_status(self) -> RecordingStatus:
        duration = 0.0
        if self._started_at:
            duration = (datetime.now() - self._started_at).total_seconds()
        
        status_str = "recording" if self._is_recording else "idle"
        
        return RecordingStatus(
            is_recording=self._is_recording,
            status=status_str,
            action_name=self._action_name,
            user_name=self._user_name,
            version=self._version,
            duration_seconds=duration,
            bag_path=self._bag_path,
            topics=self._topics,
            started_at=self._started_at.isoformat() if self._started_at else None,
        )


# 全局录制状态（单例）
_recording_state = RecordingState()


def get_recording_state() -> RecordingState:
    """获取录制状态实例"""
    return _recording_state


# ============================================================================
# 辅助函数
# ============================================================================

def get_default_topics() -> List[str]:
    """获取默认录制话题"""
    config = DefaultTopicsConfig()
    return (
        config.camera_topics +
        config.joint_topics +
        config.gripper_topics +
        config.other_topics
    )


def generate_bag_path(action_name: str, user_name: str, version: str) -> str:
    """生成录制文件路径"""
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    bag_name = f"qyh_{user_name}_{version}_{timestamp}"
    base_path = os.path.expanduser(f"~/qyh-robot-system/model_actions/{action_name}/data/bags")
    return os.path.join(base_path, bag_name)


def ensure_directory(path: str) -> bool:
    """确保目录存在"""
    try:
        Path(path).mkdir(parents=True, exist_ok=True)
        return True
    except Exception:
        return False


def list_recording_files(action_name: Optional[str] = None) -> List[dict]:
    """列出录制文件"""
    base_path = os.path.expanduser("~/qyh-robot-system/model_actions")
    files: List[dict] = []
    
    if not os.path.exists(base_path):
        return files
    
    # 遍历动作目录
    for action_dir in os.listdir(base_path):
        if action_name and action_dir != action_name:
            continue
        
        bags_path = os.path.join(base_path, action_dir, "data", "bags")
        if not os.path.exists(bags_path):
            continue
        
        for bag_name in os.listdir(bags_path):
            bag_dir = os.path.join(bags_path, bag_name)
            if not os.path.isdir(bag_dir):
                continue
            
            # 计算目录大小
            total_size = 0
            for root, dirs, filenames in os.walk(bag_dir):
                for f in filenames:
                    fp = os.path.join(root, f)
                    try:
                        total_size += os.path.getsize(fp)
                    except OSError:
                        pass
            
            # 解析文件名获取元数据
            # 格式: qyh_{user_name}_{version}_{timestamp}
            parts = bag_name.split("_")
            user_name = parts[1] if len(parts) > 1 else ""
            version = parts[2] if len(parts) > 2 else ""
            
            # 获取创建时间
            try:
                stat = os.stat(bag_dir)
                created_at = datetime.fromtimestamp(stat.st_ctime).isoformat()
            except OSError:
                created_at = ""
            
            # 生成 ID
            file_id = hashlib.md5(bag_dir.encode()).hexdigest()[:12]
            
            files.append({
                "id": file_id,
                "name": bag_name,
                "path": bag_dir,
                "action_name": action_dir,
                "user_name": user_name,
                "version": version,
                "duration_seconds": 0.0,  # 需要从 metadata 读取
                "size_mb": round(total_size / (1024 * 1024), 2),
                "created_at": created_at,
            })
    
    return files


# ============================================================================
# ROS2 服务调用
# ============================================================================

from app.services.ros2_client import get_ros2_client, ROS2ServiceClient


async def call_ros2_start_recording(
    action_name: str,
    user_name: str,
    version: str,
    bag_path: str,
    topics: List[str]
) -> tuple[bool, str]:
    """
    调用 ROS2 服务开始录制
    
    Returns:
        tuple[bool, str]: (success, message)
    """
    # 确保目录存在
    if not ensure_directory(bag_path):
        return False, f"无法创建目录: {bag_path}"
    
    # 获取 ROS2 客户端
    client = get_ros2_client()
    if not client._initialized:
        await client.initialize()
    
    # 调用 ROS2 服务
    result = await client.start_recording(
        action_name=action_name,
        user_name=user_name,
        version=version,
        topics=topics,
    )
    
    return result.success, result.message


async def call_ros2_stop_recording() -> tuple[bool, str, float]:
    """
    调用 ROS2 服务停止录制
    
    Returns:
        tuple[bool, str, float]: (success, message, duration_sec)
    """
    client = get_ros2_client()
    if not client._initialized:
        await client.initialize()
    
    result = await client.stop_recording()
    duration = result.data.get("duration_sec", 0.0) if result.success else 0.0
    
    return result.success, result.message, duration


async def call_ros2_get_recording_status():
    """
    获取 ROS2 录制状态
    
    Returns:
        RecordingStatus from ROS2 client
    """
    client = get_ros2_client()
    if not client._initialized:
        await client.initialize()
    
    return await client.get_recording_status()


async def call_ros2_get_topics() -> List[str]:
    """
    获取当前可用的 ROS2 话题
    
    Returns:
        List[str]: 话题列表
    """
    client = get_ros2_client()
    if not client._initialized:
        await client.initialize()
    return client.get_topic_list()


# ============================================================================
# 依赖注入：控制锁检查
# ============================================================================

async def require_control_lock(
    current_user: User = Depends(get_current_operator),
) -> User:
    """
    要求用户持有控制锁
    
    用于需要独占控制权的操作（如录制）
    """
    if not control_lock.is_held_by(current_user.id):
        raise HTTPException(
            status_code=403,
            detail="需要先获取控制权才能执行此操作"
        )
    return current_user


# ============================================================================
# API 端点
# ============================================================================

@router.post("/start", response_model=ApiResponse, summary="开始录制")
async def start_recording(
    request: StartRecordingRequest,
    state: RecordingState = Depends(get_recording_state),
    current_user: User = Depends(require_control_lock),
):
    """
    开始录制 rosbag
    
    需要先获取控制锁
    """
    # 兼容前端字段
    action_name = request.action_name or request.action_id
    if not action_name:
        return error_response(
            code=ErrorCodes.VALIDATION_ERROR,
            message="action_name or action_id is required"
        )
    user_name = request.user_name or current_user.username
    
    # 检查是否已在录制
    if state._is_recording:
        return error_response(
            code=ErrorCodes.CONTROL_ALREADY_HELD,
            message="已有录制正在进行，请先停止当前录制"
        )
    
    # 确定话题
    topics = request.topics if request.topics else get_default_topics()
    
    # 生成录制路径
    bag_path = generate_bag_path(
        action_name,
        user_name,
        request.version
    )
    
    # 调用 ROS2 服务
    success, message = await call_ros2_start_recording(
        action_name,
        user_name,
        request.version,
        bag_path,
        topics
    )
    
    if success:
        state.start(
            action_name=action_name,
            user_name=user_name,
            version=request.version,
            topics=topics,
            bag_path=bag_path
        )
        return success_response(
            data={
                "success": True,
                "message": message,
                "bag_path": bag_path,
            },
            message="录制已开始"
        )
    else:
        return error_response(
            code=ErrorCodes.INTERNAL_ERROR,
            message=message
        )


@router.post("/stop", response_model=ApiResponse, summary="停止录制")
async def stop_recording(
    state: RecordingState = Depends(get_recording_state),
    current_user: User = Depends(require_control_lock),
):
    """
    停止录制并保存
    
    需要先获取控制锁
    """
    if not state._is_recording:
        return error_response(
            code=ErrorCodes.NOT_FOUND,
            message="没有正在进行的录制"
        )
    
    # 调用 ROS2 服务停止
    success, message = await call_ros2_stop_recording()
    
    bag_path, duration = state.stop()
    
    if success:
        return success_response(
            data={
                "success": True,
                "message": message,
                "duration_seconds": duration,
                "bag_path": bag_path,
            },
            message="录制已停止"
        )
    else:
        return error_response(
            code=ErrorCodes.INTERNAL_ERROR,
            message=message
        )


@router.post("/discard", response_model=ApiResponse, summary="放弃录制")
async def discard_recording(
    state: RecordingState = Depends(get_recording_state),
    current_user: User = Depends(require_control_lock),
):
    """
    放弃录制，停止并删除文件
    
    需要先获取控制锁
    """
    if not state._is_recording:
        return error_response(
            code=ErrorCodes.NOT_FOUND,
            message="没有正在进行的录制"
        )
    
    bag_path = state._bag_path
    
    # 停止录制
    await call_ros2_stop_recording()
    state.reset()
    
    # 删除录制文件
    try:
        import shutil
        if os.path.exists(bag_path):
            shutil.rmtree(bag_path)
    except Exception as e:
        return error_response(
            code=ErrorCodes.INTERNAL_ERROR,
            message=f"停止录制成功，但删除文件失败: {str(e)}"
        )
    
    return success_response(
        data={"discarded": True},
        message="录制已放弃，文件已删除"
    )


@router.get("/status", response_model=ApiResponse, summary="获取录制状态")
async def get_recording_status(
    state: RecordingState = Depends(get_recording_state),
    current_user: User = Depends(get_current_user),
):
    """获取当前录制状态"""
    status = state.get_status()
    return success_response(
        data=status.model_dump(),
        message="获取录制状态成功"
    )


@router.get("/files", response_model=ApiResponse, summary="获取录制文件列表")
async def get_recording_files(
    action_name: Optional[str] = Query(None, description="按动作名称筛选"),
    current_user: User = Depends(get_current_user),
):
    """获取已保存的录制文件列表"""
    files = list_recording_files(action_name)
    
    return success_response(
        data={
            "total": len(files),
            "items": files,
        },
        message="获取录制文件列表成功"
    )


@router.delete("/files/{file_id}", response_model=ApiResponse, summary="删除录制文件")
async def delete_recording_file(
    file_id: str,
    current_user: User = Depends(get_current_operator),
):
    """删除指定的录制文件"""
    # 查找文件
    files = list_recording_files()
    target_file = None
    for f in files:
        if f["id"] == file_id:
            target_file = f
            break
    
    if not target_file:
        return error_response(
            code=ErrorCodes.NOT_FOUND,
            message=f"未找到录制文件: {file_id}"
        )
    
    # 删除文件
    try:
        import shutil
        if os.path.exists(target_file["path"]):
            shutil.rmtree(target_file["path"])
    except Exception as e:
        return error_response(
            code=ErrorCodes.INTERNAL_ERROR,
            message=f"删除失败: {str(e)}"
        )
    
    return success_response(
        data={"deleted_id": file_id},
        message=f"已删除录制文件: {target_file['name']}"
    )


@router.get("/topics", response_model=ApiResponse, summary="获取可用话题")
async def get_available_topics(
    current_user: User = Depends(get_current_user),
):
    """获取当前可用的 ROS2 话题列表"""
    topics = await call_ros2_get_topics()
    message = "获取话题列表成功" if topics else "当前无法从 ROS2 获取话题列表"
    return success_response(
        data={"topics": topics},
        message=message
    )


@router.get("/topics/default", response_model=ApiResponse, summary="获取默认话题配置")
async def get_default_topics_config(
    current_user: User = Depends(get_current_user),
):
    """获取默认录制话题配置"""
    config = DefaultTopicsConfig()
    return success_response(
        data=config.model_dump(),
        message="获取默认话题配置成功"
    )
