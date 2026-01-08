"""
任务编排 API (Task Orchestration API)

FastAPI 代理层，用于:
- 接收前端的任务 JSON
- 转发给 ROS2 Task Engine
- 订阅任务状态并通过 WebSocket 推送给前端
"""

from fastapi import APIRouter, Depends, HTTPException, WebSocket, WebSocketDisconnect
from pydantic import BaseModel, Field
from typing import Optional, List, Dict, Any
import json
import asyncio
import os
import uuid
import copy
from datetime import datetime
from pathlib import Path

from app.dependencies import get_current_admin
from app.ros2_bridge.bridge import ros2_bridge
from app.safety.watchdog import watchdog

router = APIRouter()

# 任务存储路径
TASKS_DIR = Path.home() / "qyh-robot-system" / "persistent" / "tasks"
TASKS_DIR.mkdir(parents=True, exist_ok=True)


# ============== 请求/响应模型 ==============

class TaskNode(BaseModel):
    """任务节点"""
    id: str
    type: str
    params: Dict[str, Any] = {}
    children: List['TaskNode'] = []

TaskNode.model_rebuild()


class TaskDefinition(BaseModel):
    """任务定义"""
    id: Optional[str] = None
    name: str
    description: Optional[str] = ""
    category: Optional[str] = "default"
    root: TaskNode
    created_at: Optional[str] = None
    updated_at: Optional[str] = None


class TaskListItem(BaseModel):
    """任务列表项"""
    id: str
    name: str
    description: Optional[str] = ""
    category: Optional[str] = "default"
    created_at: Optional[str] = None
    updated_at: Optional[str] = None


class TaskExecuteRequest(BaseModel):
    """任务执行请求"""
    task_json: Dict[str, Any] = Field(default=None, description="任务 JSON 描述")
    debug_mode: bool = Field(default=False, description="是否为调试模式")
    
    # 也可以直接传任务定义
    id: Optional[str] = None
    name: Optional[str] = None
    root: Optional[Dict[str, Any]] = None


class TaskResponse(BaseModel):
    """任务响应"""
    success: bool
    task_id: Optional[str] = None
    message: str


class TaskStatusResponse(BaseModel):
    """任务状态响应"""
    task_id: Optional[str] = None
    task_name: Optional[str] = None
    status: str = "idle"
    current_node_id: Optional[str] = None
    completed_nodes: int = 0
    total_nodes: int = 0
    progress: float = 0.0
    elapsed_time: float = 0.0
    node_statuses: List[Dict[str, Any]] = []


class SkillInfo(BaseModel):
    """技能信息"""
    type: str
    description: str
    params: Dict[str, Any]


# ============== Mock 状态 ==============

_mock_task_status = {
    'task_id': None,
    'task_name': None,
    'status': 'idle',
    'current_node_id': None,
    'completed_nodes': 0,
    'total_nodes': 0,
    'progress': 0.0,
    'elapsed_time': 0.0,
    'node_statuses': []
}


def _get_status_snapshot() -> Dict[str, Any]:
    """获取当前状态的深拷贝快照，确保 node_statuses 中的字典也被拷贝"""
    return {
        'task_id': _mock_task_status['task_id'],
        'task_name': _mock_task_status['task_name'],
        'status': _mock_task_status['status'],
        'current_node_id': _mock_task_status['current_node_id'],
        'completed_nodes': _mock_task_status['completed_nodes'],
        'total_nodes': _mock_task_status['total_nodes'],
        'progress': _mock_task_status['progress'],
        'elapsed_time': _mock_task_status['elapsed_time'],
        'node_statuses': copy.deepcopy(_mock_task_status['node_statuses'])
    }

# 用于 mock 模式的任务模拟器
_mock_execution_task: Optional[asyncio.Task] = None


# ============== 任务 CRUD API ==============

@router.get("/task/list", response_model=List[TaskListItem])
async def list_tasks(
    category: Optional[str] = None,
    current_user=Depends(get_current_admin)
):
    """获取任务列表"""
    tasks = []
    
    for file_path in TASKS_DIR.glob("*.json"):
        try:
            with open(file_path, 'r', encoding='utf-8') as f:
                task_data = json.load(f)
                
            # 过滤类别
            if category and task_data.get('category') != category:
                continue
                
            tasks.append(TaskListItem(
                id=task_data.get('id', file_path.stem),
                name=task_data.get('name', 'Unnamed'),
                description=task_data.get('description', ''),
                category=task_data.get('category', 'default'),
                created_at=task_data.get('created_at'),
                updated_at=task_data.get('updated_at')
            ))
        except Exception as e:
            print(f"Error loading task {file_path}: {e}")
    
    # 按更新时间排序
    tasks.sort(key=lambda t: t.updated_at or '', reverse=True)
    return tasks


@router.get("/task/{task_id}", response_model=TaskDefinition)
async def get_task(
    task_id: str,
    current_user=Depends(get_current_admin)
):
    """获取任务详情"""
    file_path = TASKS_DIR / f"{task_id}.json"
    
    if not file_path.exists():
        raise HTTPException(status_code=404, detail=f"Task '{task_id}' not found")
    
    with open(file_path, 'r', encoding='utf-8') as f:
        task_data = json.load(f)
    
    return TaskDefinition(**task_data)


@router.post("/task/create", response_model=TaskDefinition)
async def create_task(
    task: TaskDefinition,
    current_user=Depends(get_current_admin)
):
    """创建任务"""
    # 生成 ID
    task_id = task.id or str(uuid.uuid4())[:8]
    now = datetime.now().isoformat()
    
    task_data = task.model_dump()
    task_data['id'] = task_id
    task_data['created_at'] = now
    task_data['updated_at'] = now
    
    # 保存到文件
    file_path = TASKS_DIR / f"{task_id}.json"
    with open(file_path, 'w', encoding='utf-8') as f:
        json.dump(task_data, f, ensure_ascii=False, indent=2)
    
    return TaskDefinition(**task_data)


@router.put("/task/{task_id}", response_model=TaskDefinition)
async def update_task(
    task_id: str,
    task: TaskDefinition,
    current_user=Depends(get_current_admin)
):
    """更新任务"""
    file_path = TASKS_DIR / f"{task_id}.json"
    
    if not file_path.exists():
        raise HTTPException(status_code=404, detail=f"Task '{task_id}' not found")
    
    # 读取原有数据
    with open(file_path, 'r', encoding='utf-8') as f:
        old_data = json.load(f)
    
    # 更新数据
    task_data = task.model_dump()
    task_data['id'] = task_id
    task_data['created_at'] = old_data.get('created_at')
    task_data['updated_at'] = datetime.now().isoformat()
    
    # 保存
    with open(file_path, 'w', encoding='utf-8') as f:
        json.dump(task_data, f, ensure_ascii=False, indent=2)
    
    return TaskDefinition(**task_data)


@router.delete("/task/{task_id}")
async def delete_task(
    task_id: str,
    current_user=Depends(get_current_admin)
):
    """删除任务"""
    file_path = TASKS_DIR / f"{task_id}.json"
    
    if not file_path.exists():
        raise HTTPException(status_code=404, detail=f"Task '{task_id}' not found")
    
    file_path.unlink()
    return {"success": True, "message": f"Task '{task_id}' deleted"}


# ============== 任务执行 API ==============

@router.post("/task/execute", response_model=TaskResponse)
async def execute_task(
    request: TaskExecuteRequest,
    current_user=Depends(get_current_admin)
):
    """
    执行任务
    
    接收任务 JSON，转发给 ROS2 Task Engine 执行
    """
    global _mock_execution_task
    watchdog.heartbeat()
    
    # 处理请求数据 - 支持两种格式
    if request.task_json:
        task_data = request.task_json
    elif request.root:
        task_data = {
            'id': request.id,
            'name': request.name or 'Unnamed Task',
            'root': request.root
        }
    else:
        return TaskResponse(success=False, message="Missing task data")
    
    task_json_str = json.dumps(task_data)
    
    if ros2_bridge.is_connected():
        result = await ros2_bridge.execute_task(task_json_str)
        if result:
            return TaskResponse(**result)
    
    # Mock 模式 - 模拟任务执行
    task_id = task_data.get('id', f'mock_task_{int(asyncio.get_event_loop().time())}')
    task_name = task_data.get('name', 'Mock Task')
    root_node = task_data.get('root', {})
    
    # 收集所有节点
    all_nodes = _collect_nodes(root_node)
    
    _mock_task_status.update({
        'task_id': task_id,
        'task_name': task_name,
        'status': 'running',
        'progress': 0.0,
        'elapsed_time': 0.0,
        'current_node_id': None,
        'completed_nodes': 0,
        'total_nodes': len(all_nodes),
        'node_statuses': all_nodes  # all_nodes 现在已经是完整的 node_status 结构
    })
    
    # 取消之前的模拟任务
    if _mock_execution_task and not _mock_execution_task.done():
        _mock_execution_task.cancel()
    
    # 启动模拟执行 - 使用递归执行支持并行
    async def _execute_wrapper():
        import time
        start_time = time.time()
        try:
            success = await _mock_execute_tree(root_node)
            
            # 任务完成时，将所有still running的节点标记为对应状态
            final_status = 'completed' if success else 'failed'
            for ns in _mock_task_status['node_statuses']:
                if ns['status'] == 'running':
                    # 如果任务成功完成，running节点也标记为success
                    # 如果任务失败，running节点标记为failure
                    ns['status'] = 'success' if success else 'failure'
            
            _mock_task_status['status'] = final_status
            _mock_task_status['elapsed_time'] = time.time() - start_time
            _mock_task_status['current_node_id'] = None
            
            # 最终广播完成状态
            await task_status_manager.broadcast({
                'type': 'task_status',
                'data': _get_status_snapshot()
            })
        except asyncio.CancelledError:
            _mock_task_status['status'] = 'cancelled'
            # 取消时也要清理running状态
            for ns in _mock_task_status['node_statuses']:
                if ns['status'] == 'running':
                    ns['status'] = 'idle'
            await task_status_manager.broadcast({
                'type': 'task_status',
                'data': _get_status_snapshot()
            })
        except Exception as e:
            _mock_task_status['status'] = 'failed'
            _mock_task_status['error'] = str(e)
            # 错误时也要清理running状态
            for ns in _mock_task_status['node_statuses']:
                if ns['status'] == 'running':
                    ns['status'] = 'failure'
            await task_status_manager.broadcast({
                'type': 'task_status',
                'data': _get_status_snapshot()
            })
    
    _mock_execution_task = asyncio.create_task(_execute_wrapper())
    
    return TaskResponse(
        success=True,
        task_id=task_id,
        message=f"Task '{task_name}' started (mock mode)"
    )


# 控制节点类型（可以有子节点）
CONTROL_NODE_TYPES = {'Sequence', 'Parallel', 'Selector', 'Loop'}


def _collect_nodes(node: Dict[str, Any], nodes: List[Dict] = None) -> List[Dict]:
    """递归收集所有节点，生成完整的 node_statuses 结构"""
    if nodes is None:
        nodes = []
    
    if not node:
        return nodes
    
    node_id = node.get('id', f'node_{len(nodes)}')
    node_type = node.get('type', 'Unknown')
    node_name = node.get('name', node_type)
    children = node.get('children', [])
    params = node.get('params', {})
    
    # 构建完整的节点状态对象
    node_status = {
        'node_id': node_id,
        'node_type': node_type,
        'node_name': node_name,
        'status': 'idle',
        'message': '',
        'duration': 0.0,
        # 扩展字段
        'children_count': len(children) if node_type in CONTROL_NODE_TYPES else 0,
        'current_child_index': 0,
        'current_iteration': 0,
        'total_iterations': params.get('iterations', 0) if node_type == 'Loop' else 0
    }
    nodes.append(node_status)
    
    # 只有控制节点才递归处理子节点
    if node_type in CONTROL_NODE_TYPES:
        for child in children:
            _collect_nodes(child, nodes)
    
    return nodes


async def _mock_execute_nodes(nodes: List[Dict]):
    """模拟执行节点（用于演示）"""
    import time
    start_time = time.time()
    
    try:
        for i, node in enumerate(nodes):
            if _mock_task_status['status'] == 'cancelled':
                break
            
            # 等待暂停结束
            while _mock_task_status['status'] == 'paused':
                await asyncio.sleep(0.1)
                if _mock_task_status['status'] == 'cancelled':
                    return
            
            # 更新当前节点状态
            _mock_task_status['current_node_id'] = node['id']
            _mock_task_status['elapsed_time'] = time.time() - start_time
            
            # 更新节点状态为 running
            for ns in _mock_task_status['node_statuses']:
                if ns['node_id'] == node['id']:
                    ns['status'] = 'running'
                    break
            
            # 广播状态更新
            await task_status_manager.broadcast({
                'type': 'task_status',
                'data': _get_status_snapshot()
            })
            
            # 模拟执行时间
            await asyncio.sleep(0.5 + (0.3 if 'Move' in node['type'] else 0.1))
            
            # 更新节点状态为 success
            for ns in _mock_task_status['node_statuses']:
                if ns['node_id'] == node['id']:
                    ns['status'] = 'success'
                    break
            
            _mock_task_status['completed_nodes'] = i + 1
            _mock_task_status['progress'] = (i + 1) / len(nodes)
        
        # 完成
        if _mock_task_status['status'] != 'cancelled':
            _mock_task_status['status'] = 'completed'
            _mock_task_status['elapsed_time'] = time.time() - start_time
        
        await task_status_manager.broadcast({
            'type': 'task_status',
            'data': _get_status_snapshot()
        })
        
    except asyncio.CancelledError:
        _mock_task_status['status'] = 'cancelled'
    except Exception as e:
        _mock_task_status['status'] = 'failed'
        _mock_task_status['error'] = str(e)


async def _mock_execute_tree(node: Dict[str, Any]) -> bool:
    """递归执行任务树，支持并行执行"""
    if not node:
        return True
    
    node_id = node.get('id')
    node_type = node.get('type')
    children = node.get('children', [])
    
    # 检查是否取消
    if _mock_task_status['status'] == 'cancelled':
        return False
    
    # 等待暂停结束
    while _mock_task_status['status'] == 'paused':
        await asyncio.sleep(0.1)
        if _mock_task_status['status'] == 'cancelled':
            return False
    
    # 更新节点状态为 running
    node_found = False
    for ns in _mock_task_status['node_statuses']:
        if ns['node_id'] == node_id:
            ns['status'] = 'running'
            node_found = True
            break
    
    if not node_found:
        print(f"⚠️  节点 {node_id} 未在 node_statuses 中找到")
    
    # 更新 current_node_id
    _mock_task_status['current_node_id'] = node_id
    
    # 广播状态更新 - 让前端看到 running 状态
    await task_status_manager.broadcast({
        'type': 'task_status',
        'data': _get_status_snapshot()
    })
    
    # 控制节点开始执行前有个小延迟，让用户看到转圈效果
    if node_type in CONTROL_NODE_TYPES:
        await asyncio.sleep(0.15)
    
    success = True
    
    # 根据节点类型执行
    if node_type == 'Parallel':
        # 并行执行所有子节点
        tasks = [_mock_execute_tree(child) for child in children]
        results = await asyncio.gather(*tasks, return_exceptions=True)
        success = all(r is True for r in results if not isinstance(r, Exception))
        
    elif node_type == 'Sequence':
        # 顺序执行子节点
        for idx, child in enumerate(children):
            # 更新当前子节点索引
            for ns in _mock_task_status['node_statuses']:
                if ns['node_id'] == node_id:
                    ns['current_child_index'] = idx
                    break
            
            result = await _mock_execute_tree(child)
            if not result:
                success = False
                break
                
    elif node_type == 'Selector':
        # 选择器：执行直到有一个成功
        success = False
        for idx, child in enumerate(children):
            # 更新当前子节点索引
            for ns in _mock_task_status['node_statuses']:
                if ns['node_id'] == node_id:
                    ns['current_child_index'] = idx
                    break
            
            result = await _mock_execute_tree(child)
            if result:
                success = True
                break
                
    elif node_type == 'Loop':
        # 循环：执行指定次数
        params = node.get('params', {})
        iterations = params.get('iterations', 1)
        
        # 收集所有子节点ID（用于在每次迭代前重置状态）
        def collect_child_ids(n):
            ids = []
            if n:
                ids.append(n.get('id'))
                for c in n.get('children', []):
                    ids.extend(collect_child_ids(c))
            return ids
        
        all_child_ids = set()
        for child in children:
            all_child_ids.update(collect_child_ids(child))
        
        # 循环执行子节点
        for i in range(iterations):
            # 更新当前迭代次数
            for ns in _mock_task_status['node_statuses']:
                if ns['node_id'] == node_id:
                    ns['current_iteration'] = i
                    break
            
            # 每次迭代开始前，重置子节点状态为 idle（除了第一次）
            if i > 0:
                for ns in _mock_task_status['node_statuses']:
                    if ns['node_id'] in all_child_ids:
                        ns['status'] = 'idle'
                # 广播重置状态
                await task_status_manager.broadcast({
                    'type': 'task_status',
                    'data': _get_status_snapshot()
                })
                await asyncio.sleep(0.1)  # 让前端看到重置
            
            for idx, child in enumerate(children):
                # 更新当前子节点索引
                for ns in _mock_task_status['node_statuses']:
                    if ns['node_id'] == node_id:
                        ns['current_child_index'] = idx
                        break
                
                result = await _mock_execute_tree(child)
                if not result:
                    success = False
                    break
            if not success:
                break
    else:
        # 叶子节点（技能节点）：模拟执行时间
        await asyncio.sleep(0.3 + (0.2 if 'Move' in node_type else 0.1))
    
    # 更新节点状态为完成状态
    for ns in _mock_task_status['node_statuses']:
        if ns['node_id'] == node_id:
            ns['status'] = 'success' if success else 'failure'
            break
    
    # 更新完成数和进度 - 计算实际完成的唯一节点数（而非累加）
    # 这样在Loop中同一节点执行多次也只计数一次
    completed_count = sum(1 for ns in _mock_task_status['node_statuses'] 
                         if ns['status'] in ['success', 'failure'])
    _mock_task_status['completed_nodes'] = completed_count
    _mock_task_status['progress'] = completed_count / max(_mock_task_status['total_nodes'], 1)
    
    # 🔍 调试日志
    status_icon = '✅' if success else '❌'
    print(f"{status_icon} [{node_type}] {node_id} -> {'SUCCESS' if success else 'FAILURE'}")
    print(f"📊 进度更新: {completed_count}/{_mock_task_status['total_nodes']} ({_mock_task_status['progress']*100:.1f}%)")
    
    # 广播完成状态 - 添加延迟让前端能看到success/failure状态
    await task_status_manager.broadcast({
        'type': 'task_status',
        'data': _get_status_snapshot()
    })
    
    # 给前端一点时间显示完成状态
    if node_type in CONTROL_NODE_TYPES:
        await asyncio.sleep(0.05)  # 控制节点快速完成
    else:
        await asyncio.sleep(0.15)  # 叶子节点稍长
    
    return success


@router.post("/task/pause", response_model=TaskResponse)
async def pause_task(
    task_id: Optional[str] = None,
    current_user=Depends(get_current_admin)
):
    """暂停任务"""
    watchdog.heartbeat()
    
    if ros2_bridge.is_connected():
        result = await ros2_bridge.pause_task(task_id or "")
        if result:
            return TaskResponse(**result)
    
    _mock_task_status['status'] = 'paused'
    return TaskResponse(success=True, message="Task paused (mock mode)")


@router.post("/task/resume", response_model=TaskResponse)
async def resume_task(
    task_id: Optional[str] = None,
    current_user=Depends(get_current_admin)
):
    """恢复任务"""
    watchdog.heartbeat()
    
    if ros2_bridge.is_connected():
        result = await ros2_bridge.resume_task(task_id or "")
        if result:
            return TaskResponse(**result)
    
    _mock_task_status['status'] = 'running'
    return TaskResponse(success=True, message="Task resumed (mock mode)")


@router.post("/task/cancel", response_model=TaskResponse)
async def cancel_task(
    task_id: Optional[str] = None,
    current_user=Depends(get_current_admin)
):
    """取消任务"""
    watchdog.heartbeat()
    
    if ros2_bridge.is_connected():
        result = await ros2_bridge.cancel_task(task_id or "")
        if result:
            return TaskResponse(**result)
    
    _mock_task_status['status'] = 'cancelled'
    return TaskResponse(success=True, message="Task cancelled (mock mode)")


@router.get("/task/status", response_model=TaskStatusResponse)
async def get_task_status_api(
    task_id: Optional[str] = None,
    current_user=Depends(get_current_admin)
):
    """获取任务状态"""
    watchdog.heartbeat()
    
    if ros2_bridge.is_connected():
        result = await ros2_bridge.get_task_status(task_id or "")
        if result and result.get('success'):
            return TaskStatusResponse(
                task_id=result.get('task_id'),
                status=result.get('status', 'idle'),
                progress=result.get('progress', 0.0),
                current_node_id=result.get('current_node'),
                elapsed_time=result.get('elapsed_time', 0.0)
            )
    
    return TaskStatusResponse(**_mock_task_status)


@router.get("/task/skills", response_model=List[SkillInfo])
async def get_available_skills(current_user=Depends(get_current_admin)):
    """
    获取所有可用的技能节点
    
    返回技能类型、描述和参数定义，用于前端构建节点面板
    """
    # 技能定义（从 qyh_task_engine 同步）
    skills = [
        {
            "type": "ArmMoveJ",
            "description": "关节空间运动 - 控制机械臂移动到指定关节角度",
            "params": {
                "side": {"type": "string", "required": True, "enum": ["left", "right", "both"], "description": "机械臂选择"},
                "joint_positions": {"type": "array", "required": False, "description": "目标关节角度 (弧度)"},
                "pose_name": {"type": "string", "required": False, "description": "预设姿态名称"},
                "velocity": {"type": "float", "default": 0.5, "description": "关节速度 (rad/s)"},
            }
        },
        {
            "type": "ArmMoveL",
            "description": "笛卡尔空间直线运动 - 控制机械臂沿直线移动到目标位置",
            "params": {
                "side": {"type": "string", "required": True, "enum": ["left", "right"], "description": "机械臂选择"},
                "x": {"type": "float", "required": True, "description": "目标 X 坐标 (米)"},
                "y": {"type": "float", "required": True, "description": "目标 Y 坐标 (米)"},
                "z": {"type": "float", "required": True, "description": "目标 Z 坐标 (米)"},
                "velocity": {"type": "float", "default": 100.0, "description": "速度 (mm/s)"},
            }
        },
        {
            "type": "GripperControl",
            "description": "夹爪控制 - 打开或关闭夹爪",
            "params": {
                "side": {"type": "string", "required": True, "enum": ["left", "right"], "description": "机械臂选择"},
                "action": {"type": "string", "required": True, "enum": ["open", "close"], "description": "动作"},
                "force": {"type": "float", "required": False, "description": "夹持力"},
            }
        },
        {
            "type": "HeadLookAt",
            "description": "头部转向 - 控制头部云台看向指定角度或预设点位",
            "params": {
                "point_id": {"type": "string", "required": False, "description": "预设点位ID"},
                "pan": {"type": "float", "required": False, "description": "水平角 (-1.0 到 1.0)"},
                "tilt": {"type": "float", "required": False, "description": "俯仰角 (-1.0 到 1.0)"},
            }
        },
        {
            "type": "HeadScan",
            "description": "头部扫描 - 执行预定义的头部扫描动作",
            "params": {
                "pattern": {"type": "string", "default": "left_right", "enum": ["left_right", "up_down", "circle"], "description": "扫描模式"},
                "repeat": {"type": "int", "default": 1, "description": "重复次数"},
            }
        },
        {
            "type": "BaseMoveTo",
            "description": "底盘导航 - 移动到指定坐标或预设点位",
            "params": {
                "x": {"type": "float", "required": False, "description": "目标 X 坐标 (米)"},
                "y": {"type": "float", "required": False, "description": "目标 Y 坐标 (米)"},
                "theta": {"type": "float", "default": 0.0, "description": "目标朝向 (弧度)"},
                "location": {"type": "string", "required": False, "description": "预设点位名称"},
                "timeout": {"type": "float", "default": 60.0, "description": "超时时间 (秒)"},
            }
        },
        {
            "type": "Wait",
            "description": "等待 - 暂停指定时间",
            "params": {
                "duration": {"type": "float", "required": True, "description": "等待时间 (秒)"},
            }
        },
        {
            "type": "CheckCondition",
            "description": "条件检查 - 检查黑板变量是否满足条件",
            "params": {
                "condition": {"type": "string", "required": True, "description": "条件表达式 (如: arm.left.is_idle == True)"},
            }
        },
    ]
    
    return [SkillInfo(**s) for s in skills]


@router.post("/task/validate", response_model=TaskResponse)
async def validate_task(
    request: TaskExecuteRequest,
    current_user=Depends(get_current_admin)
):
    """
    验证任务 JSON 的有效性
    
    不执行任务，仅检查 JSON 格式和节点类型是否正确
    """
    # 基本验证
    task_data = request.task_json
    
    if 'root' not in task_data:
        return TaskResponse(success=False, message="Missing 'root' field")
    
    # 递归验证节点
    errors = _validate_node(task_data['root'], 'root')
    if errors:
        return TaskResponse(success=False, message='; '.join(errors))
    
    return TaskResponse(success=True, message="Task JSON is valid")


def _validate_node(node: dict, path: str) -> List[str]:
    """递归验证节点"""
    errors = []
    
    if not isinstance(node, dict):
        return [f"{path}: node must be an object"]
    
    node_type = node.get('type')
    if not node_type:
        return [f"{path}: missing 'type'"]
    
    # 已知节点类型
    known_types = {
        'Sequence', 'Parallel', 'Selector', 'Loop',
        'ArmMoveJ', 'ArmMoveL', 'ArmStop',
        'GripperControl',
        'HeadLookAt', 'HeadScan',
        'BaseMoveTo', 'BaseVelocity',
        'LiftMoveTo', 'LiftStop',
        'Wait', 'CheckCondition', 'SubTask'
    }
    
    if node_type not in known_types:
        errors.append(f"{path}: unknown node type '{node_type}'")
    
    # 验证复合节点的子节点
    if node_type in ['Sequence', 'Parallel', 'Selector', 'Loop']:
        children = node.get('children', [])
        if not isinstance(children, list):
            errors.append(f"{path}: 'children' must be an array")
        else:
            for i, child in enumerate(children):
                child_errors = _validate_node(child, f"{path}.children[{i}]")
                errors.extend(child_errors)
    
    return errors


# ============== WebSocket 状态推送 ==============

class TaskStatusManager:
    """任务状态管理器 - 管理 WebSocket 连接和状态广播"""
    
    def __init__(self):
        self.active_connections: List[WebSocket] = []
        self._broadcast_task: Optional[asyncio.Task] = None
    
    async def connect(self, websocket: WebSocket):
        await websocket.accept()
        self.active_connections.append(websocket)
        
        # 启动广播任务
        if self._broadcast_task is None or self._broadcast_task.done():
            self._broadcast_task = asyncio.create_task(self._broadcast_loop())
    
    def disconnect(self, websocket: WebSocket):
        if websocket in self.active_connections:
            self.active_connections.remove(websocket)
    
    async def broadcast(self, message: dict):
        """广播消息给所有连接"""
        disconnected = []
        for connection in self.active_connections:
            try:
                await connection.send_json(message)
            except:
                disconnected.append(connection)
        
        for conn in disconnected:
            self.disconnect(conn)
    
    async def _broadcast_loop(self):
        """状态广播循环"""
        while self.active_connections:
            try:
                # 获取当前状态
                if ros2_bridge.is_connected():
                    # 优先使用缓存的订阅状态
                    cached_status = ros2_bridge.get_cached_task_status()
                    if cached_status:
                        await self.broadcast({
                            'type': 'task_status',
                            'data': cached_status
                        })
                    else:
                        # Mock模式：只在有任务时广播
                        if _mock_task_status['status'] != 'idle' or _mock_task_status['total_nodes'] > 0:
                            await self.broadcast({
                                'type': 'task_status',
                                'data': _get_status_snapshot()
                            })
                else:
                    # 只在有任务执行时才广播Mock状态
                    if _mock_task_status['status'] != 'idle' or _mock_task_status['total_nodes'] > 0:
                        await self.broadcast({
                            'type': 'task_status',
                            'data': _get_status_snapshot()
                        })
                
                await asyncio.sleep(0.2)  # 5Hz
            except Exception as e:
                print(f"Broadcast error: {e}")
                await asyncio.sleep(1.0)


task_status_manager = TaskStatusManager()


@router.websocket("/task/ws")
async def task_websocket(websocket: WebSocket):
    """任务状态 WebSocket 端点"""
    await task_status_manager.connect(websocket)
    try:
        while True:
            # 接收客户端消息（心跳等）
            data = await websocket.receive_text()
            if data == 'ping':
                await websocket.send_text('pong')
    except WebSocketDisconnect:
        task_status_manager.disconnect(websocket)
