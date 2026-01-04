"""WebSocket API"""
from fastapi import APIRouter, WebSocket, WebSocketDisconnect, Query
from typing import List
import asyncio
from app.core.security import decode_access_token
from app.ros2_bridge.bridge import ros2_bridge
from app.ros2_bridge.joint_state_bridge import get_bridge
from app.safety.watchdog import watchdog

router = APIRouter()


class ConnectionManager:
    """WebSocket 连接管理器"""
    
    def __init__(self):
        self.active_connections: List[WebSocket] = []
    
    async def connect(self, websocket: WebSocket):
        await websocket.accept()
        self.active_connections.append(websocket)
    
    def disconnect(self, websocket: WebSocket):
        if websocket in self.active_connections:
            self.active_connections.remove(websocket)
    
    async def broadcast(self, message: dict):
        for connection in self.active_connections[:]:  # 复制列表避免迭代时修改
            try:
                await connection.send_json(message)
            except Exception:
                self.disconnect(connection)


manager = ConnectionManager()


@router.websocket("")
async def websocket_endpoint(
    websocket: WebSocket,
    token: str = Query(...)
):
    """WebSocket 端点 - 用于机器人状态和控制"""
    # 验证 Token
    try:
        payload = decode_access_token(token)
        user_id = int(payload.get("sub"))
        username = payload.get("username")
    except Exception:
        await websocket.close(code=1008)
        return
    
    await manager.connect(websocket)
    print(f"✅ WebSocket 连接: {username} (ID: {user_id})")
    
    try:
        # 启动状态推送任务
        push_task = asyncio.create_task(push_robot_state(websocket))
        
        # 接收客户端消息
        while True:
            data = await websocket.receive_json()
            await handle_message(data, user_id)
    
    except WebSocketDisconnect:
        manager.disconnect(websocket)
        push_task.cancel()
        print(f"🔌 WebSocket 断开: {username} (ID: {user_id})")
    
    except Exception as e:
        print(f"❌ WebSocket 错误: {e}")
        manager.disconnect(websocket)


@router.websocket("/robot")
async def websocket_robot_endpoint(websocket: WebSocket):
    """WebSocket 端点 - 用于 3D 可视化（无需认证，用于演示）"""
    await websocket.accept()
    print("✅ 3D Viewer WebSocket 连接")
    
    # 获取 Joint State Bridge
    try:
        bridge = get_bridge()
        bridge.add_client(websocket)
    except Exception as e:
        print(f"❌ 无法连接到 ROS2 Joint State Bridge: {e}")
        await websocket.send_json({
            'type': 'error',
            'message': 'ROS2 未连接，使用演示模式'
        })
    
    try:
        # 保持连接并接收客户端消息
        while True:
            data = await websocket.receive_json()
            
            # 处理客户端请求（如视角切换、录制等）
            msg_type = data.get('op')
            
            if msg_type == 'subscribe':
                # 客户端请求订阅话题
                topic = data.get('topic')
                print(f"📡 客户端订阅话题: {topic}")
            
            elif msg_type == 'heartbeat':
                # 心跳响应
                await websocket.send_json({'type': 'pong'})
    
    except WebSocketDisconnect:
        print("🔌 3D Viewer WebSocket 断开")
        if 'bridge' in locals():
            bridge.remove_client(websocket)
    
    except Exception as e:
        print(f"❌ 3D Viewer WebSocket 错误: {e}")
        if 'bridge' in locals():
            bridge.remove_client(websocket)


async def push_robot_state(websocket: WebSocket):
    """推送机器人状态（30Hz）"""
    while True:
        try:
            state = ros2_bridge.get_robot_state()
            if state:
                await websocket.send_json({
                    'type': 'robot_state',
                    'data': state
                })
            await asyncio.sleep(1/30)  # 30Hz
        except Exception:
            break


async def handle_message(data: dict, user_id: int):
    """处理客户端消息"""
    msg_type = data.get('type')
    
    if msg_type == 'heartbeat':
        # 接收心跳
        watchdog.heartbeat()
    
    elif msg_type == 'subscribe':
        # TODO: 处理订阅请求
        topics = data.get('topics', [])
        print(f"📡 订阅话题: {topics}")
    
    elif msg_type == 'velocity_command':
        # TODO: 验证控制权后发送速度命令
        ros2_bridge.send_command(data)
