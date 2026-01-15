# QYH Jushen Robot API 接口文档

> **版本**: 1.0.0  
> **更新日期**: 2026-01-15  
> **基础URL**: `http://<robot_ip>:8000`

---

## 目录

1. [概述](#1-概述)
2. [认证机制](#2-认证机制)
3. [REST API 接口](#3-rest-api-接口)
4. [WebSocket 接口](#4-websocket-接口)
5. [错误码说明](#5-错误码说明)
6. [行业最佳实践](#6-行业最佳实践)
7. [SDK 集成示例](#7-sdk-集成示例)

---

## 1. 概述

### 1.1 架构设计

```
┌─────────────────────────────────────────────────────────────┐
│                     外部系统 / 客户端                         │
│  (Web前端、移动App、MES系统、第三方集成)                       │
└─────────────────────────────┬───────────────────────────────┘
                              │
                    ┌─────────▼─────────┐
                    │   HTTP/WebSocket   │
                    │   (对外网关层)      │
                    └─────────┬─────────┘
                              │
┌─────────────────────────────▼───────────────────────────────┐
│                    QYH Jushen Backend                        │
│  ┌──────────────┬──────────────┬──────────────────────────┐ │
│  │  REST API    │  WebSocket   │  安全/控制权管理           │ │
│  │  (控制/配置)  │  (实时推送)   │  (认证、急停、看门狗)      │ │
│  └──────┬───────┴──────┬───────┴─────────────┬────────────┘ │
│         │              │                     │              │
│         └──────────────┼─────────────────────┘              │
│                        │                                    │
│              ┌─────────▼─────────┐                          │
│              │   ROS2 Bridge     │                          │
│              │   (内部通信)       │                          │
│              └─────────┬─────────┘                          │
└────────────────────────┼────────────────────────────────────┘
                         │
           ┌─────────────▼─────────────┐
           │   ROS2 / 硬件驱动层        │
           │   (底盘、机械臂、传感器)    │
           └───────────────────────────┘
```

### 1.2 接口协议

| 协议 | 用途 | 特点 |
|------|------|------|
| **REST/HTTP** | 控制命令、配置、任务管理 | 请求-响应模式，适合低频操作 |
| **WebSocket** | 实时状态推送、遥操作 | 双向通信，30Hz 状态推送 |

### 1.3 API 文档

本后端集成了 **Swagger UI** 自动文档：
- Swagger UI: `http://<robot_ip>:8000/docs`
- ReDoc: `http://<robot_ip>:8000/redoc`
- OpenAPI JSON: `http://<robot_ip>:8000/openapi.json`

---

## 2. 认证机制

### 2.1 JWT Token 认证

所有 API（除登录外）都需要 JWT Token 认证。

#### 登录获取 Token

```http
POST /api/auth/login
Content-Type: application/json

{
  "username": "admin",
  "password": "admin123"
}
```

**响应:**
```json
{
  "access_token": "eyJhbGciOiJIUzI1NiIs...",
  "token_type": "bearer",
  "expires_in": 1800,
  "user": {
    "id": 1,
    "username": "admin",
    "role": "admin",
    "email": "admin@example.com"
  }
}
```

#### 使用 Token

```http
GET /api/robot/status
Authorization: Bearer eyJhbGciOiJIUzI1NiIs...
```

#### 刷新 Token

```http
POST /api/auth/refresh
Authorization: Bearer <current_token>
```

### 2.2 角色权限

| 角色 | 权限说明 |
|------|----------|
| `admin` | 完整权限，包括系统配置、用户管理 |
| `operator` | 控制机器人、执行任务 |
| `viewer` | 只读访问，查看状态 |

---

## 3. REST API 接口

### 3.1 系统状态

#### 健康检查

```http
GET /health
```

**响应:**
```json
{
  "status": "healthy",
  "ros2_connected": true,
  "database": "ok"
}
```

#### 获取机器人状态

```http
GET /api/robot/status
Authorization: Bearer <token>
```

**响应:**
```json
{
  "timestamp": "2026-01-15T10:30:00Z",
  "joints": {
    "left_arm": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    "right_arm": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
  },
  "grippers": {
    "left": {"position": 0.05, "force": 10.0},
    "right": {"position": 0.08, "force": 12.0}
  },
  "base": {
    "x": 0.0, "y": 0.0, "theta": 0.0,
    "velocity": {"linear": 0.0, "angular": 0.0}
  },
  "system": {
    "cpu_temp": 65.0,
    "battery": 85.0,
    "mode": "idle"
  }
}
```

---

### 3.2 控制权管理

> **重要**: 执行运动命令前必须先获取控制权

#### 获取控制权

```http
POST /api/control/acquire
Authorization: Bearer <token>
Content-Type: application/json

{
  "duration": 300
}
```

**响应:**
```json
{
  "success": true,
  "holder": {
    "user_id": 1,
    "username": "admin",
    "acquired_at": "2026-01-15T10:30:00Z",
    "expires_at": "2026-01-15T10:35:00Z"
  }
}
```

#### 释放控制权

```http
POST /api/control/release
Authorization: Bearer <token>
```

#### 查询控制权状态

```http
GET /api/control/status
```

---

### 3.3 机械臂控制

#### 获取机械臂状态

```http
GET /api/v1/arm/state
Authorization: Bearer <token>
```

**响应:**
```json
{
  "connected": true,
  "powered_on": true,
  "enabled": true,
  "in_estop": false,
  "in_error": false,
  "servo_mode_enabled": false,
  "left_joint_positions": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
  "right_joint_positions": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
  "left_cartesian_pose": {"x": 0.3, "y": 0.0, "z": 0.5, "rx": 0, "ry": 0, "rz": 0},
  "right_cartesian_pose": {"x": 0.3, "y": 0.0, "z": 0.5, "rx": 0, "ry": 0, "rz": 0}
}
```

#### 机械臂使能

```http
POST /api/v1/arm/enable
Authorization: Bearer <token>
Content-Type: application/json

{
  "robot_id": -1
}
```

| robot_id | 说明 |
|----------|------|
| -1 | 双臂 |
| 0 | 左臂 |
| 1 | 右臂 |

#### 关节运动 (MoveJ)

```http
POST /api/v1/arm/movej
Authorization: Bearer <token>
Content-Type: application/json

{
  "robot_id": -1,
  "joint_positions": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
  "velocity": 0.5,
  "acceleration": 0.3,
  "is_block": true
}
```

#### 直线运动 (MoveL)

```http
POST /api/v1/arm/movel
Authorization: Bearer <token>
Content-Type: application/json

{
  "robot_id": 0,
  "x": 0.3,
  "y": 0.0,
  "z": 0.5,
  "rx": 0.0,
  "ry": 0.0,
  "rz": 0.0,
  "velocity": 100.0,
  "acceleration": 50.0,
  "is_block": true
}
```

#### 点动控制 (Jog)

```http
POST /api/v1/arm/jog
Authorization: Bearer <token>
Content-Type: application/json

{
  "robot_id": 0,
  "axis_num": 1,
  "move_mode": 1,
  "coord_type": 1,
  "velocity": 0.1,
  "position": 0.01
}
```

| move_mode | 说明 |
|-----------|------|
| 0 | 绝对 |
| 1 | 步进 (INCR) |
| 2 | 连续 (CONTINUE) |

---

### 3.4 夹爪控制

#### 获取夹爪状态

```http
GET /api/v1/gripper/state
Authorization: Bearer <token>
```

**响应:**
```json
{
  "left": {
    "is_activated": true,
    "is_moving": false,
    "object_status": 2,
    "object_status_text": "外夹抓到",
    "current_position": 150,
    "current_force": 50
  },
  "right": {
    "is_activated": true,
    "is_moving": false,
    "object_status": 3,
    "object_status_text": "到达位置",
    "current_position": 0,
    "current_force": 0
  }
}
```

#### 控制夹爪

```http
POST /api/v1/gripper/move
Authorization: Bearer <token>
Content-Type: application/json

{
  "side": "left",
  "position": 255,
  "speed": 255,
  "force": 150
}
```

| 参数 | 范围 | 说明 |
|------|------|------|
| position | 0-255 | 0=全开, 255=全闭 |
| speed | 0-255 | 运动速度 |
| force | 0-255 | 夹持力 |

---

### 3.5 底盘控制

#### 获取底盘状态

```http
GET /api/v1/chassis/status
Authorization: Bearer <token>
```

**响应:**
```json
{
  "connected": true,
  "system_status": 2,
  "system_status_text": "系统空闲",
  "location_status": 3,
  "location_status_text": "定位成功",
  "current_map": "warehouse_floor1",
  "current_site": 5,
  "battery_level": 85,
  "position": {"x": 1.2, "y": 3.4, "yaw": 0.5},
  "velocity": {"linear": 0.0, "angular": 0.0}
}
```

#### 导航到站点

```http
POST /api/v1/chassis/navigate-to-site
Authorization: Bearer <token>
Content-Type: application/json

{
  "site_id": 5
}
```

#### 速度控制

```http
POST /api/v1/chassis/velocity
Authorization: Bearer <token>
Content-Type: application/json

{
  "linear_x": 0.5,
  "linear_y": 0.0,
  "angular_z": 0.1
}
```

#### 取消导航

```http
POST /api/v1/chassis/cancel
Authorization: Bearer <token>
```

---

### 3.6 升降机构

#### 获取升降状态

```http
GET /api/v1/lift/state
Authorization: Bearer <token>
```

#### 升降控制

```http
POST /api/v1/lift/control
Authorization: Bearer <token>
Content-Type: application/json

{
  "command": 4,
  "value": 500.0
}
```

| command | 说明 |
|---------|------|
| 1 | 使能 |
| 2 | 去使能 |
| 3 | 设置速度 |
| 4 | 去目标位置 |
| 5 | 手动上升 |
| 6 | 手动下降 |
| 7 | 复位报警 |
| 8 | 停止运动 |

---

### 3.7 头部控制

#### 获取头部状态

```http
GET /api/v1/head/state
Authorization: Bearer <token>
```

#### 头部控制

```http
POST /api/v1/head/control
Authorization: Bearer <token>
Content-Type: application/json

{
  "pan": 0.5,
  "tilt": -0.3,
  "speed": 50.0
}
```

| 参数 | 范围 | 说明 |
|------|------|------|
| pan | -1.0 ~ 1.0 | 左右转动 |
| tilt | -1.0 ~ 1.0 | 上下俯仰 |
| speed | 0 ~ 100 | 运动速度% |

---

### 3.8 任务管理

#### 创建任务

```http
POST /api/tasks
Authorization: Bearer <token>
Content-Type: application/json

{
  "name": "搬运任务",
  "description": "从A点搬运到B点",
  "program": [
    {"action": "navigate", "params": {"site_id": 1}},
    {"action": "arm_move", "params": {"preset": "pick_pose"}},
    {"action": "gripper", "params": {"side": "left", "position": 255}},
    {"action": "navigate", "params": {"site_id": 2}},
    {"action": "gripper", "params": {"side": "left", "position": 0}}
  ]
}
```

#### 列出任务

```http
GET /api/tasks?status=pending
Authorization: Bearer <token>
```

#### 启动任务

```http
POST /api/tasks/{task_id}/start
Authorization: Bearer <token>
```

#### 取消任务

```http
POST /api/tasks/{task_id}/cancel
Authorization: Bearer <token>
```

---

### 3.9 紧急停止

```http
POST /api/emergency/stop
Authorization: Bearer <token>
```

**响应:**
```json
{
  "success": true,
  "message": "所有关节已停止"
}
```

> ⚠️ 紧急停止会立即停止所有运动，并强制释放控制权

---

### 3.10 动作管理 (AI 模型)

#### 列出所有动作

```http
GET /api/v1/actions
Authorization: Bearer <token>
```

#### 获取动作详情

```http
GET /api/v1/actions/{action_id}
Authorization: Bearer <token>
```

#### 创建动作

```http
POST /api/v1/actions
Authorization: Bearer <token>
Content-Type: application/json

{
  "id": "pickup_cube",
  "name": "夹取方块",
  "description": "从桌面夹取方块"
}
```

---

### 3.11 数据录制

#### 开始录制

```http
POST /api/v1/recording/start
Authorization: Bearer <token>
Content-Type: application/json

{
  "action_name": "pickup_cube",
  "user_name": "operator1",
  "version": "1.0",
  "topics": [
    "/joint_states",
    "/camera/image_raw"
  ]
}
```

#### 停止录制

```http
POST /api/v1/recording/stop
Authorization: Bearer <token>
```

#### 录制状态

```http
GET /api/v1/recording/status
Authorization: Bearer <token>
```

---

## 4. WebSocket 接口

### 4.1 主 WebSocket 端点

**连接地址:** `ws://<robot_ip>:8000/ws?token=<jwt_token>`

#### 连接示例 (JavaScript)

```javascript
const token = "eyJhbGciOiJIUzI1NiIs...";
const ws = new WebSocket(`ws://192.168.1.100:8000/ws?token=${token}`);

ws.onopen = () => {
  console.log("WebSocket 已连接");
  // 发送心跳
  setInterval(() => {
    ws.send(JSON.stringify({ type: "heartbeat" }));
  }, 5000);
};

ws.onmessage = (event) => {
  const message = JSON.parse(event.data);
  
  if (message.type === "robot_state") {
    // 处理机器人状态 (30Hz)
    console.log("机器人状态:", message.data);
  }
};

ws.onclose = () => {
  console.log("WebSocket 已断开");
};
```

#### 连接示例 (Python)

```python
import asyncio
import websockets
import json

async def connect():
    token = "eyJhbGciOiJIUzI1NiIs..."
    uri = f"ws://192.168.1.100:8000/ws?token={token}"
    
    async with websockets.connect(uri) as ws:
        # 订阅话题
        await ws.send(json.dumps({
            "type": "subscribe",
            "topics": ["/joint_states", "/gripper_state"]
        }))
        
        # 接收消息
        while True:
            message = await ws.recv()
            data = json.loads(message)
            
            if data["type"] == "robot_state":
                print(f"状态: {data['data']}")

asyncio.run(connect())
```

### 4.2 服务器推送消息类型

| type | 说明 | 频率 |
|------|------|------|
| `robot_state` | 机器人完整状态 | 30Hz |
| `error` | 错误通知 | 事件触发 |
| `task_update` | 任务状态更新 | 事件触发 |
| `control_change` | 控制权变更通知 | 事件触发 |

#### robot_state 消息格式

```json
{
  "type": "robot_state",
  "data": {
    "timestamp": "2026-01-15T10:30:00.123Z",
    "joints": {
      "left_arm": [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7],
      "right_arm": [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7]
    },
    "grippers": {
      "left": {"position": 100, "force": 50},
      "right": {"position": 0, "force": 0}
    },
    "base": {
      "x": 1.2, "y": 3.4, "theta": 0.5,
      "velocity": {"linear": 0.3, "angular": 0.1}
    }
  }
}
```

### 4.3 客户端发送消息类型

| type | 说明 |
|------|------|
| `heartbeat` | 心跳保活 |
| `subscribe` | 订阅话题 |
| `velocity_command` | 速度指令 (遥操作) |

#### 发送速度指令

```json
{
  "type": "velocity_command",
  "data": {
    "linear_x": 0.5,
    "linear_y": 0.0,
    "angular_z": 0.1
  }
}
```

### 4.4 3D 可视化 WebSocket

**连接地址:** `ws://<robot_ip>:8000/ws/robot` (无需认证，用于演示)

用于 3D 模型可视化的关节状态推送：

```json
{
  "type": "joint_states",
  "data": {
    "name": ["joint1", "joint2", "joint3", ...],
    "position": [0.1, 0.2, 0.3, ...],
    "velocity": [0.0, 0.0, 0.0, ...],
    "effort": [0.0, 0.0, 0.0, ...]
  }
}
```

---

## 5. 错误码说明

### 5.1 HTTP 状态码

| 状态码 | 说明 |
|--------|------|
| 200 | 成功 |
| 201 | 创建成功 |
| 400 | 请求参数错误 |
| 401 | 未认证 / Token 无效 |
| 403 | 权限不足 |
| 404 | 资源不存在 |
| 409 | 冲突 (如控制权已被占用) |
| 500 | 服务器内部错误 |
| 503 | 服务不可用 (如 ROS2 未连接) |

### 5.2 错误响应格式

```json
{
  "detail": "控制权已被用户 admin 持有"
}
```

---

## 6. 行业最佳实践

### 6.1 接口设计原则

| 原则 | 说明 |
|------|------|
| REST + WebSocket | REST 用于控制/配置，WebSocket 用于实时状态 |
| 控制权锁 | 防止多客户端冲突控制机器人 |
| 心跳机制 | 检测连接存活，自动释放控制权 |
| 看门狗 | 客户端断连后自动停止机器人 |

### 6.2 与主流厂商对比

| 厂商 | 接口方案 |
|------|----------|
| **Universal Robots (UR)** | REST API + RTDE (实时数据) |
| **ABB** | Robot Web Services (REST) + WebSocket |
| **FANUC** | FANUC Web Server (REST) |
| **Boston Dynamics** | REST API + gRPC |
| **QYH (本系统)** | REST API + WebSocket ✅ |

### 6.3 集成建议

```
外部系统集成建议：

1. MES/WMS 系统集成
   - 使用 REST API 下发任务
   - WebSocket 监听任务状态变化

2. 视觉/AI 系统集成
   - REST API 获取相机视频流
   - REST API 触发动作执行

3. 监控系统集成
   - WebSocket 实时获取状态
   - REST API 获取历史数据

4. 移动端 App
   - REST API 登录认证
   - WebSocket 实时遥操作
```

---

## 7. SDK 集成示例

### 7.1 Python SDK 示例

```python
import requests
import websockets
import asyncio
import json

class QYHRobotClient:
    """QYH 机器人 Python SDK"""
    
    def __init__(self, host: str, port: int = 8000):
        self.base_url = f"http://{host}:{port}"
        self.ws_url = f"ws://{host}:{port}/ws"
        self.token = None
    
    def login(self, username: str, password: str) -> bool:
        """登录获取 Token"""
        resp = requests.post(
            f"{self.base_url}/api/auth/login",
            json={"username": username, "password": password}
        )
        if resp.status_code == 200:
            self.token = resp.json()["access_token"]
            return True
        return False
    
    @property
    def headers(self):
        return {"Authorization": f"Bearer {self.token}"}
    
    def get_status(self) -> dict:
        """获取机器人状态"""
        resp = requests.get(
            f"{self.base_url}/api/robot/status",
            headers=self.headers
        )
        return resp.json()
    
    def acquire_control(self, duration: int = 300) -> bool:
        """获取控制权"""
        resp = requests.post(
            f"{self.base_url}/api/control/acquire",
            headers=self.headers,
            json={"duration": duration}
        )
        return resp.status_code == 200
    
    def release_control(self) -> bool:
        """释放控制权"""
        resp = requests.post(
            f"{self.base_url}/api/control/release",
            headers=self.headers
        )
        return resp.status_code == 200
    
    def emergency_stop(self) -> bool:
        """紧急停止"""
        resp = requests.post(
            f"{self.base_url}/api/emergency/stop",
            headers=self.headers
        )
        return resp.status_code == 200
    
    def move_arm_joint(self, positions: list, velocity: float = 0.5) -> dict:
        """关节运动"""
        resp = requests.post(
            f"{self.base_url}/api/v1/arm/movej",
            headers=self.headers,
            json={
                "robot_id": -1,
                "joint_positions": positions,
                "velocity": velocity,
                "is_block": True
            }
        )
        return resp.json()
    
    def control_gripper(self, side: str, position: int) -> dict:
        """控制夹爪"""
        resp = requests.post(
            f"{self.base_url}/api/v1/gripper/move",
            headers=self.headers,
            json={"side": side, "position": position}
        )
        return resp.json()
    
    async def subscribe_state(self, callback):
        """订阅实时状态"""
        uri = f"{self.ws_url}?token={self.token}"
        async with websockets.connect(uri) as ws:
            while True:
                msg = await ws.recv()
                data = json.loads(msg)
                if data["type"] == "robot_state":
                    callback(data["data"])


# 使用示例
if __name__ == "__main__":
    robot = QYHRobotClient("192.168.1.100")
    
    # 登录
    robot.login("admin", "admin123")
    
    # 获取状态
    status = robot.get_status()
    print(f"电池: {status['system']['battery']}%")
    
    # 获取控制权
    if robot.acquire_control():
        # 控制夹爪
        robot.control_gripper("left", 255)  # 关闭
        robot.control_gripper("left", 0)    # 打开
        
        # 释放控制权
        robot.release_control()
```

### 7.2 JavaScript/TypeScript SDK 示例

```typescript
class QYHRobotClient {
  private baseUrl: string;
  private wsUrl: string;
  private token: string | null = null;
  private ws: WebSocket | null = null;

  constructor(host: string, port: number = 8000) {
    this.baseUrl = `http://${host}:${port}`;
    this.wsUrl = `ws://${host}:${port}/ws`;
  }

  async login(username: string, password: string): Promise<boolean> {
    const resp = await fetch(`${this.baseUrl}/api/auth/login`, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ username, password })
    });
    
    if (resp.ok) {
      const data = await resp.json();
      this.token = data.access_token;
      return true;
    }
    return false;
  }

  private get headers() {
    return {
      'Authorization': `Bearer ${this.token}`,
      'Content-Type': 'application/json'
    };
  }

  async getStatus(): Promise<any> {
    const resp = await fetch(`${this.baseUrl}/api/robot/status`, {
      headers: this.headers
    });
    return resp.json();
  }

  async acquireControl(duration: number = 300): Promise<boolean> {
    const resp = await fetch(`${this.baseUrl}/api/control/acquire`, {
      method: 'POST',
      headers: this.headers,
      body: JSON.stringify({ duration })
    });
    return resp.ok;
  }

  async emergencyStop(): Promise<boolean> {
    const resp = await fetch(`${this.baseUrl}/api/emergency/stop`, {
      method: 'POST',
      headers: this.headers
    });
    return resp.ok;
  }

  subscribeState(callback: (state: any) => void): void {
    this.ws = new WebSocket(`${this.wsUrl}?token=${this.token}`);
    
    this.ws.onmessage = (event) => {
      const data = JSON.parse(event.data);
      if (data.type === 'robot_state') {
        callback(data.data);
      }
    };

    // 心跳保活
    setInterval(() => {
      if (this.ws?.readyState === WebSocket.OPEN) {
        this.ws.send(JSON.stringify({ type: 'heartbeat' }));
      }
    }, 5000);
  }

  disconnect(): void {
    this.ws?.close();
  }
}

// 使用示例
const robot = new QYHRobotClient('192.168.1.100');

(async () => {
  await robot.login('admin', 'admin123');
  
  const status = await robot.getStatus();
  console.log(`电池: ${status.system.battery}%`);
  
  // 订阅实时状态
  robot.subscribeState((state) => {
    console.log('实时状态:', state);
  });
})();
```

---

## 附录

### A. 完整接口列表

| 方法 | 路径 | 说明 |
|------|------|------|
| POST | `/api/auth/login` | 登录 |
| POST | `/api/auth/logout` | 登出 |
| POST | `/api/auth/refresh` | 刷新 Token |
| GET | `/api/robot/status` | 机器人状态 |
| POST | `/api/control/acquire` | 获取控制权 |
| POST | `/api/control/release` | 释放控制权 |
| GET | `/api/control/status` | 控制权状态 |
| POST | `/api/emergency/stop` | 紧急停止 |
| GET | `/api/v1/arm/state` | 机械臂状态 |
| POST | `/api/v1/arm/enable` | 机械臂使能 |
| POST | `/api/v1/arm/disable` | 机械臂去使能 |
| POST | `/api/v1/arm/movej` | 关节运动 |
| POST | `/api/v1/arm/movel` | 直线运动 |
| POST | `/api/v1/arm/jog` | 点动控制 |
| GET | `/api/v1/gripper/state` | 夹爪状态 |
| POST | `/api/v1/gripper/activate` | 激活夹爪 |
| POST | `/api/v1/gripper/move` | 控制夹爪 |
| GET | `/api/v1/chassis/status` | 底盘状态 |
| POST | `/api/v1/chassis/velocity` | 速度控制 |
| POST | `/api/v1/chassis/navigate-to-site` | 导航到站点 |
| GET | `/api/v1/lift/state` | 升降状态 |
| POST | `/api/v1/lift/control` | 升降控制 |
| GET | `/api/v1/head/state` | 头部状态 |
| POST | `/api/v1/head/control` | 头部控制 |
| GET | `/api/tasks` | 任务列表 |
| POST | `/api/tasks` | 创建任务 |
| POST | `/api/tasks/{id}/start` | 启动任务 |
| POST | `/api/tasks/{id}/cancel` | 取消任务 |
| GET | `/api/v1/actions` | 动作列表 |
| POST | `/api/v1/actions` | 创建动作 |
| POST | `/api/v1/recording/start` | 开始录制 |
| POST | `/api/v1/recording/stop` | 停止录制 |
| WS | `/ws?token=<token>` | 主 WebSocket |
| WS | `/ws/robot` | 3D 可视化 WebSocket |

### B. 联系方式

- **技术支持**: support@qyh-robot.com
- **API 问题**: api@qyh-robot.com
