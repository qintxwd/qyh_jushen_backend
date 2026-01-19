# Control Plane 待实现功能清单

> 基于 `重构.md`、`重构开发计划.md`、`重构补充.md` 以及 `old_backup` 旧代码分析，整理 control_plane 中需要实现的功能。
>
> ⚠️ **重要提醒**：高频控制（arm/chassis/head/gripper 等实时控制）、视频流、VR遥操 已分离到 `data_plane` 和 `media_plane`！
> Control Plane 只负责**低频管理操作**（< 5Hz）。

---

## 📊 新旧对照与实现状态

### 旧版 API 迁移分析

| 旧版 API 文件 | 迁移目标 | 状态 | 说明 |
|--------------|---------|------|------|
| `auth.py` | control_plane | ✅ 已实现 | 登录/登出/Token |
| `control.py` | control_plane | ✅ 已实现 | 控制权管理 |
| `tasks.py` | control_plane | ⚠️ 部分完成 | 缺 ROS2 集成 |
| `preset.py` | control_plane | ✅ 已实现 | 预设 CRUD |
| `arm_points.py` | control_plane | ✅ 已实现 | 合并到预设管理 |
| `head_points.py` | control_plane | ✅ 已实现 | 合并到预设管理 |
| `lift_points.py` | control_plane | ✅ 已实现 | 合并到预设管理 |
| `waist_points.py` | control_plane | ✅ 已实现 | 合并到预设管理 |
| `recording.py` | control_plane | ✅ 已实现 | 录制生命周期管理（待 ROS2 集成） |
| `actions.py` | control_plane | ✅ 已实现 | 动作模型管理 |
| `shutdown.py` | control_plane | ✅ 已实现 | 系统关机/重启 |
| `status.py` | control_plane | ✅ 已实现 | 合并到 robot.py |
| `robot.py` | control_plane | ✅ 已实现 | 机器人信息/状态概览/URDF |
| `arm.py` | ❌ **data_plane** | - | 高频控制，不在此实现 |
| `head.py` | ❌ **data_plane** | - | 高频控制，不在此实现 |
| `lift.py` | ❌ **data_plane** | - | 高频控制，不在此实现 |
| `waist.py` | ❌ **data_plane** | - | 高频控制，不在此实现 |
| `gripper.py` | ❌ **data_plane** | - | 高频控制，不在此实现 |
| `chassis.py` | ❌ **data_plane** | - | 高频控制，不在此实现 |
| `chassis_manual.py` | ❌ **data_plane** | - | 高频控制，不在此实现 |
| `vr_teleoperation.py` | ❌ **data_plane** | - | VR 遥操，不在此实现 |
| `websocket.py` | ❌ **data_plane** | - | 高频状态推送，不在此实现 |
| `camera.py` | ❌ **media_plane** | - | 视频流，不在此实现 |
| `emergency.py` | 特殊 | ⚠️ 待讨论 | 急停可能需要双通道 |

### Control Plane 当前实现状态

| 模块 | 状态 | 说明 |
|------|------|------|
| 认证 (`auth.py`) | ✅ 完成 | 登录/登出/Token刷新/当前用户 |
| 系统配置 (`system.py`) | ✅ 完成 | 服务发现/健康检查/系统信息 |
| 控制权 (`control.py`) | ✅ 完成 | 获取/释放/续约/强制释放 |
| 模式管理 (`mode.py`) | ✅ 完成 | idle/teleop/auto/maintenance 状态机 |
| 任务管理 (`tasks.py`) | ⚠️ 部分完成 | CRUD 完成，缺 ROS2 通知 |
| **预设管理** | ✅ 已实现 | schemas/preset.py, services/preset_manager.py, api/v1/presets.py |
| **录制管理** | ✅ 已实现 | schemas/recording.py, api/v1/recording.py (待 ROS2 集成) |
| **动作管理** | ✅ 已实现 | schemas/action.py, api/v1/actions.py |
| **机器人信息** | ✅ 已实现 | schemas/robot.py, api/v1/robot.py (URDF/状态概览) |
| **系统关机** | ✅ 已实现 | api/v1/robot.py (关机/重启) |
| **审计日志** | ✅ 已实现 | schemas/audit.py, services/audit_service.py, api/v1/audit.py |

---

## 1. 🔴 需要新增的 API 模块

### 1.1 预设/点位管理 (`api/v1/presets.py`)

整合旧版 `arm_points.py`, `head_points.py`, `lift_points.py`, `waist_points.py`, `preset.py`

**统一预设类型**:
| 类型 | 说明 | 旧文件来源 |
|------|------|-----------|
| `arm_pose` | 机械臂点位 | arm_points.py |
| `head_position` | 头部点位 | head_points.py |
| `lift_height` | 升降高度 | lift_points.py |
| `waist_angle` | 腰部角度 | waist_points.py |
| `location` | 底盘导航点 | preset.py (从地图读取) |
| `gripper_position` | 夹爪位置 | - |
| `full_pose` | 完整姿态 | - |

**API 设计**:
```
GET    /api/v1/presets?type={preset_type}     # 列出指定类型预设
POST   /api/v1/presets                        # 创建预设
GET    /api/v1/presets/{id}                   # 获取预设详情
PUT    /api/v1/presets/{id}                   # 更新预设
DELETE /api/v1/presets/{id}                   # 删除预设（内置不可删）
POST   /api/v1/presets/{id}/apply             # 应用预设（通过 ROS2 Service）
POST   /api/v1/presets/capture                # 采集当前状态为新预设
```

**存储位置**: `~/qyh-robot-system/persistent/preset/*.json`（沿用旧版）

---

### 1.2 录制管理 (`api/v1/recording.py`)

参考 `old_backup/app/api/recording.py`，提供 rosbag 录制的生命周期管理

```
POST /api/v1/recording/start     # 开始录制
POST /api/v1/recording/stop      # 停止录制
POST /api/v1/recording/discard   # 丢弃当前录制
GET  /api/v1/recording/status    # 获取录制状态
GET  /api/v1/recording/list      # 列出录制文件
```

**关键参数**:
- `action_name`: 动作名称（决定保存目录）
- `user_name`: 用户名
- `version`: 版本号
- `topics`: 要录制的话题列表

**存储位置**: `~/qyh-robot-system/model_actions/{action_name}/data/bags/`

---

### 1.3 动作模型管理 (`api/v1/actions.py`)

参考 `old_backup/app/api/actions.py`，管理训练动作的元数据

```
GET  /api/v1/actions                     # 列出所有动作
GET  /api/v1/actions/{action_id}         # 获取动作详情
POST /api/v1/actions                     # 创建新动作
PUT  /api/v1/actions/{action_id}         # 更新动作配置
DELETE /api/v1/actions/{action_id}       # 删除动作
GET  /api/v1/actions/{action_id}/episodes # 获取轨迹列表
```

**动作状态**:
- `collecting` - 采集中
- `trained` - 已训练

**存储位置**: `~/qyh-robot-system/model_actions/{robot_name}/{version}/{action_id}/`

---

### 1.4 机器人信息 (`api/v1/robot.py`)

参考 `old_backup/app/api/robot.py` 和 `status.py`

```
GET /api/v1/robot/info       # 机器人基本信息
GET /api/v1/robot/urdf       # 获取 URDF 模型（XML）
GET /api/v1/robot/overview   # 状态概览（各子系统连接状态）
```

**注意**: 这里只提供**低频查询**，高频状态推送走 data_plane WebSocket

---

### 1.5 系统关机 (`api/v1/shutdown.py`)

参考 `old_backup/app/api/shutdown.py`

```
GET  /api/v1/system/shutdown/state   # 获取关机状态
POST /api/v1/system/shutdown         # 触发系统关机（需管理员）
```

---

### 1.6 审计日志 (`api/v1/audit.py`)

```
GET /api/v1/audit/logs               # 查询审计日志
GET /api/v1/audit/logs/{id}          # 日志详情
GET /api/v1/audit/export             # 导出日志（CSV）
```

---

## 2. 🟡 需要完善的现有模块

### 2.1 系统健康检查 (`api/v1/system.py`)

需要实现实际的服务健康检查：

```python
# 检查 Data Plane 健康
async def check_data_plane_health() -> ServiceHealth:
    # 通过 HTTP 检查 ws://host:8765/health
    
# 检查 Media Plane 健康  
async def check_media_plane_health() -> ServiceHealth:
    # 通过 HTTP 检查 http://host:8888/health

# 检查 ROS2 连接（通过服务调用）
async def check_ros2_connection() -> ServiceHealth:
    # 调用 ROS2 Service 检查
```

---

### 2.2 任务管理 ROS2 集成 (`api/v1/tasks.py`)

需要实现任务操作的 ROS2 通知：

```python
# 位置：tasks.py

# 启动任务时
await ros2_client.call_service('task_engine/start_task', {...})

# 暂停任务时
await ros2_client.call_service('task_engine/pause_task', {...})

# 恢复任务时
await ros2_client.call_service('task_engine/resume_task', {...})

# 取消任务时
await ros2_client.call_service('task_engine/cancel_task', {...})
```

---

### 2.3 控制权会话持久化

当前 `control_lock.py` 是内存实现，需要：
- [ ] 获取控制权时写入 `control_sessions` 表
- [ ] 释放时更新 `ended_at` 和 `end_reason`
- [ ] 支持查询历史控制会话

---

## 3. 🔵 服务层 (`services/`) - 需新建

### 3.1 ROS2 服务客户端 (`services/ros2_client.py`)

参考 `old_backup/app/api/ros_client.py`

```python
class ROS2ServiceClient:
    """
    ROS2 服务调用客户端
    
    ⚠️ 注意：只用于低频管理操作，禁止订阅高频 Topic！
    """
    
    # 录制控制（调用 qyh_bag_recorder 服务）
    async def start_recording(self, action_name, user_name, version, topics) -> dict
    async def stop_recording(self) -> dict
    async def get_recording_status(self) -> dict
    
    # 任务控制
    async def start_task(self, task_id: int, program: dict) -> bool
    async def pause_task(self, task_id: int) -> bool
    async def cancel_task(self, task_id: int) -> bool
    
    # 预设应用（一次性动作）
    async def apply_arm_pose(self, joints: dict, velocity: float) -> bool
    async def apply_head_position(self, pan: float, tilt: float) -> bool
    async def apply_lift_height(self, height: float) -> bool
    async def apply_waist_angle(self, angle: float) -> bool
    
    # 系统
    async def call_shutdown(self) -> dict
    async def check_connection(self) -> bool
```

---

### 3.2 预设管理器 (`services/preset_manager.py`)

参考 `old_backup/app/preset/manager.py`，提供预设的持久化管理

```python
class PresetManager:
    """预设管理器（文件存储）"""
    
    def list(self, preset_type: str, category: str = None) -> List[dict]
    def get(self, preset_type: str, preset_id: str) -> dict
    def create(self, preset_type: str, data: dict) -> dict
    def update(self, preset_type: str, preset_id: str, data: dict) -> dict
    def delete(self, preset_type: str, preset_id: str) -> bool
    def capture_current(self, preset_type: str, name: str) -> dict  # 需要 ROS2
```

---

### 3.3 健康检查器 (`services/health_checker.py`)

```python
class HealthChecker:
    """服务健康检查器"""
    
    async def check_data_plane(self) -> dict
    async def check_media_plane(self) -> dict
    async def check_ros2(self) -> dict
    async def check_all(self) -> dict
```

---

## 4. 🟣 需要新增的 Schema

### 4.1 预设 Schema (`schemas/preset.py`)

```python
from enum import Enum

class PresetType(str, Enum):
    ARM_POSE = "arm_pose"
    HEAD_POSITION = "head_position"
    LIFT_HEIGHT = "lift_height"
    WAIST_ANGLE = "waist_angle"
    LOCATION = "location"
    GRIPPER_POSITION = "gripper_position"
    FULL_POSE = "full_pose"

class PresetBase(BaseModel):
    name: str
    description: str = ""
    category: str = "custom"
    preset_type: PresetType
    data: dict

class CreatePresetRequest(BaseModel):
    name: str
    description: str = ""
    category: str = "custom"
    preset_type: PresetType
    data: dict

class CapturePresetRequest(BaseModel):
    name: str
    description: str = ""
    preset_type: PresetType
    # 采集参数
    side: str = "both"  # 对于 arm_pose: left/right/both

class ApplyPresetRequest(BaseModel):
    velocity: float = 0.5
    acceleration: float = 0.3
    side: str = "both"

class PresetInfo(BaseModel):
    id: str
    name: str
    description: str
    category: str
    preset_type: str
    is_builtin: bool
    data: dict
    created_at: str
    updated_at: str
```

---

### 4.2 录制 Schema (`schemas/recording.py`)

```python
class StartRecordingRequest(BaseModel):
    action_name: str
    user_name: str
    version: str
    topics: List[str]

class RecordingStatus(BaseModel):
    is_recording: bool
    action_name: str = ""
    user_name: str = ""
    duration_seconds: float = 0.0
    bag_path: str = ""
    topics: List[str] = []
```

---

### 4.3 动作 Schema (`schemas/action.py`)

```python
class ActionStatus(str, Enum):
    COLLECTING = "collecting"
    TRAINED = "trained"

class ActionSummary(BaseModel):
    id: str
    name: str
    description: str = ""
    status: ActionStatus
    has_model: bool
    episode_count: int
    robot_name: str
    robot_version: str
    created_at: str
    updated_at: str

class CreateActionRequest(BaseModel):
    id: str  # 动作 ID，如 pickup_cube
    name: str
    description: str = ""
    collection: dict = {}  # 采集配置
    training: dict = {}    # 训练配置
```

---

### 4.4 机器人 Schema (`schemas/robot.py`)

```python
class RobotOverview(BaseModel):
    ros_connected: bool
    timestamp: float
    subsystems: dict  # 各子系统连接状态
    battery_percentage: Optional[float]
    is_emergency_stopped: bool
    current_mode: str
```

---

## 5. 📋 实现优先级

### P0 - 立即需要（前端依赖）

| 优先级 | 功能 | 原因 |
|--------|------|------|
| 1 | 预设管理 API | 前端点位示教依赖 |
| 2 | 录制管理 API | 数据采集依赖 |
| 3 | ROS2 服务客户端 | 预设应用/录制控制需要 |
| 4 | 动作管理 API | 训练流程依赖 |

### P1 - 尽快完成（功能完整）

| 优先级 | 功能 | 原因 |
|--------|------|------|
| 5 | 系统健康检查完善 | 运维监控 |
| 6 | 机器人信息/状态概览 | 仪表板显示 |
| 7 | 任务 ROS2 集成 | 自主任务执行 |
| 8 | 系统关机 | 安全需求 |

### P2 - 可以延后

| 优先级 | 功能 | 原因 |
|--------|------|------|
| 9 | 审计日志 API | 合规需求 |
| 10 | 控制权会话持久化 | 状态恢复 |

---

## 6. ❌ 明确不在 Control Plane 实现的功能

以下功能已迁移到 Data Plane 或 Media Plane，**禁止**在 Control Plane 实现：

### 高频控制（走 Data Plane WebSocket）
- 机械臂连续控制 (`arm.py`)
- 底盘速度控制 (`chassis.py`, `chassis_manual.py`)
- 头部连续控制 (`head.py`)
- 升降连续控制 (`lift.py`)
- 腰部连续控制 (`waist.py`)
- 夹爪连续控制 (`gripper.py`)
- VR 遥操控制 (`vr_teleoperation.py`)
- 高频状态推送 (`websocket.py`)

### 视频流（走 Media Plane WebRTC）
- 相机画面 (`camera.py`)

### 为什么？
根据 `重构.md` 核心原则：
> ❌ FastAPI 禁止使用 `asyncio.sleep()` 实现任何形式的高频循环
> ❌ FastAPI 禁止订阅 ROS Topic 并持续推流
> ❌ FastAPI 禁止承载 WebSocket 高频状态流
> ❌ FastAPI 禁止承载视频/图像流

---

## 7. 🔗 相关文档

- [重构.md](../重构.md) - 架构设计原则
- [重构开发计划.md](../重构开发计划.md) - 详细开发计划
- [重构补充.md](../重构补充.md) - 工程细节补充
- [README.md](../README.md) - 项目说明

---

*最后更新：2026-01-19*
