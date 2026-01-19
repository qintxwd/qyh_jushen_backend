# Control Plane 实现状态报告

> 基于 `重构.md`、`重构开发计划.md`、`重构补充.md` 以及 `old_backup` 旧代码分析，整理 control_plane 实现状态。
>
> ⚠️ **重要提醒**：高频控制（arm/chassis/head/gripper 等实时控制）、视频流、VR遥操 已分离到 `data_plane` 和 `media_plane`！
> Control Plane 只负责**低频管理操作**（< 5Hz）。
>
> **最后更新**: 2025-01-19

---

## ✅ 实现完成总结

Control Plane 核心功能已全部实现，共 **79 个 API 端点**，覆盖 **11 个模块**。

**主要功能**：
- ✅ ROS2 服务集成（自动检测，支持 mock fallback）
- ✅ 健康检查（Data Plane / Media Plane / ROS2 连接检测）
- ✅ 审计日志（登录/控制权/模式切换等关键操作记录）
- ✅ 控制权持久化（数据库会话记录，支持历史查询和统计）

---

## 📊 新旧对照与实现状态

### 旧版 API 迁移分析

| 旧版 API 文件 | 迁移目标 | 状态 | 说明 |
|--------------|---------|------|------|
| `auth.py` | control_plane | ✅ 已实现 | 登录/登出/Token刷新/当前用户 |
| `control.py` | control_plane | ✅ 已实现 | 控制权获取/释放/续约/强制释放 |
| `tasks.py` | control_plane | ✅ 已实现 | 任务 CRUD + 执行控制 |
| `task_orchestration.py` | control_plane | ✅ 已合并 | 合并到 tasks.py |
| `preset.py` | control_plane | ✅ 已实现 | 统一预设管理 |
| `arm_points.py` | control_plane | ✅ 已实现 | 合并到 presets.py |
| `head_points.py` | control_plane | ✅ 已实现 | 合并到 presets.py |
| `lift_points.py` | control_plane | ✅ 已实现 | 合并到 presets.py |
| `waist_points.py` | control_plane | ✅ 已实现 | 合并到 presets.py |
| `recording.py` | control_plane | ✅ 已实现 | 录制生命周期管理 |
| `ros_client.py` | control_plane | ✅ 已合并 | 录制客户端合并到 recording.py |
| `actions.py` | control_plane | ✅ 已实现 | 动作模型管理 |
| `shutdown.py` | control_plane | ✅ 已实现 | 合并到 robot.py |
| `status.py` | control_plane | ✅ 已实现 | 合并到 robot.py |
| `robot.py` | control_plane | ✅ 已实现 | 机器人信息/状态概览/URDF |
| `robot_model.py` | control_plane | ✅ 已实现 | URDF 功能合并到 robot.py |
| `arm.py` | ❌ **data_plane** | ➡️ 迁移 | 高频控制，不在此实现 |
| `head.py` | ❌ **data_plane** | ➡️ 迁移 | 高频控制，不在此实现 |
| `lift.py` | ❌ **data_plane** | ➡️ 迁移 | 高频控制，不在此实现 |
| `waist.py` | ❌ **data_plane** | ➡️ 迁移 | 高频控制，不在此实现 |
| `gripper.py` | ❌ **data_plane** | ➡️ 迁移 | 高频控制，不在此实现 |
| `chassis.py` | ❌ **data_plane** | ➡️ 迁移 | 高频控制，不在此实现 |
| `chassis_manual.py` | ❌ **data_plane** | ➡️ 迁移 | 高频控制，不在此实现 |
| `vr_teleoperation.py` | ❌ **data_plane** | ➡️ 迁移 | VR 遥操，不在此实现 |
| `websocket.py` | ❌ **data_plane** | ➡️ 迁移 | 高频状态推送，不在此实现 |
| `camera.py` | ❌ **media_plane** | ➡️ 迁移 | 视频流，不在此实现 |
| `led.py` | ❌ **data_plane** | ➡️ 迁移 | LED 控制（低延迟要求） |
| `emergency.py` | ❌ **data_plane** | ➡️ 迁移 | 急停需要最低延迟 |
| `terminal.py` | ❌ 不迁移 | 🚫 安全原因 | WebSocket 终端不建议在生产环境使用 |
| `ros_gui.py` | ❌ 不迁移 | 🚫 开发调试 | ROS GUI 管理仅用于调试 |

### Control Plane 当前实现状态

| 模块 | 文件 | 端点数 | 状态 | 说明 |
|------|------|--------|------|------|
| 认证 | `auth.py` | 4 | ✅ 完成 | login/logout/me/refresh + 审计日志 |
| 系统配置 | `system.py` | 3 | ✅ 完成 | config/health/info（健康检查已实现） |
| 控制权 | `control.py` | 7 | ✅ 完成 | acquire/release/renew/status/force-release/history/statistics |
| 模式管理 | `mode.py` | 3 | ✅ 完成 | current/switch/available + 审计日志 |
| 任务管理 | `tasks.py` | 7 | ✅ 完成 | CRUD + start/pause/resume/cancel |
| 预设管理 | `presets.py` | 12 | ✅ 完成 | 统一 CRUD + apply/capture + 兼容接口 |
| 录制管理 | `recording.py` | 8 | ✅ 完成 | start/stop/discard/status/files/topics |
| 动作管理 | `actions.py` | 14 | ✅ 完成 | CRUD + episodes + inference-config |
| 机器人信息 | `robot.py` | 6 | ✅ 完成 | info/overview/urdf/shutdown/reboot |
| 审计日志 | `audit.py` | 7 | ✅ 完成 | 查询/统计/清理 |
| 健康检查 | `health.py` | 1 | ✅ 完成 | 根路径健康检查 |
| **总计** | **11 模块** | **79** | ✅ | |

---

## 🟡 后续优化工作（非阻塞）

### 1. ROS2 集成状态

✅ **已实现统一 ROS2 服务客户端** `app/services/ros2_client.py`：

| 模块 | 服务方法 | ROS2 服务 | 状态 |
|------|---------|-----------|------|
| 录制管理 | `start_recording()` | `/qyh_bag_recorder/start_recording` | ✅ 已集成 |
| 录制管理 | `stop_recording()` | `/qyh_bag_recorder/stop_recording` | ✅ 已集成 |
| 录制管理 | `get_recording_status()` | `/qyh_bag_recorder/get_status` | ✅ 已集成 |
| 任务执行 | `execute_task()` | `/task_engine/execute` | ✅ 已集成 |
| 任务执行 | `cancel_task()` | `/task_engine/cancel` | ✅ 已集成 |
| 任务执行 | `pause_task()` | `/task_engine/pause` | ✅ 已集成 |
| 任务执行 | `resume_task()` | `/task_engine/resume` | ✅ 已集成 |
| 预设应用 | `arm_move_j()` | `/jaka_control/move_j` | ✅ 已集成 |
| 预设应用 | `lift_go_position()` | `/lift/control` | ✅ 已集成 |
| 预设应用 | `waist_go_angle()` | `/waist/control` | ✅ 已集成 |
| 预设应用 | `gripper_move()` | `/gripper/move` | ✅ 已集成 |
| 关机/重启 | `request_shutdown()` | `/qyh_shutdown_node/control` | ✅ 已集成 |
| 关机/重启 | `request_reboot()` | `/qyh_shutdown_node/control` | ✅ 已集成 |
| 头部控制 | `head_enable_torque()` | `/head_motor/enable_torque` | ✅ 已集成 |

**特性**:
- 自动检测 `rclpy` 是否可用
- ROS2 不可用时自动 fallback 到 mock 模式
- 使用 `asyncio.run_in_executor()` 处理阻塞的 ROS2 调用
- 单例模式管理 ROS2 节点生命周期

### 2. 待完善：ROS2 Topic 订阅

预设采集功能需要订阅 ROS2 Topic 获取当前状态（目前使用模拟数据）：

| 功能 | 需要的 Topic | 状态 |
|------|-------------|------|
| 机械臂状态采集 | `/jaka/joint_states` | 🟡 待实现 |
| 升降高度采集 | `/lift/status` | 🟡 待实现 |
| 腰部角度采集 | `/waist/status` | 🟡 待实现 |
| 头部位置采集 | `/head/status` | 🟡 待实现 |

### 3. 健康检查 ✅ 已完成

`app/services/health_checker.py` 已实现：
- ✅ Data Plane (ws://host:8765) TCP 连接检查
- ✅ Media Plane (http://host:8888) HTTP 健康检查
- ✅ ROS2 连接状态检查（通过 ros2_client）
- ✅ 结果缓存（5秒 TTL）
- ✅ 并行检查所有服务

### 4. 控制权会话持久化 ✅ 已完成

`app/services/control_session_service.py` 已实现：
- ✅ 获取控制权时写入 `control_sessions` 表
- ✅ 释放时更新 `ended_at` 和 `end_reason`
- ✅ `/api/v1/control/history` - 查询历史控制会话
- ✅ `/api/v1/control/statistics` - 控制权统计信息（仅管理员）

### 5. 审计日志集成 ✅ 已完成

`app/services/audit_service.py` 已集成到关键 API：
- ✅ `auth.py` - 用户登录 (`user_login`)
- ✅ `control.py` - 控制权获取/释放/强制释放 (`control_acquire`, `control_release`, `control_force_release`)
- ✅ `mode.py` - 模式切换 (`mode_switch`)

---

## ❌ 明确不在 Control Plane 实现的功能

以下功能已迁移到 Data Plane 或 Media Plane，**禁止**在 Control Plane 实现：

### 高频控制（走 Data Plane WebSocket）
| 旧版文件 | 功能 | 迁移目标 |
|---------|------|---------|
| `arm.py` | 机械臂连续控制 | Data Plane |
| `chassis.py` | 底盘速度控制 | Data Plane |
| `chassis_manual.py` | 底盘手动控制 | Data Plane |
| `head.py` | 头部连续控制 | Data Plane |
| `lift.py` | 升降连续控制 | Data Plane |
| `waist.py` | 腰部连续控制 | Data Plane |
| `gripper.py` | 夹爪连续控制 | Data Plane |
| `vr_teleoperation.py` | VR 遥操控制 | Data Plane |
| `websocket.py` | 高频状态推送 | Data Plane |
| `led.py` | LED 灯带控制 | Data Plane |
| `emergency.py` | 急停控制 | Data Plane |

### 视频流（走 Media Plane WebRTC）
| 旧版文件 | 功能 | 迁移目标 |
|---------|------|---------|
| `camera.py` | 相机画面 | Media Plane |

### 不迁移（安全/调试功能）
| 旧版文件 | 功能 | 原因 |
|---------|------|------|
| `terminal.py` | WebSocket 终端 | 安全风险，不建议生产使用 |
| `ros_gui.py` | ROS GUI 管理 | 仅调试用途 |

### 为什么分离？

根据 `重构.md` 核心原则：
> ❌ FastAPI 禁止使用 `asyncio.sleep()` 实现任何形式的高频循环  
> ❌ FastAPI 禁止订阅 ROS Topic 并持续推流  
> ❌ FastAPI 禁止承载 WebSocket 高频状态流  
> ❌ FastAPI 禁止承载视频/图像流

---

## 🔗 相关文档

- [重构.md](../重构.md) - 架构设计原则
- [重构开发计划.md](../重构开发计划.md) - 详细开发计划
- [重构补充.md](../重构补充.md) - 工程细节补充
- [README.md](../README.md) - 项目说明

---

*最后更新：2025-01-19 - 健康检查、审计日志、控制权持久化完成*
