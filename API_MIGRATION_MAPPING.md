# QYH Jushen 后端 API 迁移映射文档 (最终版)

> **文档版本**: 1.2 (Final)
> **生成日期**: 2026-02-07
> **修正说明**:
> 1. 修正了底盘特殊控制(充电/复位)的状态误判，以及状态查询接口的准确描述。
> 2. 补充了 Admin Tools (ROS GUI, Terminal) 的缺失情况。
> 3. 修正了 Robot Model 相关的发现（`/urdf` 和 `/package` 接口存在但需要验证路径有效性）。

---

## 📋 概述

本文档详细记录了从旧版单体 FastAPI 架构到新版三平面分离架构的接口映射关系。

### 状态标注说明

| 标注 | 说明 |
|------|------|
| ✅ **已实现** | 新版已实现 (包括 HTTP 接口保留) |
| ⚠️ **差异/部分** | 功能存在但接口有变，或仅部分实现(如Stub建议验证) |
| 🔄 **WebSocket** | 推荐使用 WebSocket (Data Plane) 以获得实时性 |
| ❌ **缺失** | 新版代码中完全未找到对应实现 |
| 🆕 **新增** | 新版新增功能 |

---

## 1. 认证与授权 (Auth)

| 旧版接口 | Method | 新版接口 | 状态 | 说明 |
|---------|--------|---------|------|------|
| `/api/v1/auth/login` | POST | `/api/v1/auth/login` | ✅ 已实现 | 完全兼容 |
| `/api/v1/auth/logout` | POST | `/api/v1/auth/logout` | ✅ 已实现 | 完全兼容 |
| `/api/v1/auth/heartbeat` | POST | `MSG_HEARTBEAT` (WS) | 🔄 WebSocket | 推荐使用 WS 心跳机制 (每200ms) |

---

## 2. 机器人通用 (Robot)

| 旧版接口 | Method | 新版接口 | 状态 | 说明 |
|---------|--------|---------|------|------|
| `/api/v1/robot/status` | GET | `/api/v1/robot/status` | ✅ 已实现 | **HTTP快照** (用于初始化)。实时更新请用 WS。 |
| `/api/v1/robot/urdf` | GET | `/api/v1/robot/urdf` | ✅ 已实现 | 已包含真实文件读取逻辑，需确保路径正确。 |
| (资源文件) | - | `/api/v1/robot/package/{pkg}/{path}`| ✅ 已实现 | 新增了通用资源文件读取接口，支持 mesh 加载。 |
| `/api/v1/system/health` | GET | `/api/v1/system/health` | ✅ 已实现 | 健康检查 |
| `/api/v1/system/config` | GET | `/api/v1/system/config` | 🆕 新增 | 服务发现（WS地址等） |

---

## 3. 机械臂控制 (Arm)

| 旧版接口 | Method | 新版接口 | 状态 | 说明 |
|---------|--------|---------|------|------|
| `/api/v1/arm/power_on` | POST | `/api/v1/arm/power_on` | ✅ 已实现 | 上电 |
| `/api/v1/arm/power_off` | POST | `/api/v1/arm/power_off` | ✅ 已实现 | 下电 |
| `/api/v1/arm/enable` | POST | `/api/v1/arm/enable` | ✅ 已实现 | 使能 |
| `/api/v1/arm/disable` | POST | `/api/v1/arm/disable` | ✅ 已实现 | 去使能 |
| `/api/v1/arm/connect` | POST | (无) | ❌ 缺失 | 应该由底层自动连接，无显式接口 |
| `/api/v1/arm/disconnect` | POST | (无) | ❌ 缺失 | 同上 |
| `/api/v1/arm/move_j` | POST | `MSG_ARM_MOVE` (WS) | 🔄 WebSocket | 运动控制已全面迁移至 DataPlane |
| `/api/v1/arm/jog` | POST | `MSG_ARM_JOG` (WS) | 🔄 WebSocket | 点动控制已全面迁移 |
| `/api/v1/arm/payload/*` | - | `/api/v1/arm/payload/*` | ✅ 已实现 | 负载配置完整保留 |
| `/api/v1/arm/points/*` | - | `/api/v1/presets` | ⚠️ 架构变更 | 点位并入统一的 `presets` 系统 (type=`arm_pose`) |

---

## 4. 底盘控制 (Chassis)

### 4.1 基础控制与状态

| 旧版接口 | Method | 新版接口 | 状态 | 说明 |
|---------|--------|---------|------|------|
| `/api/v1/chassis/status` | GET | `/api/v1/chassis/status` | ✅ 已实现 | **HTTP快照**。实时更新请订阅 WS `chassis_state`。 |
| `/api/v1/chassis/velocity` | POST | `MSG_CHASSIS_VELOCITY` | 🔄 WebSocket | 推荐使用 WS 控制速度 |
| `/api/v1/chassis/manual_velocity`| POST | `/api/v1/chassis/manual_velocity`| ✅ 已实现 | HTTP 备用通道 |

### 4.2 特殊功能

| 旧版接口 | Method | 新版接口 | 状态 | 说明 |
|---------|--------|---------|------|------|
| `/api/v1/chassis/start_charging` | POST | `/api/v1/chassis/start_charging` | ✅ 已实现 | 充电控制 |
| `/api/v1/chassis/stop_charging` | POST | `/api/v1/chassis/stop_charging` | ✅ 已实现 | 停止充电 |
| `/api/v1/chassis/system_reset` | POST | `/api/v1/chassis/system_reset` | ✅ 已实现 | 系统复位 |
| `/api/v1/chassis/stop_localization`| POST | `/api/v1/chassis/stop_localization`| ✅ 已实现 | 停止定位 |
| `/api/v1/chassis/enter_low_power`| POST | (无) | ❌ 缺失 | **低功耗模式确实未找到实现** |

### 4.3 导航与地图

| 旧版接口 | Method | 新版接口 | 状态 | 说明 |
|---------|--------|---------|------|------|
| `/api/v1/chassis/navigate/coordinate`| POST | `/api/v1/chassis/navigate/pose` | ✅ 已实现 | 接口名微调 `coordinate` -> `pose` |
| `/api/v1/chassis/navigate/site` | POST | `/api/v1/chassis/navigate/station` | ✅ 已实现 | 接口名微调 `site` -> `station` |
| `/api/v1/chassis/navigate/cancel` | POST | `/api/v1/chassis/navigate/cancel`| ✅ 已实现 | HTTP 取消导航接口 |
| `/api/v1/chassis/map_data` | GET | `/api/v1/chassis/map_data` | ✅ 已实现 | 获取地图元数据 |
| `/api/v1/chassis/map_image/*` | GET | `/api/v1/chassis/map_image/*` | ✅ 已实现 | 获取地图图片 |

---

## 5. 夹爪 (Gripper)

| 旧版接口 | Method | 新版接口 | 状态 | 说明 |
|---------|--------|---------|------|------|
| `/api/v1/gripper/enable` | POST | (无) | ❌ 缺失 | **严重缺失**: 未在代码中发现单独的夹爪使能接口 |
| `/api/v1/gripper/activate` | POST | (无) | ❌ 缺失 | **严重缺失**: 未在代码中发现单独的夹爪激活接口 |
| `/api/v1/gripper/move` | POST | `MSG_GRIPPER_COMMAND` | 🔄 WebSocket | 夹爪运动已迁移至 WebSocket |
| (无) | - | `/api/v1/arm/payload/apply_gripper` | ✅ 已实现 | 夹爪负载配置存在 |
| `/api/v1/gripper/presets` | GET | `/api/v1/presets` | ⚠️ 架构变更 | 整合到 presets (type=`gripper_position`) |

---

## 6. 管理工具 (Admin Tools)

| 旧版接口 | 功能 | 新版状态 | 说明 |
|---------|------|---------|------|
| `/api/v1/ros_gui/*` | 远程启动 GUI | ❌ 缺失 | 新版完全移除了 ROS GUI 进程管理功能 |
| `/api/v1/terminal/*` | Web 终端 | ❌ 缺失 | 新版完全移除了 Web Terminal 功能 |
| `/api/v1/shutdown` | 系统关机 | ✅ 已实现 | 位于 `api/v1/robot.py` (需要Admin权限) |
| `/api/v1/reboot` | 系统重启 | ✅ 已实现 | 位于 `api/v1/robot.py` (需要Admin权限) |

---

## 7. 其他 (Misc)

| 功能域 | 新版状态 | 说明 |
|-------|---------|------|
| **预设 (Presets)** | ✅ 统一重构 | 原 Arm/Head/Lift Points 全部整合为 `presets` API |
| **模式 (Mode)** | 🆕 新增 | 新增 `/api/v1/mode/*` 管理 (Idle/Auto/Manual) |
| **任务 (Tasks)** | ✅ 已实现 | 任务系统基本完整 |
| **升降/腰部** | 🔄 WebSocket | 实时控制迁移至 WS，预设迁移至 `presets` API |

---

## 🕵️‍♂️ 总结与行动建议

1.  **Backend 需补全功能 (优先级排序)**:
    *   **High**: **Gripper Enable/Activate** 接口。 如果没有这个，夹爪可能无法初始化。
    *   **Medium**: **Low Power Mode** (底盘低功耗)。
    *   **Low**: **Admin Tools** (ROS GUI/Terminal)，如果是设计上有意移除则可忽略。

2.  **Frontend 需适配变更**:
    *   **Arm Points**: 必须重写为使用 `presets` API。
    *   **Navigation**: 调整 API 路径 (`navigate/pose`, `navigate/station`)。
    *   **Robot Model**: 移除对 ROS GUI 的依赖，确保使用新的 `/api/v1/robot/package/` 接口加载模型资源。
    *   **Gripper**: 确认夹爪是否需要显式 Enable/Activate 按钮，还是开机自启动。
