# 后端代码审查计划 (Backend Code Review Plan)

**目标**: 确保后端代码与 API 文档一致，验证三平面架构的正确实现，检查安全性、性能和可维护性。

**执行策略**: 分阶段审查，发现问题后记录在 `backend_audit_log.md` 中，并提供修复建议。

---

## 📅 阶段 1: 架构与配置 (Architecture & Configuration)
**目标**: 验证三平面分离架构的正确性，确保配置管理规范。

- [ ] **1.1 项目结构**
    - 审查文件: 整体目录结构
    - 检查点:
        - Control Plane、Data Plane、Media Plane 是否独立。
        - 共享 Protobuf 定义是否正确放置在 `shared/proto/`。
        - 部署配置是否完整。

- [ ] **1.2 API 文档一致性**
    - 审查文件: `API_DOCUMENTATION.md`
    - 检查点:
        - 文档版本是否与实际代码版本一致。
        - 端口定义是否正确 (8000/8765/8888)。
        - 接口职责划分是否清晰（HTTP vs WebSocket vs WebRTC）。
        - 禁止的设计模式是否在文档中明确说明。

- [ ] **1.3 配置文件**
    - 审查文件: `control_plane/.env.example`, `data_plane/config/config.yaml`, `media_plane/config/config.yaml`
    - 检查点:
        - 环境变量定义是否完整。
        - 敏感信息是否正确处理（不应硬编码）。
        - 配置项是否与代码一致。

## 📅 阶段 2: Control Plane - 认证与安全 (Auth & Security)
**目标**: 确保认证机制安全可靠，权限控制正确实现。

- [ ] **2.1 认证实现**
    - 审查文件: `control_plane/app/core/security.py`, `control_plane/app/api/v1/auth.py`
    - 对照文档: Section 3.1
    - 检查点:
        - JWT Token 生成与验证是否安全。
        - Token 刷新机制是否正确。
        - 密码加密是否使用安全算法。
        - 登出是否正确清理 Token。

- [ ] **2.2 权限控制**
    - 审查文件: `control_plane/app/dependencies.py`
    - 检查点:
        - `get_current_user`, `get_current_operator`, `get_current_admin` 是否正确实现。
        - 角色权限检查是否严格。
        - API 路由是否正确应用权限装饰器。

- [ ] **2.3 控制权管理**
    - 审查文件: `control_plane/app/core/control_lock.py`, `control_plane/app/api/v1/control.py`
    - 对照文档: Section 3.3
    - 检查点:
        - 控制锁的互斥性是否保证。
        - 控制权超时是否正确处理。
        - 强制释放功能是否仅限管理员。

## 📅 阶段 3: Control Plane - 业务逻辑 (Business Logic)
**目标**: 验证核心业务逻辑的正确性和完整性。

- [ ] **3.1 模式管理**
    - 审查文件: `control_plane/app/core/mode_manager.py`, `control_plane/app/api/v1/mode.py`
    - 对照文档: Section 3.5
    - 检查点:
        - 模式状态机是否正确实现。
        - 模式切换规则是否符合文档。

- [ ] **3.2 任务管理**
    - 审查文件: `control_plane/app/api/v1/tasks.py`, `control_plane/app/models/task.py`
    - 对照文档: Section 3.6
    - 检查点:
        - CRUD 接口是否完整。
        - 任务状态流转是否正确。
        - 节点类型定义是否与前端一致。

- [ ] **3.3 预设管理**
    - 审查文件: `control_plane/app/api/v1/presets.py`, `control_plane/app/services/preset_manager.py`
    - 对照文档: Section 3.7
    - 检查点:
        - 预设类型定义是否完整。
        - 内置预设是否不可删除。
        - 预设应用逻辑是否正确调用 ROS2。

- [ ] **3.4 录制管理**
    - 审查文件: `control_plane/app/api/v1/recording.py`
    - 对照文档: Section 3.8
    - 检查点:
        - 是否只有 start/stop/discard 接口（无 pause/resume）。
        - 是否正确调用 ROS2 录制服务。

## 📅 阶段 4: Control Plane - 硬件接口 (Hardware Interfaces)
**目标**: 确保硬件控制接口职责清晰，不违反架构原则。

- [ ] **4.1 底盘接口**
    - 审查文件: `control_plane/app/api/v1/chassis.py`
    - 对照文档: Section 3.10
    - 检查点:
        - ❌ 是否存在速度控制接口（应仅在 WebSocket）。
        - ✅ 是否只有配置、状态查询、导航接口。
        - 导航接口是否正确调用 ROS2。

- [ ] **4.2 机械臂接口**
    - 审查文件: `control_plane/app/api/v1/arm.py`
    - 对照文档: Section 3.11
    - 检查点:
        - ❌ 是否存在高频运动控制接口（Jog/Move 应走 WebSocket）。
        - ✅ 是否只有电源、使能、负载、点位 CRUD 接口。

- [ ] **4.3 其他硬件模块**
    - 审查文件: `control_plane/app/api/v1/led.py`, `control_plane/app/api/v1/camera.py`, `control_plane/app/api/v1/vr.py`
    - 对照文档: Section 3.12-3.14
    - 检查点: 接口职责是否符合文档。

## 📅 阶段 5: Control Plane - ROS2 集成 (ROS2 Integration)
**目标**: 验证与 ROS2 的通信是否正确实现。

- [ ] **5.1 ROS2 客户端**
    - 审查文件: `control_plane/app/services/ros2_client.py`
    - 检查点:
        - ROS2 服务调用是否异步化。
        - 错误处理是否完善。
        - 超时机制是否正确。
        - 服务名称是否与 ROS2 节点一致。

- [ ] **5.2 健康检查**
    - 审查文件: `control_plane/app/services/health_checker.py`
    - 对照文档: Section 3.2.2
    - 检查点:
        - 是否检查 ROS2 连接状态。
        - 是否检查 Data Plane 连接。
        - 是否检查 Media Plane 连接。

## 📅 阶段 6: Data Plane - 架构与协议 (Architecture & Protocol)
**目标**: 验证 WebSocket + Protobuf 实现的正确性。

- [ ] **6.1 Protobuf 定义**
    - 审查文件: `shared/proto/*.proto`
    - 检查点:
        - 消息类型枚举是否与文档一致。
        - 消息结构是否完整。
        - 字段命名是否规范。

- [ ] **6.2 服务器实现**
    - 审查文件: `data_plane/src/server.cpp`, `data_plane/include/data_plane/server.hpp`
    - 检查点:
        - WebSocket 服务器是否使用 Boost.Beast。
        - 端口是否为 8765。
        - 并发处理是否正确。

- [ ] **6.3 认证机制**
    - 审查文件: `data_plane/src/auth.cpp`
    - 检查点:
        - 是否验证 JWT Token。
        - 是否与 Control Plane 共享密钥。
        - 认证失败是否正确拒绝连接。

- [ ] **6.4 Watchdog 机制**
    - 审查文件: `data_plane/src/watchdog.cpp`
    - 检查点:
        - 心跳超时阈值是否为 200ms。
        - 超时是否触发紧急停止。
        - 是否正确清理超时连接。

## 📅 阶段 7: Data Plane - 消息处理 (Message Handling)
**目标**: 验证控制命令和状态推送的正确性。

- [ ] **7.1 消息处理器**
    - 审查文件: `data_plane/src/message_handler.cpp`
    - 检查点:
        - 是否正确处理所有消息类型。
        - 是否正确调用 ROS2 桥接。
        - 错误处理是否完善。

- [ ] **7.2 ROS2 桥接**
    - 审查文件: `data_plane/src/ros2_bridge.cpp`
    - 检查点:
        - 控制命令是否正确发布到 ROS2 话题。
        - 状态订阅是否正确。
        - 频率控制是否正确（避免过载）。

- [ ] **7.3 限流器**
    - 审查文件: `data_plane/src/rate_limiter.cpp`
    - 检查点:
        - 是否对高频消息进行限流。
        - 限流阈值是否合理。

## 📅 阶段 8: Media Plane - WebRTC 实现 (WebRTC Implementation)
**目标**: 验证视频流传输的正确性。

- [ ] **8.1 信令服务器**
    - 审查文件: `media_plane/src/signaling_server.cpp`
    - 检查点:
        - 端口是否为 8888。
        - 是否正确处理 SDP offer/answer。
        - 是否正确处理 ICE candidates。

- [ ] **8.2 视频管道**
    - 审查文件: `media_plane/src/pipeline_manager.cpp`
    - 检查点:
        - 是否使用 GStreamer。
        - 是否支持多摄像头。
        - 编码格式是否正确（H264/VP8）。

- [ ] **8.3 ROS2 图像源**
    - 审查文件: `media_plane/src/ros2_image_source.cpp`
    - 检查点:
        - 是否正确订阅 ROS2 图像话题。
        - 帧率是否可配置。

## 📅 阶段 9: 部署与文档 (Deployment & Documentation)
**目标**: 确保部署配置正确，文档完整。

- [ ] **9.1 Systemd 服务**
    - 审查文件: `deploy/systemd/*.service`
    - 检查点:
        - 服务依赖关系是否正确。
        - 环境变量是否正确加载。
        - 重启策略是否合理。

- [ ] **9.2 安装脚本**
    - 审查文件: `deploy/install.sh`
    - 检查点:
        - 是否正确安装所有服务。
        - 权限设置是否正确。
        - 是否创建必要的目录。

- [ ] **9.3 README 文档**
    - 审查文件: `README.md`
    - 检查点:
        - 架构说明是否清晰。
        - 快速开始指南是否可用。
        - 接口职责说明是否准确。

---
**审查记录格式**:
对于每个发现的问题，使用以下格式记录：
- **[Severity]** (Critical/Major/Minor/Info)
- **位置**: 文件路径 + 行号
- **问题**: 描述问题
- **建议**: 修复建议
- **优先级**: P0/P1/P2/P3
