# QYH Jushen Web - Backend

基于 FastAPI + ROS2 的机器人Web控制后端。

## 快速开始

### 1. 安装依赖

```bash
# 确保已安装 ROS2 Humble
source /opt/ros/humble/setup.bash

# 安装 Python 依赖
pip install -r requirements.txt
```

### 2. 配置环境变量

```bash
# 复制配置文件
cp .env.example .env

# 编辑 .env 修改配置（特别是 SECRET_KEY）
```

### 3. 运行

```bash
# 开发模式（Mock ROS2）
MOCK_MODE=true uvicorn app.main:app --reload

# 生产模式（需要真实 ROS2）
uvicorn app.main:app --host 0.0.0.0 --port 8000
```

### 4. 访问 API 文档

打开浏览器访问：
- Swagger UI: http://localhost:8000/docs
- ReDoc: http://localhost:8000/redoc

## 默认账号

- **用户名**: `admin`
- **密码**: `admin123`
- **角色**: Admin

## 项目结构

```
backend/
├── app/
│   ├── api/              # API 路由
│   ├── core/             # 核心业务逻辑
│   ├── models/           # 数据库模型
│   ├── schemas/          # Pydantic 模型
│   ├── ros2_bridge/      # ROS2 集成
│   ├── safety/           # 安全机制
│   ├── config.py         # 配置管理
│   ├── database.py       # 数据库连接
│   ├── dependencies.py   # 依赖注入
│   └── main.py           # 应用入口
├── data/                 # 数据目录（SQLite 数据库）
├── requirements.txt      # Python 依赖
├── .env.example          # 环境变量模板
└── Dockerfile            # Docker 镜像
```

## 开发指南

参考 `../doc/dev/05_BACKEND_GUIDE.md`
