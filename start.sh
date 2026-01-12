#!/bin/bash

# QYH Jushen Web 后端启动脚本

# 读取 ROS_DOMAIN_ID
ROS_DOMAIN_ID_FILE="$HOME/qyh-robot-system/persistent/ros/ROS_DOMAIN_ID"
if [ -f "$ROS_DOMAIN_ID_FILE" ]; then
    export ROS_DOMAIN_ID=$(cat "$ROS_DOMAIN_ID_FILE")
else
    export ROS_DOMAIN_ID=0
fi
echo "🔧 ROS_DOMAIN_ID = $ROS_DOMAIN_ID"

# Source 全局配置（GLOBAL_ROBOT_NAME, GLOBAL_ROBOT_VERSION）
CONFIG_FILE="$HOME/qyh-robot-system/qyh_jushen_ws/config.bash"
if [ -f "$CONFIG_FILE" ]; then
    source "$CONFIG_FILE"
    echo "✅ 已加载全局配置: ROBOT_NAME=$GLOBAL_ROBOT_NAME, VERSION=$GLOBAL_ROBOT_VERSION"
else
    echo "⚠️  未找到配置文件: $CONFIG_FILE，使用默认值"
    export GLOBAL_ROBOT_NAME=general
    export GLOBAL_ROBOT_VERSION=1.0
fi

# Source ROS2 环境
source /opt/ros/humble/setup.bash

# Source 工作空间（包含 qyh_lift_msgs 等自定义消息）
WS_SETUP="$HOME/qyh-robot-system/qyh_jushen_ws/install/setup.bash"
if [ -f "$WS_SETUP" ]; then
    source "$WS_SETUP"
    echo "✅ 已加载 ROS2 工作空间"
else
    echo "⚠️  未找到工作空间: $WS_SETUP"
fi

# 切换到后端目录
cd "$(dirname "$0")"

# 检查是否有一个已经在运行的占用8000端口的进程，如果有则杀掉它
PORT=8000
if lsof -i:$PORT -t >/dev/null ; then
    PID=$(lsof -i:$PORT -t)
    echo "⚠️  端口 $PORT 已被占用，正在终止进程 $PID ..."
    kill -9 $PID
    echo "✅ 已终止进程 $PID"
fi

# 激活虚拟环境并启动
source venv/bin/activate
echo "🚀 启动后端服务器..."
python -m uvicorn app.main:app --host 0.0.0.0 --port $PORT
