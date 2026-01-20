#!/bin/bash

# QYH Jushen Web 后端启动脚本 (Control Plane + Data Plane + Media Plane)

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

# 激活虚拟环境
if [ -f "venv/bin/activate" ]; then
    source venv/bin/activate
else
    echo "⚠️  未找到虚拟环境 venv，尝试直接运行..."
fi

# 杀死所有子进程的函数
cleanup() {
    echo ""
    echo "⚠️  收到终止信号，正在关闭所有服务..."
    
    # 杀掉 Python 后端
    if [ ! -z "$PID_CONTROL" ]; then
        echo "Killing Control Plane (PID $PID_CONTROL)..."
        kill $PID_CONTROL 2>/dev/null
    fi

    # 杀掉 Data Plane
    if [ ! -z "$PID_DATA" ]; then
        echo "Killing Data Plane (PID $PID_DATA)..."
        kill $PID_DATA 2>/dev/null
    fi

    # 杀掉 Media Plane
    if [ ! -z "$PID_MEDIA" ]; then
        echo "Killing Media Plane (PID $PID_MEDIA)..."
        kill $PID_MEDIA 2>/dev/null
    fi
    
    echo "✅ 所有服务已关闭"
    exit
}

# 捕获 SIGINT (Ctrl+C)
trap cleanup SIGINT

echo "🚀 启动后端服务器组件..."

# 1. 启动 Control Plane (Python)
echo "  -> Starting Control Plane (Port $PORT)..."
if [ -d "arm64/control_plane" ]; then
    (cd arm64/control_plane && python -m uvicorn app.main:app --host 0.0.0.0 --port $PORT > ../../control_plane.log 2>&1) &
    PID_CONTROL=$!
    echo "     PID: $PID_CONTROL"
else
    echo "⚠️  未找到 Control Plane 部署 (请先执行 ./build_all.sh)"
    # 尝试原地运行作为 fallback
    if [ -d "control_plane/app" ]; then
         echo "     Trying to run in-place..."
         python -m uvicorn app.main:app --host 0.0.0.0 --port $PORT > control_plane.log 2>&1 &
         PID_CONTROL=$!
    fi
fi

# 2. 启动 Data Plane (C++)
if [ -f "arm64/data_plane/data_plane_server" ]; then
    echo "  -> Starting Data Plane..."
    (cd arm64/data_plane && ./data_plane_server > ../../data_plane.log 2>&1) &
    PID_DATA=$!
    echo "     PID: $PID_DATA"
else
    echo "⚠️  未找到 Data Plane 可执行文件 (请先执行 ./build_all.sh)"
fi

# 3. 启动 Media Plane (C++)
if [ -f "arm64/media_plane/media_plane_server" ]; then
    echo "  -> Starting Media Plane..."
    (cd arm64/media_plane && ./media_plane_server > ../../media_plane.log 2>&1) &
    PID_MEDIA=$!
    echo "     PID: $PID_MEDIA"
else
    echo "⚠️  未找到 Media Plane 可执行文件 (请先执行 ./build_all.sh)"
fi

echo "✅ 所有服务已启动。查看日志: control_plane.log, data_plane.log, media_plane.log"
echo "按 Ctrl+C 停止所有服务"

# 等待任一进程退出
wait $PID_CONTROL $PID_DATA $PID_MEDIA
