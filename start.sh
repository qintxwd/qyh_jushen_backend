#!/bin/bash

# ==============================================================================
# QYH Robot Backend Startup Script
# ==============================================================================

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORK_DIR="$SCRIPT_DIR"
STOP_REQUESTED=0
ROBOT_ROOT="$(dirname "$SCRIPT_DIR")"

# Source ROS2 environment
if [ -f "/opt/ros/humble/setup.bash" ]; then
    source /opt/ros/humble/setup.bash
    echo "✅ 已加载 ROS2 Humble"
fi

# Source custom workspace (for qyh_lift_msgs etc.)
ROS2_WS="$ROBOT_ROOT/qyh_jushen_ws/install/setup.bash"
if [ -f "$ROS2_WS" ]; then
    source "$ROS2_WS"
    echo "✅ 已加载 ROS2 工作空间: $ROS2_WS"
else
    echo "⚠️  未找到 ROS2 工作空间: $ROS2_WS"
fi

export ROS_DOMAIN_ID=0
echo "🔧 ROS_DOMAIN_ID = $ROS_DOMAIN_ID"

# Cleanup function
cleanup() {
    STOP_REQUESTED=1
    echo ""
    echo "⚠️  正在关闭所有服务..."
    
    # Kill in reverse order
    if [ ! -z "$MEDIA_PID" ] && kill -0 $MEDIA_PID 2>/dev/null; then
        echo "Killing Media Plane (PID $MEDIA_PID)..."
        kill $MEDIA_PID 2>/dev/null || true
    fi
    
    if [ ! -z "$DATA_PID" ] && kill -0 $DATA_PID 2>/dev/null; then
        echo "Killing Data Plane (PID $DATA_PID)..."
        kill $DATA_PID 2>/dev/null || true
    fi
    
    if [ ! -z "$CONTROL_PID" ] && kill -0 $CONTROL_PID 2>/dev/null; then
        echo "Killing Control Plane (PID $CONTROL_PID)..."
        kill $CONTROL_PID 2>/dev/null || true
    fi
    
    wait 2>/dev/null
    echo "✅ 所有服务已关闭"
}

# Trap signals
trap cleanup SIGINT SIGTERM EXIT

echo "✅ 已加载全局配置: ROBOT_NAME=general, VERSION=1.0"

# Detect venv
VENV_PYTHON="$WORK_DIR/venv/bin/python"
if [ ! -f "$VENV_PYTHON" ]; then
    echo "⚠️  未找到虚拟环境 venv，尝试直接运行..."
    PYTHON_CMD="python3"
else
    PYTHON_CMD="$VENV_PYTHON"
fi

echo "🚀 启动后端服务器组件..."

# 1. Start Control Plane
echo "  -> Starting Control Plane (Port 8000)..."
cd "$WORK_DIR/arm64/control_plane"
$PYTHON_CMD -m uvicorn app.main:app --host 0.0.0.0 --port 8000 >> "$WORK_DIR/control_plane.log" 2>&1 &
CONTROL_PID=$!
echo "     PID: $CONTROL_PID"
cd "$WORK_DIR"

# Wait for Control Plane to start
sleep 3

# Check if Control Plane is still running
if ! kill -0 $CONTROL_PID 2>/dev/null; then
    echo "     ❌ Control Plane 启动失败！查看 control_plane.log"
    cat control_plane.log
    exit 1
fi
echo "     ✅ Control Plane 已启动"

# 2. Start Data Plane
echo "  -> Starting Data Plane..."
DATA_CONFIG="$WORK_DIR/arm64/data_plane/config/config.yaml"
if [ ! -f "$DATA_CONFIG" ]; then
    DATA_CONFIG="$WORK_DIR/data_plane/config/config.yaml"
fi
echo "     Config: $DATA_CONFIG"

cd "$WORK_DIR/arm64/data_plane"
if [ -f "./data_plane_server" ]; then
    ./data_plane_server "$DATA_CONFIG" >> "$WORK_DIR/data_plane.log" 2>&1 &
    DATA_PID=$!
    echo "     PID: $DATA_PID"
else
    echo "     ❌ 错误: 未找到 data_plane_server，请先运行 ./build_all.sh"
    exit 1
fi
cd "$WORK_DIR"

sleep 1
if ! kill -0 $DATA_PID 2>/dev/null; then
    echo "     ❌ Data Plane 启动失败！查看 data_plane.log"
    exit 1
fi
echo "     ✅ Data Plane 已启动"

# 3. Start Media Plane
echo "  -> Starting Media Plane..."
MEDIA_CONFIG="$WORK_DIR/arm64/media_plane/config/config.yaml"
if [ ! -f "$MEDIA_CONFIG" ]; then
     MEDIA_CONFIG="$WORK_DIR/media_plane/config/config.yaml"
fi
echo "     Config: $MEDIA_CONFIG"

cd "$WORK_DIR/arm64/media_plane"
if [ -f "./media_plane_server" ]; then
    ./media_plane_server "$MEDIA_CONFIG" >> "$WORK_DIR/media_plane.log" 2>&1 &
    MEDIA_PID=$!
    echo "     PID: $MEDIA_PID"
else
    echo "     ❌ 错误: 未找到 media_plane_server，请先运行 ./build_all.sh"
    exit 1
fi
cd "$WORK_DIR"

sleep 1
if ! kill -0 $MEDIA_PID 2>/dev/null; then
    echo "     ❌ Media Plane 启动失败！查看 media_plane.log"
    exit 1
fi
echo "     ✅ Media Plane 已启动"

echo ""
echo "✅ 所有服务已启动！"
echo "   - Control Plane: http://localhost:8000"
echo "   - Data Plane:    ws://localhost:8765"
echo "   - Media Plane:   ws://localhost:8888"
echo ""
echo "查看日志: control_plane.log, data_plane.log, media_plane.log"
echo "按 Ctrl+C 停止所有服务"

# Monitor processes and exit if any stops
while true; do
    sleep 1
    if [ "$STOP_REQUESTED" -eq 1 ]; then
        exit 0
    fi
    if ! kill -0 $CONTROL_PID 2>/dev/null; then
        echo "⚠️  检测到 Control Plane 退出，正在关闭所有服务..."
        exit 1
    fi
    if ! kill -0 $DATA_PID 2>/dev/null; then
        echo "⚠️  检测到 Data Plane 退出，正在关闭所有服务..."
        exit 1
    fi
    if ! kill -0 $MEDIA_PID 2>/dev/null; then
        echo "⚠️  检测到 Media Plane 退出，正在关闭所有服务..."
        exit 1
    fi
done
