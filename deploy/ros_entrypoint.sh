#!/bin/bash
# ROS2 Entrypoint Script
# 用于在 Systemd 服务中加载 ROS2 环境

# 加载 ROS2 基础环境 (根据实际情况修改 distro)
if [ -f "/opt/ros/humble/setup.bash" ]; then
    source "/opt/ros/humble/setup.bash"
elif [ -f "/opt/ros/foxy/setup.bash" ]; then
    source "/opt/ros/foxy/setup.bash"
fi

# 加载自定义工作空间 (如果有)
if [ -f "/opt/qyh-robot/ws/install/setup.bash" ]; then
    source "/opt/qyh-robot/ws/install/setup.bash"
fi

# 执行传入的命令
exec "$@"
