#!/bin/bash
# Data Plane 快速编译脚本

set -e  # 遇到错误立即退出

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}  Data Plane 编译脚本${NC}"
echo -e "${GREEN}========================================${NC}"
echo ""

# 1. 检查 ROS2 环境
echo -e "${YELLOW}[1/5]${NC} 检查 ROS2 环境..."
if [ -z "$ROS_DISTRO" ]; then
    echo -e "${RED}错误: ROS2 环境未设置${NC}"
    echo "请先运行: source /opt/ros/humble/setup.bash"
    exit 1
fi
echo -e "${GREEN}✓${NC} ROS2 $ROS_DISTRO 已加载"

# 2. 检查自定义消息包
echo -e "${YELLOW}[2/5]${NC} 检查自定义消息包..."
REQUIRED_PKGS=("qyh_lift_msgs" "qyh_waist_msgs" "qyh_gripper_msgs" "qyh_standard_robot_msgs" "qyh_jaka_control_msgs" "qyh_task_engine_msgs")
MISSING_PKGS=()

for pkg in "${REQUIRED_PKGS[@]}"; do
    if ! echo $AMENT_PREFIX_PATH | grep -q "$pkg"; then
        MISSING_PKGS+=("$pkg")
    fi
done

if [ ${#MISSING_PKGS[@]} -ne 0 ]; then
    echo -e "${RED}错误: 以下消息包未找到:${NC}"
    printf '  - %s\n' "${MISSING_PKGS[@]}"
    echo ""
    echo "请先编译并 source workspace:"
    echo "  cd ~/qyh-robot-system/qyh_jushen_ws"
    echo "  colcon build"
    echo "  source install/setup.bash"
    exit 1
fi
echo -e "${GREEN}✓${NC} 所有消息包已加载"

# 3. 检查系统依赖
echo -e "${YELLOW}[3/5]${NC} 检查系统依赖..."
DEPS_OK=true

if ! dpkg -l | grep -q "libboost.*-dev"; then
    echo -e "${RED}✗${NC} Boost 未安装"
    DEPS_OK=false
else
    echo -e "${GREEN}✓${NC} Boost 已安装"
fi

if ! command -v protoc &> /dev/null; then
    echo -e "${RED}✗${NC} Protobuf 未安装"
    DEPS_OK=false
else
    PROTOC_VER=$(protoc --version | awk '{print $2}')
    echo -e "${GREEN}✓${NC} Protobuf $PROTOC_VER 已安装"
fi

if [ "$DEPS_OK" = false ]; then
    echo ""
    echo -e "${RED}依赖缺失！请安装:${NC}"
    echo "  sudo apt install libboost-all-dev libprotobuf-dev protobuf-compiler libyaml-cpp-dev nlohmann-json3-dev libssl-dev"
    exit 1
fi

# 4. 编译
echo -e "${YELLOW}[4/5]${NC} 配置 CMake..."
cd "$(dirname "$0")"
mkdir -p build && cd build
cmake .. || exit 1

echo -e "${YELLOW}[5/5]${NC} 编译项目（使用 $(nproc) 核心）..."
make -j$(nproc) || exit 1

echo ""
echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}  编译成功！${NC}"
echo -e "${GREEN}========================================${NC}"
echo ""
echo "可执行文件: $(pwd)/data_plane_server"
ls -lh data_plane_server 2>/dev/null || true
echo ""
