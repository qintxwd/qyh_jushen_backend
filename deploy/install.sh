#!/bin/bash
# QYH Robot Backend 安装脚本
# 在 Jetson Nano 上运行此脚本进行部署

set -e

# 颜色输出
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo -e "${GREEN}=== QYH Robot Backend 安装脚本 ===${NC}"

# 检查是否以 root 运行
if [ "$EUID" -ne 0 ]; then
    echo -e "${RED}请以 root 权限运行此脚本${NC}"
    exit 1
fi

# 安装目录
INSTALL_DIR="/opt/qyh-robot"
CONFIG_DIR="/etc/qyh-robot"
DATA_DIR="/var/lib/qyh-robot"
LOG_DIR="/var/log/qyh-robot"

# 获取脚本所在目录的上一级目录 (项目根目录)
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"

echo -e "${YELLOW}项目根目录: ${PROJECT_ROOT}${NC}"

# ==================== 创建用户 ====================
echo -e "${YELLOW}创建 robot 用户...${NC}"
if ! id "robot" &>/dev/null; then
    useradd -r -s /bin/false -d /opt/qyh-robot robot
    # 添加到必要的硬件访问组: video(摄像头), dialout(串口), input(手柄), i2c(传感器), gpio(IO)
    usermod -aG video,dialout,input,plugdev robot
    # 尝试添加 i2c/gpio 组 (如果存在)
    getent group i2c >/dev/null && usermod -aG i2c robot
    getent group gpio >/dev/null && usermod -aG gpio robot
fi

# ==================== 创建目录 ====================
echo -e "${YELLOW}创建目录...${NC}"
mkdir -p $INSTALL_DIR/{control_plane,data_plane,media_plane}
mkdir -p $CONFIG_DIR/{data_plane,media_plane}
mkdir -p $DATA_DIR
mkdir -p $LOG_DIR

# ==================== 安装系统依赖 ====================
echo -e "${YELLOW}安装系统依赖...${NC}"
apt-get update
apt-get install -y \
    python3.10 \
    python3.10-venv \
    python3-pip \
    libboost-all-dev \
    libprotobuf-dev \
    protobuf-compiler \
    libyaml-cpp-dev \
    libssl-dev \
    nlohmann-json3-dev \
    gstreamer1.0-tools \
    gstreamer1.0-plugins-base \
    gstreamer1.0-plugins-good \
    gstreamer1.0-plugins-bad \
    gstreamer1.0-plugins-ugly \
    gstreamer1.0-nice \
    gstreamer1.0-libav \
    libgstreamer1.0-dev \
    libgstreamer-plugins-base1.0-dev \
    libgstreamer-plugins-bad1.0-dev

# ==================== 部署 Control Plane ====================
echo -e "${YELLOW}部署 Control Plane...${NC}"
# 使用 PROJECT_ROOT 确保路径正确
cp -r ${PROJECT_ROOT}/control_plane/* $INSTALL_DIR/control_plane/

cd $INSTALL_DIR/control_plane
python3 -m venv .venv
source .venv/bin/activate
pip install --upgrade pip
pip install -r requirements.txt
deactivate

# ==================== 部署 Helper Scripts ====================
# 安装 entrypoint 脚本
cp ${SCRIPT_DIR}/ros_entrypoint.sh $INSTALL_DIR/
chmod +x $INSTALL_DIR/ros_entrypoint.sh

# ==================== 编译 Protobuf ====================
echo -e "${YELLOW}编译 Protobuf...${NC}"
mkdir -p $INSTALL_DIR/shared/proto_gen
# 切换到项目根目录执行 protoc，确保相对路径正确
cd ${PROJECT_ROOT}
protoc --proto_path=shared/proto \
    --cpp_out=$INSTALL_DIR/shared/proto_gen \
    --python_out=$INSTALL_DIR/control_plane/app \
    shared/proto/*.proto

# ==================== 编译 Data Plane ====================
echo -e "${YELLOW}编译 Data Plane...${NC}"
cd ${PROJECT_ROOT}/data_plane
# 清理旧 build
rm -rf build
mkdir -p build && cd build
# 使用 CMAKE_INSTALL_PREFIX 而不是 DESTDIR，确保路径结构扁平
cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=$INSTALL_DIR/data_plane
make -j$(nproc)
make install
# 注意: make install 后，二进制可能在 $INSTALL_DIR/data_plane/bin/

# ==================== 编译 Media Plane ====================
echo -e "${YELLOW}编译 Media Plane...${NC}"
cd ${PROJECT_ROOT}/media_plane
# 清理旧 build
rm -rf build
mkdir -p build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=$INSTALL_DIR/media_plane
make -j$(nproc)
make install

# ==================== 安装配置文件 ====================
echo -e "${YELLOW}安装配置文件...${NC}"
cp ${SCRIPT_DIR}/systemd/environment $CONFIG_DIR/
if [ -f "${PROJECT_ROOT}/data_plane/config/config.yaml" ]; then
    cp ${PROJECT_ROOT}/data_plane/config/config.yaml $CONFIG_DIR/data_plane/
fi
# Media Plane 可能没有默认 config.yaml，仅示例
if [ -f "${PROJECT_ROOT}/media_plane/config/config.yaml" ]; then
    cp ${PROJECT_ROOT}/media_plane/config/config.yaml $CONFIG_DIR/media_plane/
fi

# ==================== 安装 systemd 服务 ====================
echo -e "${YELLOW}安装 systemd 服务...${NC}"
cp ${SCRIPT_DIR}/systemd/*.service /etc/systemd/system/
systemctl daemon-reload

# ==================== 设置权限 ====================
echo -e "${YELLOW}设置权限...${NC}"
chown -R robot:robot $INSTALL_DIR
chown -R robot:robot $CONFIG_DIR
chown -R robot:robot $DATA_DIR
chown -R robot:robot $LOG_DIR

# 确保 entrypoint 可执行
chmod +x $INSTALL_DIR/ros_entrypoint.sh

# 保护凭据文件
chmod 600 $CONFIG_DIR/environment

# ==================== 启用服务 ====================
echo -e "${YELLOW}启用服务...${NC}"
systemctl enable qyh-control-plane.service
systemctl enable qyh-data-plane.service
systemctl enable qyh-media-plane.service

echo -e "${GREEN}=== 安装完成 ===${NC}"
echo ""
echo "启动服务:"
echo "  sudo systemctl start qyh-control-plane"
echo "  sudo systemctl start qyh-data-plane"
echo "  sudo systemctl start qyh-media-plane"
echo ""
echo "查看状态:"
echo "  sudo systemctl status qyh-control-plane"
echo "  sudo systemctl status qyh-data-plane"
echo "  sudo systemctl status qyh-media-plane"
echo ""
echo "查看日志:"
echo "  journalctl -u qyh-control-plane -f"
echo "  journalctl -u qyh-data-plane -f"
echo "  journalctl -u qyh-media-plane -f"
echo ""
echo -e "${YELLOW}注意: 请修改 /etc/qyh-robot/environment 中的 JWT_SECRET${NC}"
