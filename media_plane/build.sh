#!/bin/bash
#
# media_plane 编译脚本
#

set -e

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BUILD_DIR="${SCRIPT_DIR}/build"

log_info() { echo -e "${GREEN}[INFO]${NC} $1"; }
log_warn() { echo -e "${YELLOW}[WARN]${NC} $1"; }
log_error() { echo -e "${RED}[ERROR]${NC} $1"; }

check_ros2() {
    if [ -z "$ROS_DISTRO" ]; then
        log_warn "ROS2 未配置，尝试自动配置..."
        for setup in /opt/ros/*/setup.bash; do
            [ -f "$setup" ] && source "$setup" && break
        done
        [ -z "$ROS_DISTRO" ] && log_error "未找到 ROS2" && exit 1
    fi
    log_info "ROS2: $ROS_DISTRO"
}

check_deps() {
    log_info "检查依赖..."
    pkg-config --exists gstreamer-1.0 || { log_error "需要 GStreamer"; exit 1; }
    log_info "GStreamer: $(pkg-config --modversion gstreamer-1.0)"
}

main() {
    log_info "media_plane 编译脚本"
    
    [ "$1" = "clean" ] && rm -rf "$BUILD_DIR"
    
    check_ros2
    check_deps
    
    mkdir -p "$BUILD_DIR" && cd "$BUILD_DIR"
    
    log_info "CMake 配置..."
    cmake .. || exit 1
    
    NPROC=$(nproc 2>/dev/null || echo 2)
    log_info "Make (${NPROC} 线程)..."
    make -j"$NPROC" || exit 1
    
    [ -f media_plane_server ] && {
        SIZE=$(du -h media_plane_server | cut -f1)
        log_info "✅ 编译成功: ${BUILD_DIR}/media_plane_server (${SIZE})"
    } || {
        log_error "未找到可执行文件"
        exit 1
    }
}

main "$@"
