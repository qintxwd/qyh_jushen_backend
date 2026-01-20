#!/bin/bash
#
# QYH后端一体化编译部署脚本 (control_plane + data_plane + media_plane)
#

set -e

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DIST_DIR="${SCRIPT_DIR}/arm64" 

log_info() { echo -e "${GREEN}[INFO]${NC} $1"; }
log_warn() { echo -e "${YELLOW}[WARN]${NC} $1"; }
log_error() { echo -e "${RED}[ERROR]${NC} $1"; }

# 1. 编译 Data Plane
build_data_plane() {
    log_info "========================================"
    log_info "正在编译 Data Plane ..."
    log_info "========================================"
    cd "${SCRIPT_DIR}/data_plane"
    ./build.sh
    cd "${SCRIPT_DIR}"
}

# 2. 编译 Media Plane
build_media_plane() {
    log_info "========================================"
    log_info "正在编译 Media Plane ..."
    log_info "========================================"
    cd "${SCRIPT_DIR}/media_plane"
    ./build.sh
    cd "${SCRIPT_DIR}"
}

# 3. 部署到 arm64 目录
deploy() {
    log_info "========================================"
    log_info "正在部署到 ${DIST_DIR} ..."
    log_info "========================================"

    # 清理旧的 arm64 目录
    # rm -rf "${DIST_DIR}"  # 如果不想全删可以保留
    mkdir -p "${DIST_DIR}/data_plane"
    mkdir -p "${DIST_DIR}/media_plane"
    mkdir -p "${DIST_DIR}/control_plane"
    
    # 3.1 部署 Data Plane
    log_info "复制 Data Plane 文件..."
    DATA_SRC="${SCRIPT_DIR}/data_plane"
    DATA_DST="${DIST_DIR}/data_plane"
    
    # 复制可执行文件
    if [ -f "${DATA_SRC}/build/data_plane_server" ]; then
        cp "${DATA_SRC}/build/data_plane_server" "${DATA_DST}/"
        log_info "  -> data_plane_server"
    else
        log_error "未找到 data_plane_server，请检查编译是否成功"
        exit 1
    fi
    
    # 复制配置文件
    if [ -d "${DATA_SRC}/config" ]; then
        cp -r "${DATA_SRC}/config" "${DATA_DST}/"
        log_info "  -> config/"
    fi
    
    # 3.2 部署 Media Plane
    log_info "复制 Media Plane 文件..."
    MEDIA_SRC="${SCRIPT_DIR}/media_plane"
    MEDIA_DST="${DIST_DIR}/media_plane"
    
    # 复制可执行文件
    if [ -f "${MEDIA_SRC}/build/media_plane_server" ]; then
        cp "${MEDIA_SRC}/build/media_plane_server" "${MEDIA_DST}/"
        log_info "  -> media_plane_server"
    else
        log_error "未找到 media_plane_server，请检查编译是否成功"
        exit 1
    fi
    
    # 复制配置文件
    if [ -d "${MEDIA_SRC}/config" ]; then
        cp -r "${MEDIA_SRC}/config" "${MEDIA_DST}/"
        log_info "  -> config/"
    fi
    
    # 3.3 部署 Control Plane
    log_info "复制 Control Plane 文件..."
    CONTROL_SRC="${SCRIPT_DIR}/control_plane"
    CONTROL_DST="${DIST_DIR}/control_plane"
    
    mkdir -p "${CONTROL_DST}"
    
    # 复制 Python 源码及相关文件
    if [ -d "${CONTROL_SRC}/app" ]; then
        cp -r "${CONTROL_SRC}/app" "${CONTROL_DST}/"
        log_info "  -> app/"

        # 编译 Python 源码并清理 (保护源码)
        log_info "  -> 编译 Python 代码并移除源码..."
        
        # 清理可能存在的旧 __pycache__
        find "${CONTROL_DST}/app" -name "__pycache__" -type d -exec rm -rf {} +

        # 尝试使用 python3 或 python
        PY_CMD="python3"
        if ! command -v python3 &> /dev/null; then
            PY_CMD="python"
        fi
        
        $PY_CMD -m compileall "${CONTROL_DST}/app" >/dev/null
        
        # 移动 .pyc 并重命名 (flat structure)
        find "${CONTROL_DST}/app" -type d -name "__pycache__" | while read -r cache_dir; do
             for pyc in "$cache_dir"/*.pyc; do
                 if [ -f "$pyc" ]; then
                     filename=$(basename "$pyc")
                     # 替换规则: main.cpython-310.pyc -> main.pyc
                     new_name=$(echo "$filename" | sed -E 's/\.[^.]+\.pyc$/.pyc/')
                     mv "$pyc" "$(dirname "$cache_dir")/$new_name"
                 fi
             done
             rm -rf "$cache_dir"
        done
        
        # 删除源码
        find "${CONTROL_DST}/app" -name "*.py" -delete
        log_info "  -> 已移除 .py 源码文件，仅保留 bytecode"
    fi
    if [ -f "${CONTROL_SRC}/requirements.txt" ]; then
        cp "${CONTROL_SRC}/requirements.txt" "${CONTROL_DST}/"
        log_info "  -> requirements.txt"
    fi
    if [ -f "${CONTROL_SRC}/pyproject.toml" ]; then
        cp "${CONTROL_SRC}/pyproject.toml" "${CONTROL_DST}/"
        log_info "  -> pyproject.toml"
    fi
    
    log_info "部署完成: ${DIST_DIR}"
}

main() {
    build_data_plane
    build_media_plane
    deploy
    
    log_info "✅ 所有组件编译并部署成功！"
    log_info "使用 ./start.sh 启动系统"
}

main "$@"
