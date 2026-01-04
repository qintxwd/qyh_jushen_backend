"""Robot Model API - 提供URDF和模型文件"""

from fastapi import APIRouter, HTTPException
from fastapi.responses import FileResponse, JSONResponse
from pathlib import Path
import os
import subprocess
import re

from app.ros2_bridge.bridge import ros2_bridge
from app.safety.watchdog import watchdog

router = APIRouter()

# 获取用户主目录
HOME_DIR = Path.home()

# ROS2 工作空间路径 (部署环境使用 install 目录)
ROS2_WS = HOME_DIR / "qyh_jushen_ws" / "qyh_jushen_ws"

# 模型文件目录 (可配置) - 支持多个可能的路径
# 优先使用 install 目录 (部署环境)，其次使用 src 目录 (开发环境)
POSSIBLE_MESH_DIRS = [
    # install 目录 (部署环境) - 首选
    ROS2_WS / "install" / "qyh_dual_arms_description" / "share" / "qyh_dual_arms_description" / "meshes",
    # src 目录 (开发环境备用)
    ROS2_WS / "src" / "qyh_dual_arms_description" / "meshes",
    # 相对路径
    Path(__file__).parent.parent.parent.parent.parent / "qyh_jushen_ws" / "install" / "qyh_dual_arms_description" / "share" / "qyh_dual_arms_description" / "meshes",
]

# URDF 文件路径 - 同样优先 install 目录
POSSIBLE_URDF_DIRS = [
    # install 目录 (部署环境) - 首选
    ROS2_WS / "install" / "qyh_dual_arms_description" / "share" / "qyh_dual_arms_description" / "urdf",
    # src 目录 (开发环境备用)
    ROS2_WS / "src" / "qyh_dual_arms_description" / "urdf",
    # 相对路径
    Path(__file__).parent.parent.parent.parent.parent / "qyh_jushen_ws" / "install" / "qyh_dual_arms_description" / "share" / "qyh_dual_arms_description" / "urdf",
]

# ROS2 包路径映射表 (用于解析 package:// URL)
# 格式: { "package_name": Path("包的 share 目录") }
ROS2_PACKAGE_PATHS: dict[str, Path] = {}


def discover_ros2_packages():
    """
    发现 ROS2 工作空间中的所有包
    扫描 install 目录下的所有包并建立映射
    """
    global ROS2_PACKAGE_PATHS
    install_dir = ROS2_WS / "install"
    
    if not install_dir.exists():
        print(f"警告: install 目录不存在: {install_dir}")
        return
    
    for pkg_dir in install_dir.iterdir():
        if pkg_dir.is_dir() and not pkg_dir.name.startswith(('_', '.')):
            share_dir = pkg_dir / "share" / pkg_dir.name
            if share_dir.exists():
                ROS2_PACKAGE_PATHS[pkg_dir.name] = share_dir
                print(f"发现 ROS2 包: {pkg_dir.name} -> {share_dir}")


# 启动时发现包
discover_ros2_packages()

# 缓存处理后的 URDF
_cached_urdf: str | None = None
_cached_urdf_source: str = ""


def preprocess_urdf():
    """
    预处理 URDF 文件，缓存结果
    """
    global _cached_urdf, _cached_urdf_source
    
    urdf_dir = get_urdf_dir()
    xacro_file = urdf_dir / "dual_arms.urdf.xacro"
    
    if not xacro_file.exists():
        print(f"URDF 文件不存在: {xacro_file}")
        return
    
    # 尝试处理 xacro
    result = process_xacro_file(xacro_file)
    if result:
        _cached_urdf = result
        _cached_urdf_source = "xacro_processed"
        print("URDF 预处理成功 (xacro)")
        return
    
    # 手动处理
    try:
        with open(xacro_file, 'r') as f:
            content = f.read()
        _cached_urdf = manual_process_xacro(content, xacro_file.parent)
        _cached_urdf_source = "manual_processed"
        print("URDF 预处理成功 (手动处理)")
    except Exception as e:
        print(f"URDF 预处理失败: {e}")


# 从环境变量或自动检测
def get_meshes_dir() -> Path:
    env_path = os.environ.get("ROBOT_MESHES_DIR")
    if env_path:
        return Path(env_path)
    
    for path in POSSIBLE_MESH_DIRS:
        if path.exists():
            return path
    
    return POSSIBLE_MESH_DIRS[0]  # 默认返回第一个


def get_urdf_dir() -> Path:
    env_path = os.environ.get("ROBOT_URDF_DIR")
    if env_path:
        return Path(env_path)
    
    for path in POSSIBLE_URDF_DIRS:
        if path.exists():
            return path
    
    return POSSIBLE_URDF_DIRS[0]


MESHES_DIR = get_meshes_dir()
URDF_DIR = get_urdf_dir()

# JAKA Zu7 7-DOF 机械臂的DH参数和连杆配置
# 基于 jaka_zu7_macro_left.xacro 提取
ROBOT_CONFIG = {
    "name": "qyh_dual_arms",
    "base_link": "base_link",
    "arms": {
        "left": {
            "prefix": "left_",
            "base_origin": {"xyz": [-0.06, -0.06, 0], "rpy": [1.5708, 0, 2.356194]},
            "color": "#3498db",
            "joints": [
                {"name": "joint1", "type": "revolute", "origin": {"xyz": [0, 0, 0.217], "rpy": [0, 0, 0]}, "axis": [0, 0, 1], "limits": [-6.2832, 6.2832]},
                {"name": "joint2", "type": "revolute", "origin": {"xyz": [0, 0, 0.2075], "rpy": [-1.5708, 0, 0]}, "axis": [0, 0, 1], "limits": [-1.8325, 1.8325]},
                {"name": "joint3", "type": "revolute", "origin": {"xyz": [0, 0, 0], "rpy": [1.5708, 0, 0]}, "axis": [0, 0, 1], "limits": [-6.2832, 6.2832]},
                {"name": "joint4", "type": "revolute", "origin": {"xyz": [-0.00040027, 0, 0.33028], "rpy": [-1.5708, 0, 0]}, "axis": [0, 0, 1], "limits": [-2.5307, 0.5235]},
                {"name": "joint5", "type": "revolute", "origin": {"xyz": [0, 0, 0], "rpy": [1.5708, 0, 0]}, "axis": [0, 0, 1], "limits": [-6.2832, 6.2832]},
                {"name": "joint6", "type": "revolute", "origin": {"xyz": [-0.00036111, 0, 0.31546], "rpy": [-1.5708, 0, 0]}, "axis": [0, 0, 1], "limits": [-2.5307, 2.5307]},
                {"name": "joint7", "type": "revolute", "origin": {"xyz": [0, 0, 0], "rpy": [1.5708, 0, 0]}, "axis": [0, 0, 1], "limits": [-6.2832, 6.2832]},
            ],
            "links": [
                {"name": "base_link", "mesh": "left.stl", "visual_origin": {"xyz": [-0.01, -0.21, 0.2], "rpy": [-1.5708, 0, 0]}},
                {"name": "link1", "mesh": "l1.STL", "visual_origin": {"xyz": [0, 0, 0], "rpy": [0, 0, 0]}},
                {"name": "link2", "mesh": "l2.STL", "visual_origin": {"xyz": [0, 0, 0], "rpy": [0, 0, 0]}},
                {"name": "link3", "mesh": "l3.STL", "visual_origin": {"xyz": [0, 0, 0], "rpy": [0, 0, 0]}},
                {"name": "link4", "mesh": "l4.STL", "visual_origin": {"xyz": [0, 0, 0], "rpy": [0, 0, 0]}},
                {"name": "link5", "mesh": "l5.STL", "visual_origin": {"xyz": [0, 0, 0], "rpy": [0, 0, 0]}},
                {"name": "link6", "mesh": "l6.STL", "visual_origin": {"xyz": [0, 0, 0], "rpy": [0, 0, 0]}},
                {"name": "link7", "mesh": "l7.STL", "visual_origin": {"xyz": [0, 0, 0], "rpy": [0, 0, 0]}},
                {"name": "tool0", "mesh": "lt.STL", "visual_origin": {"xyz": [0, 0, 0], "rpy": [0, 0, 0]}},
            ]
        },
        "right": {
            "prefix": "right_",
            "base_origin": {"xyz": [-0.06, 0.06, 0], "rpy": [1.5708, 0, 0.785398]},
            "color": "#9b59b6",
            "joints": [
                {"name": "joint1", "type": "revolute", "origin": {"xyz": [0, 0, 0.217], "rpy": [0, 0, 0]}, "axis": [0, 0, 1], "limits": [-6.2832, 6.2832]},
                {"name": "joint2", "type": "revolute", "origin": {"xyz": [0, 0, 0.2075], "rpy": [1.5708, 0, 0]}, "axis": [0, 0, 1], "limits": [-1.8325, 1.8325]},
                {"name": "joint3", "type": "revolute", "origin": {"xyz": [0, 0, 0], "rpy": [-1.5708, 0, 0]}, "axis": [0, 0, 1], "limits": [-6.2832, 6.2832]},
                {"name": "joint4", "type": "revolute", "origin": {"xyz": [0.00040027, 0, 0.33028], "rpy": [1.5708, 0, 0]}, "axis": [0, 0, 1], "limits": [-0.5235, 2.5307]},
                {"name": "joint5", "type": "revolute", "origin": {"xyz": [0, 0, 0], "rpy": [-1.5708, 0, 0]}, "axis": [0, 0, 1], "limits": [-6.2832, 6.2832]},
                {"name": "joint6", "type": "revolute", "origin": {"xyz": [0.00036111, 0, 0.31546], "rpy": [1.5708, 0, 0]}, "axis": [0, 0, 1], "limits": [-2.5307, 2.5307]},
                {"name": "joint7", "type": "revolute", "origin": {"xyz": [0, 0, 0], "rpy": [-1.5708, 0, 0]}, "axis": [0, 0, 1], "limits": [-6.2832, 6.2832]},
            ],
            "links": [
                {"name": "base_link", "mesh": "right.stl", "visual_origin": {"xyz": [0.01, -0.21, 0.2], "rpy": [-1.5708, 0, 0]}},
                {"name": "link1", "mesh": "r1.STL", "visual_origin": {"xyz": [0, 0, 0], "rpy": [0, 0, 0]}},
                {"name": "link2", "mesh": "r2.STL", "visual_origin": {"xyz": [0, 0, 0], "rpy": [0, 0, 0]}},
                {"name": "link3", "mesh": "r3.STL", "visual_origin": {"xyz": [0, 0, 0], "rpy": [0, 0, 0]}},
                {"name": "link4", "mesh": "r4.STL", "visual_origin": {"xyz": [0, 0, 0], "rpy": [0, 0, 0]}},
                {"name": "link5", "mesh": "r5.STL", "visual_origin": {"xyz": [0, 0, 0], "rpy": [0, 0, 0]}},
                {"name": "link6", "mesh": "r6.STL", "visual_origin": {"xyz": [0, 0, 0], "rpy": [0, 0, 0]}},
                {"name": "link7", "mesh": "r7.STL", "visual_origin": {"xyz": [0, 0, 0], "rpy": [0, 0, 0]}},
                {"name": "tool0", "mesh": "rt.STL", "visual_origin": {"xyz": [0, 0, 0], "rpy": [0, 0, 0]}},
            ]
        }
    }
}


@router.get("/config")
async def get_robot_config():
    """获取机器人配置（关节、连杆信息）"""
    return JSONResponse(content=ROBOT_CONFIG)


@router.get("/meshes/{mesh_name}")
async def get_mesh_file(mesh_name: str):
    """获取 STL 模型文件"""
    mesh_path = MESHES_DIR / mesh_name
    
    if not mesh_path.exists():
        raise HTTPException(status_code=404, detail=f"Mesh file not found: {mesh_name}")
    
    return FileResponse(
        path=str(mesh_path),
        media_type="application/octet-stream",
        filename=mesh_name
    )


@router.get("/meshes")
async def list_meshes():
    """列出所有可用的模型文件"""
    if not MESHES_DIR.exists():
        return {"meshes": [], "error": f"Meshes directory not found: {MESHES_DIR}"}
    
    meshes = [f.name for f in MESHES_DIR.glob("*.stl")] + \
             [f.name for f in MESHES_DIR.glob("*.STL")]
    
    return {"meshes": meshes, "path": str(MESHES_DIR)}


@router.get("/joint_states")
async def get_joint_states():
    """获取当前关节状态（从ROS2获取或返回未连接状态）"""
    watchdog.heartbeat()
    
    # 尝试从 ROS2 获取真实关节状态
    if ros2_bridge.is_connected():
        joint_data = ros2_bridge.get_joint_states()
        if joint_data and joint_data.get("timestamp", 0) > 0:
            # 有真实的 ROS2 数据
            return {
                "timestamp": joint_data["timestamp"],
                "left": joint_data["left"],
                "right": joint_data["right"],
                "names": joint_data.get("names", {
                    "left": [f"left_joint{i+1}" for i in range(7)],
                    "right": [f"right_joint{i+1}" for i in range(7)]
                }),
                "source": "ros2"  # 标记为真实数据
            }
    
    # 未连接或 Mock 模式: 返回全 0，标记为 mock
    import time
    return {
        "timestamp": time.time(),
        "left": [0.0] * 7,
        "right": [0.0] * 7,
        "names": {
            "left": [f"left_joint{i+1}" for i in range(7)],
            "right": [f"right_joint{i+1}" for i in range(7)]
        },
        "source": "mock"  # 标记为未连接
    }


@router.get("/urdf")
async def get_robot_urdf():
    """
    获取机器人 URDF 描述
    优先从 ROS2 /robot_description 话题获取
    否则返回处理后的 URDF 文件
    
    重要: 会将 package:// 路径替换为后端 API 可访问的 URL
    """
    global _cached_urdf, _cached_urdf_source
    watchdog.heartbeat()
    
    urdf_content = None
    source = "unknown"
    
    # 优先级1: 从 ROS2 获取 (最佳方式，已被 robot_state_publisher 处理)
    if ros2_bridge.is_connected():
        urdf_content = ros2_bridge.get_robot_description()
        if urdf_content:
            source = "ros2"
    
    # 优先级2: 使用缓存的预处理结果
    if not urdf_content and _cached_urdf:
        urdf_content = _cached_urdf
        source = _cached_urdf_source + "_cached"
    
    # 优先级3: 实时处理 xacro
    if not urdf_content:
        urdf_file = URDF_DIR / "dual_arms.urdf.xacro"
        if not urdf_file.exists():
            raise HTTPException(
                status_code=404,
                detail=f"URDF file not found: {urdf_file}"
            )
        
        # 尝试多种方式处理 xacro
        urdf_content = process_xacro_file(urdf_file)
        if urdf_content:
            source = "xacro_processed"
            # 缓存结果
            _cached_urdf = urdf_content
            _cached_urdf_source = source
        else:
            # 最后的备选：读取原始文件并手动处理
            with open(urdf_file, 'r') as f:
                urdf_content = f.read()
            # 手动处理 $(find xxx) 和 xacro:include
            urdf_content = manual_process_xacro(
                urdf_content, urdf_file.parent
            )
            source = "manual_processed"
            # 缓存结果
            _cached_urdf = urdf_content
            _cached_urdf_source = source
    
    # 处理 package:// 路径，替换为后端 API URL
    processed_urdf = process_package_urls(urdf_content)
    
    return {
        "urdf": processed_urdf,
        "source": source,
        "package_paths": {k: str(v) for k, v in ROS2_PACKAGE_PATHS.items()}
    }


def process_xacro_file(xacro_file: Path) -> str | None:
    """
    尝试使用 xacro 命令处理 xacro 文件
    会尝试多种方式调用 xacro
    """
    # 方法1: 直接调用 xacro (如果已在 ROS 环境中)
    try:
        result = subprocess.run(
            ["xacro", str(xacro_file)],
            capture_output=True,
            text=True,
            timeout=10
        )
        if result.returncode == 0 and result.stdout.strip():
            print(f"xacro 直接调用成功")
            return result.stdout
        else:
            print(f"xacro 调用失败: {result.stderr}")
    except FileNotFoundError:
        print("xacro 命令不存在，尝试其他方式")
    except Exception as e:
        print(f"xacro 调用异常: {e}")
    
    # 方法2: 通过 bash 调用并 source ROS 环境
    ros_setup_scripts = [
        "/opt/ros/humble/setup.bash",
        "/opt/ros/galactic/setup.bash",
        "/opt/ros/foxy/setup.bash",
    ]
    
    install_setup = ROS2_WS / "install" / "setup.bash"
    
    for ros_setup in ros_setup_scripts:
        if Path(ros_setup).exists():
            try:
                # 构建命令：source ROS + source 工作空间 + xacro
                cmd_parts = [f"source {ros_setup}"]
                if install_setup.exists():
                    cmd_parts.append(f"source {install_setup}")
                cmd_parts.append(f"xacro {xacro_file}")
                
                full_cmd = " && ".join(cmd_parts)
                
                result = subprocess.run(
                    ["bash", "-c", full_cmd],
                    capture_output=True,
                    text=True,
                    timeout=15
                )
                if result.returncode == 0 and result.stdout.strip():
                    print(f"通过 bash 调用 xacro 成功 (使用 {ros_setup})")
                    return result.stdout
            except Exception as e:
                print(f"bash xacro 调用失败: {e}")
    
    return None


def manual_process_xacro(content: str, base_dir: Path) -> str:
    """
    手动处理 xacro 文件（当 xacro 命令不可用时的备选方案）
    
    处理:
    1. $(find package_name) -> 实际路径
    2. xacro:include -> 展开包含的文件
    
    注意：这不是完整的 xacro 处理器，只处理常见情况
    """
    # 处理 $(find package_name)
    def replace_find(match):
        package_name = match.group(1)
        # 优先从已发现的包路径中查找
        if package_name in ROS2_PACKAGE_PATHS:
            return str(ROS2_PACKAGE_PATHS[package_name])
        # 尝试从 install 目录查找
        install_path = ROS2_WS / "install" / package_name / "share" / package_name
        if install_path.exists():
            return str(install_path)
        # 尝试从 src 目录查找
        src_path = ROS2_WS / "src" / package_name
        if src_path.exists():
            return str(src_path)
        # 返回原始内容
        print(f"警告: 无法解析包 {package_name}")
        return match.group(0)
    
    content = re.sub(r'\$\(find\s+([^)]+)\)', replace_find, content)
    
    # 处理 xacro:include
    include_pattern = r'<xacro:include\s+filename="([^"]+)"\s*/>'
    
    def expand_include(match):
        include_path = match.group(1)
        # 解析路径
        if include_path.startswith('/'):
            full_path = Path(include_path)
        else:
            full_path = base_dir / include_path
        
        if full_path.exists():
            try:
                with open(full_path, 'r') as f:
                    included_content = f.read()
                # 递归处理包含的文件
                included_content = manual_process_xacro(
                    included_content, full_path.parent
                )
                # 移除 XML 声明和 robot 标签
                included_content = re.sub(
                    r'<\?xml[^?]*\?>', '', included_content
                )
                included_content = re.sub(
                    r'<robot[^>]*>', '', included_content
                )
                included_content = re.sub(
                    r'</robot>', '', included_content
                )
                return included_content.strip()
            except Exception as e:
                print(f"读取包含文件失败 {full_path}: {e}")
        else:
            print(f"包含文件不存在: {full_path}")
        
        return f"<!-- Include failed: {include_path} -->"
    
    # 展开所有 include
    content = re.sub(include_pattern, expand_include, content)
    
    # 移除 xacro 命名空间声明（简化处理）
    content = re.sub(r'xmlns:xacro="[^"]*"', '', content)
    
    return content


def process_package_urls(urdf_content: str) -> str:
    """
    将 URDF 中的 package:// URL 替换为后端 API 可访问的路径
    
    例如:
    package://qyh_dual_arms_description/meshes/left.stl
    -> /api/v1/robot-model/package/qyh_dual_arms_description/meshes/left.stl
    
    这样前端可以通过后端 API 访问这些资源，无需 ROS 环境
    """
    def replace_package_url(match):
        full_url = match.group(0)
        # 提取 package://package_name/path
        package_pattern = r'package://([^/]+)/(.+)'
        pkg_match = re.match(package_pattern, full_url)
        if pkg_match:
            package_name = pkg_match.group(1)
            resource_path = pkg_match.group(2)
            # 返回后端 API 路径
            api_path = f"/api/v1/robot-model/package/{package_name}"
            return f"{api_path}/{resource_path}"
        return full_url
    
    # 匹配 package://xxx/xxx 格式的 URL
    pattern = r'package://[^\s"<>]+'
    processed = re.sub(pattern, replace_package_url, urdf_content)
    
    return processed


@router.get("/package/{package_name}/{resource_path:path}")
async def get_package_resource(package_name: str, resource_path: str):
    """
    获取 ROS2 包中的资源文件
    
    用于解析 package:// URL，使前端无需 ROS 环境也能访问模型文件
    
    Args:
        package_name: ROS2 包名，如 qyh_dual_arms_description
        resource_path: 包内资源路径，如 meshes/left.stl
    """
    # 先检查包路径映射
    if package_name in ROS2_PACKAGE_PATHS:
        file_path = ROS2_PACKAGE_PATHS[package_name] / resource_path
        if file_path.exists():
            # 根据文件扩展名设置 MIME 类型
            ext = file_path.suffix.lower()
            media_types = {
                '.stl': 'application/octet-stream',
                '.dae': 'model/vnd.collada+xml',
                '.obj': 'text/plain',
                '.urdf': 'application/xml',
                '.xacro': 'application/xml',
            }
            media_type = media_types.get(ext, 'application/octet-stream')
            
            return FileResponse(
                path=str(file_path),
                media_type=media_type,
                filename=file_path.name
            )
    
    # 如果包路径映射中没有，尝试从可能的目录中查找
    possible_paths = [
        ROS2_WS / "install" / package_name / "share" / package_name / resource_path,
        ROS2_WS / "src" / package_name / resource_path,
    ]
    
    for path in possible_paths:
        if path.exists():
            ext = path.suffix.lower()
            media_types = {
                '.stl': 'application/octet-stream',
                '.dae': 'model/vnd.collada+xml',
                '.obj': 'text/plain',
                '.urdf': 'application/xml',
                '.xacro': 'application/xml',
            }
            media_type = media_types.get(ext, 'application/octet-stream')
            
            return FileResponse(
                path=str(path),
                media_type=media_type,
                filename=path.name
            )
    
    raise HTTPException(
        status_code=404,
        detail=f"Resource not found: {package_name}/{resource_path}"
    )


@router.get("/debug/paths")
async def debug_paths():
    """
    调试端点：查看当前路径配置
    """
    return {
        "home_dir": str(HOME_DIR),
        "ros2_ws": str(ROS2_WS),
        "ros2_ws_exists": ROS2_WS.exists(),
        "meshes_dir": str(MESHES_DIR),
        "meshes_dir_exists": MESHES_DIR.exists(),
        "urdf_dir": str(URDF_DIR),
        "urdf_dir_exists": URDF_DIR.exists(),
        "package_paths": {k: str(v) for k, v in ROS2_PACKAGE_PATHS.items()},
        "possible_mesh_dirs": [
            {"path": str(p), "exists": p.exists()}
            for p in POSSIBLE_MESH_DIRS
        ],
        "possible_urdf_dirs": [
            {"path": str(p), "exists": p.exists()}
            for p in POSSIBLE_URDF_DIRS
        ],
    }


@router.post("/debug/refresh-packages")
async def refresh_packages():
    """
    刷新 ROS2 包路径映射
    """
    discover_ros2_packages()
    return {
        "message": "Package paths refreshed",
        "package_paths": {k: str(v) for k, v in ROS2_PACKAGE_PATHS.items()}
    }


@router.post("/debug/refresh-urdf")
async def refresh_urdf():
    """
    刷新 URDF 缓存，重新处理 xacro 文件
    """
    global _cached_urdf, _cached_urdf_source
    _cached_urdf = None
    _cached_urdf_source = ""
    
    # 重新处理
    urdf_file = URDF_DIR / "dual_arms.urdf.xacro"
    if urdf_file.exists():
        result = process_xacro_file(urdf_file)
        if result:
            _cached_urdf = result
            _cached_urdf_source = "xacro_processed"
        else:
            with open(urdf_file, 'r') as f:
                content = f.read()
            _cached_urdf = manual_process_xacro(content, urdf_file.parent)
            _cached_urdf_source = "manual_processed"
    
    return {
        "message": "URDF cache refreshed",
        "source": _cached_urdf_source,
        "urdf_length": len(_cached_urdf) if _cached_urdf else 0,
        "urdf_preview": _cached_urdf[:500] if _cached_urdf else None
    }


@router.get("/debug/urdf-raw")
async def debug_urdf_raw():
    """
    获取处理后的原始 URDF（未转换 package:// 路径）
    用于调试 xacro 处理是否正确
    """
    if _cached_urdf:
        return {
            "source": _cached_urdf_source,
            "urdf": _cached_urdf
        }
    
    # 尝试获取
    urdf_file = URDF_DIR / "dual_arms.urdf.xacro"
    if not urdf_file.exists():
        return {"error": f"URDF file not found: {urdf_file}"}
    
    result = process_xacro_file(urdf_file)
    if result:
        return {"source": "xacro", "urdf": result}
    
    with open(urdf_file, 'r') as f:
        content = f.read()
    processed = manual_process_xacro(content, urdf_file.parent)
    return {"source": "manual", "urdf": processed}
