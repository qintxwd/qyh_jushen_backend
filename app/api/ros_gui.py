"""ROS GUI 管理 API（管理员专用）"""
import asyncio
import subprocess
import signal
import os
from typing import Dict, Optional
from fastapi import APIRouter, Depends, HTTPException
from ..dependencies import get_current_admin
from ..models.user import User

router = APIRouter()

# 存储运行中的 GUI 进程
gui_processes: Dict[str, Optional[subprocess.Popen]] = {
    'gripper': None,
    'jaka': None,
    'chassis': None
}

# ROS GUI 启动命令
GUI_COMMANDS = {
    'gripper': ['ros2', 'run', 'qyh_gripper_gui', 'gripper_gui'],
    'jaka': ['ros2', 'run', 'qyh_jaka_control_gui', 'jaka_gui'],
    'chassis': ['ros2', 'run', 'qyh_standard_robot_gui', 'chassis_gui']
}

GUI_NAMES = {
    'gripper': '夹爪控制',
    'jaka': 'JAKA机械臂控制',
    'chassis': 'Standard底盘控制'
}


def is_gui_running(gui_type: str) -> bool:
    """检查 GUI 是否正在运行"""
    process = gui_processes.get(gui_type)
    if process is None:
        return False
    
    # 检查进程是否还在运行
    if process.poll() is None:
        return True
    else:
        # 进程已结束，清理
        gui_processes[gui_type] = None
        return False


@router.get("/status")
async def get_gui_status(current_user: User = Depends(get_current_admin)):
    """
    获取所有 ROS GUI 的运行状态
    仅管理员可访问
    """
    return {
        'gripper': 'running' if is_gui_running('gripper') else 'stopped',
        'jaka': 'running' if is_gui_running('jaka') else 'stopped',
        'chassis': 'running' if is_gui_running('chassis') else 'stopped'
    }


@router.post("/{gui_type}/start")
async def start_gui(
    gui_type: str,
    current_user: User = Depends(get_current_admin)
):
    """
    启动指定的 ROS GUI
    
    参数:
        gui_type: GUI 类型 (gripper/jaka/chassis)
    """
    if gui_type not in GUI_COMMANDS:
        raise HTTPException(status_code=400, detail=f"未知的 GUI 类型: {gui_type}")
    
    # 检查是否已经在运行
    if is_gui_running(gui_type):
        return {
            'success': False,
            'message': f'{GUI_NAMES[gui_type]} GUI 已在运行中'
        }
    
    try:
        # 设置环境变量（确保 ROS2 环境已配置）
        env = os.environ.copy()
        
        # 如果需要，可以添加额外的 ROS2 环境变量
        # env['ROS_DOMAIN_ID'] = '0'
        
        # 启动 GUI 进程
        process = subprocess.Popen(
            GUI_COMMANDS[gui_type],
            env=env,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            start_new_session=True  # 创建新的进程组
        )
        
        gui_processes[gui_type] = process
        
        # 等待一小段时间确保进程启动成功
        await asyncio.sleep(0.5)
        
        if process.poll() is not None:
            # 进程已经退出，说明启动失败
            stdout, stderr = process.communicate()
            error_msg = stderr.decode('utf-8', errors='ignore') if stderr else '未知错误'
            raise Exception(f"GUI 启动失败: {error_msg}")
        
        return {
            'success': True,
            'message': f'{GUI_NAMES[gui_type]} GUI 启动成功',
            'pid': process.pid
        }
        
    except FileNotFoundError:
        raise HTTPException(
            status_code=500,
            detail=f"未找到 ROS2 命令，请确保已正确安装 ROS2 和 {gui_type} 包"
        )
    except Exception as e:
        raise HTTPException(
            status_code=500,
            detail=f"启动 {GUI_NAMES[gui_type]} GUI 失败: {str(e)}"
        )


@router.post("/{gui_type}/stop")
async def stop_gui(
    gui_type: str,
    current_user: User = Depends(get_current_admin)
):
    """
    停止指定的 ROS GUI
    
    参数:
        gui_type: GUI 类型 (gripper/jaka/chassis)
    """
    if gui_type not in GUI_COMMANDS:
        raise HTTPException(status_code=400, detail=f"未知的 GUI 类型: {gui_type}")
    
    process = gui_processes.get(gui_type)
    
    if process is None or process.poll() is not None:
        gui_processes[gui_type] = None
        return {
            'success': False,
            'message': f'{GUI_NAMES[gui_type]} GUI 未在运行'
        }
    
    try:
        # 发送 SIGTERM 信号优雅关闭
        os.killpg(os.getpgid(process.pid), signal.SIGTERM)
        
        # 等待进程结束（最多等待5秒）
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            # 如果5秒后还没结束，强制杀死
            os.killpg(os.getpgid(process.pid), signal.SIGKILL)
            process.wait()
        
        gui_processes[gui_type] = None
        
        return {
            'success': True,
            'message': f'{GUI_NAMES[gui_type]} GUI 已停止'
        }
        
    except Exception as e:
        raise HTTPException(
            status_code=500,
            detail=f"停止 {GUI_NAMES[gui_type]} GUI 失败: {str(e)}"
        )


@router.post("/stop-all")
async def stop_all_guis(current_user: User = Depends(get_current_admin)):
    """
    停止所有 ROS GUI
    """
    results = []
    
    for gui_type in GUI_COMMANDS.keys():
        try:
            if is_gui_running(gui_type):
                result = await stop_gui(gui_type, current_user)
                results.append({
                    'gui_type': gui_type,
                    'success': result['success'],
                    'message': result['message']
                })
        except Exception as e:
            results.append({
                'gui_type': gui_type,
                'success': False,
                'message': str(e)
            })
    
    return {
        'success': True,
        'message': '批量停止完成',
        'results': results
    }


@router.get("/{gui_type}/logs")
async def get_gui_logs(
    gui_type: str,
    lines: int = 50,
    current_user: User = Depends(get_current_admin)
):
    """
    获取 GUI 的日志输出
    
    参数:
        gui_type: GUI 类型
        lines: 返回的行数（默认50行）
    """
    if gui_type not in GUI_COMMANDS:
        raise HTTPException(status_code=400, detail=f"未知的 GUI 类型: {gui_type}")
    
    process = gui_processes.get(gui_type)
    
    if process is None or process.poll() is not None:
        return {
            'success': False,
            'message': f'{GUI_NAMES[gui_type]} GUI 未在运行',
            'logs': ''
        }
    
    try:
        # 读取进程的标准输出和错误输出
        # 注意：这里只能读取启动时缓冲的内容
        # 实际生产环境应该使用日志文件
        stdout = process.stdout.read() if process.stdout else b''
        stderr = process.stderr.read() if process.stderr else b''
        
        logs = stdout.decode('utf-8', errors='ignore') + '\n' + stderr.decode('utf-8', errors='ignore')
        
        # 只返回最后 N 行
        log_lines = logs.split('\n')
        recent_logs = '\n'.join(log_lines[-lines:])
        
        return {
            'success': True,
            'logs': recent_logs
        }
        
    except Exception as e:
        raise HTTPException(
            status_code=500,
            detail=f"读取日志失败: {str(e)}"
        )
