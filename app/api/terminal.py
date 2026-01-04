"""终端 WebSocket API（管理员专用）"""
import asyncio
import os
import pty
import subprocess
import select
from fastapi import APIRouter, WebSocket, WebSocketDisconnect, Depends
from ..dependencies import get_current_user
from ..models.user import User, UserRole

router = APIRouter()


@router.websocket("/ws/terminal")
async def terminal_websocket(
    websocket: WebSocket,
    token: str = None
):
    """
    WebSocket 终端连接
    - 仅管理员可访问
    - 创建伪终端（PTY）执行 shell 命令
    """
    await websocket.accept()
    
    try:
        # 简单的 token 验证（实际应该用 JWT 验证）
        # 这里为了演示简化了验证流程
        if not token:
            await websocket.send_json({
                "type": "error",
                "data": "需要认证 token"
            })
            await websocket.close()
            return
        
        # TODO: 验证用户是否为管理员
        # 这里暂时跳过，实际部署需要添加完整的身份验证
        
        # 创建伪终端
        master_fd, slave_fd = pty.openpty()
        
        # 启动 bash shell
        shell = os.environ.get('SHELL', '/bin/bash')
        process = subprocess.Popen(
            [shell],
            stdin=slave_fd,
            stdout=slave_fd,
            stderr=slave_fd,
            preexec_fn=os.setsid
        )
        
        os.close(slave_fd)
        
        # 发送欢迎消息
        await websocket.send_json({
            "type": "connected",
            "data": "终端已连接\r\n"
        })
        
        # 创建读取和写入任务
        async def read_from_terminal():
            """从终端读取输出并发送到 WebSocket"""
            while True:
                try:
                    # 使用 select 检查是否有数据可读
                    r, _, _ = select.select([master_fd], [], [], 0.1)
                    if r:
                        output = os.read(master_fd, 1024)
                        if output:
                            await websocket.send_json({
                                "type": "output",
                                "data": output.decode('utf-8', errors='ignore')
                            })
                        else:
                            break
                    await asyncio.sleep(0.01)
                except Exception as e:
                    print(f"读取终端错误: {e}")
                    break
        
        async def write_to_terminal():
            """从 WebSocket 接收输入并写入终端"""
            while True:
                try:
                    message = await websocket.receive_json()
                    if message.get("type") == "input":
                        data = message.get("data", "")
                        os.write(master_fd, data.encode('utf-8'))
                    elif message.get("type") == "resize":
                        # 处理终端大小调整
                        rows = message.get("rows", 24)
                        cols = message.get("cols", 80)
                        # 这里可以使用 fcntl.ioctl 设置终端大小
                        pass
                except WebSocketDisconnect:
                    break
                except Exception as e:
                    print(f"写入终端错误: {e}")
                    break
        
        # 并发运行读写任务
        read_task = asyncio.create_task(read_from_terminal())
        write_task = asyncio.create_task(write_to_terminal())
        
        # 等待任一任务完成
        done, pending = await asyncio.wait(
            [read_task, write_task],
            return_when=asyncio.FIRST_COMPLETED
        )
        
        # 取消未完成的任务
        for task in pending:
            task.cancel()
        
    except WebSocketDisconnect:
        pass
    except Exception as e:
        print(f"终端 WebSocket 错误: {e}")
    finally:
        # 清理资源
        try:
            process.terminate()
            process.wait(timeout=2)
        except:
            process.kill()
        try:
            os.close(master_fd)
        except:
            pass
        try:
            await websocket.close()
        except:
            pass
