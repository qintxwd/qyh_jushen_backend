"""任务执行引擎"""
import threading
import time
from typing import List, Dict, Any, Optional
from datetime import datetime
from sqlalchemy.orm import Session
from app.models.task import Task, TaskStatus


class TaskRunner:
    """任务执行引擎（服务端执行）"""
    
    def __init__(self):
        self.current_task_id: Optional[int] = None
        self.running = False
        self.thread: Optional[threading.Thread] = None
        self.should_stop = False
    
    def start_task(self, task_id: int, db: Session):
        """
        启动任务执行
        
        Args:
            task_id: 任务ID
            db: 数据库会话
        """
        if self.running:
            raise RuntimeError("已有任务正在运行")
        
        task = db.query(Task).filter(Task.id == task_id).first()
        if not task:
            raise ValueError("任务不存在")
        
        if task.status != TaskStatus.PENDING:
            raise ValueError("只能启动待执行的任务")
        
        self.current_task_id = task_id
        self.running = True
        self.should_stop = False
        
        # 更新任务状态
        task.status = TaskStatus.RUNNING
        task.started_at = datetime.utcnow()
        db.commit()
        
        # 在新线程中执行
        self.thread = threading.Thread(
            target=self._execute_task,
            args=(task_id,),
            daemon=True
        )
        self.thread.start()
    
    def stop_task(self):
        """停止当前任务"""
        self.should_stop = True
    
    def _execute_task(self, task_id: int):
        """
        执行任务（在独立线程）
        
        Args:
            task_id: 任务ID
        """
        from app.database import SessionLocal
        db = SessionLocal()
        
        try:
            task = db.query(Task).filter(Task.id == task_id).first()
            if not task:
                return
            
            program: List[Dict[str, Any]] = task.program
            total_steps = len(program)
            
            for index, action in enumerate(program):
                if self.should_stop:
                    task.status = TaskStatus.CANCELLED
                    break
                
                # 更新进度
                task.current_step = index + 1
                task.progress = (index + 1) / total_steps
                db.commit()
                
                # 执行动作
                try:
                    self._execute_action(action)
                except Exception as e:
                    print(f"❌ 执行动作失败: {action['type']}, 错误: {e}")
                    task.status = TaskStatus.FAILED
                    task.error_message = str(e)
                    break
            else:
                # 正常完成
                task.status = TaskStatus.COMPLETED
                task.completed_at = datetime.utcnow()
        
        except Exception as e:
            if task:
                task.status = TaskStatus.FAILED
                task.error_message = str(e)
            print(f"❌ 任务执行异常: {e}")
        
        finally:
            if task:
                db.commit()
            db.close()
            self.running = False
            self.current_task_id = None
    
    def _execute_action(self, action: Dict[str, Any]):
        """
        执行单个动作
        
        Args:
            action: 动作描述 {"type": "...", "params": {...}}
        """
        action_type = action['type']
        params = action['params']
        
        print(f"🤖 执行动作: {action_type}, 参数: {params}")
        
        # TODO: 调用 ROS2 Bridge 执行实际动作
        # from app.ros2_bridge import ros2_bridge
        # ros2_bridge.send_command(action)
        
        # 模拟执行时间
        if action_type == 'move_to':
            time.sleep(3.0)
        elif action_type == 'move_home':
            time.sleep(2.0)
        elif action_type in ['gripper_open', 'gripper_close']:
            time.sleep(1.0)
        elif action_type == 'wait':
            time.sleep(params.get('seconds', 1.0))
        else:
            time.sleep(0.5)
    
    def get_status(self) -> dict:
        """获取任务执行状态"""
        return {
            "running": self.running,
            "current_task_id": self.current_task_id
        }


# 全局单例
task_runner = TaskRunner()
