"""任务引擎组件"""
from typing import Optional, Dict, Any, List
from .base import BridgeComponent


class TaskEngineComponent(BridgeComponent):
    """任务引擎组件
    
    功能:
    - 任务状态订阅
    - 任务执行、暂停、恢复、取消
    """
    
    def __init__(self):
        super().__init__()
        self.task_status: Optional[Dict[str, Any]] = None
        
        # 服务客户端
        self.execute_client = None
        self.pause_client = None
        self.resume_client = None
        self.cancel_client = None
        self.status_client = None
    
    def setup_subscribers(self):
        """设置任务状态订阅器"""
        try:
            from qyh_task_engine_msgs.msg import TaskStatus
            
            def callback(msg):
                node_statuses = []
                for ns in msg.node_statuses:
                    node_statuses.append({
                        "node_id": ns.node_id,
                        "node_type": ns.node_type,
                        "node_name": ns.node_name,
                        "status": ns.status,
                        "message": ns.message,
                        "duration": ns.duration,
                        "children_count": ns.children_count,
                        "current_child_index": ns.current_child_index,
                        "current_iteration": ns.current_iteration,
                        "total_iterations": ns.total_iterations
                    })
                
                self.task_status = {
                    "task_id": msg.task_id,
                    "task_name": msg.task_name,
                    "status": msg.status,
                    "progress": msg.progress,
                    "current_node_id": msg.current_node_id,
                    "completed_nodes": msg.completed_nodes,
                    "total_nodes": msg.total_nodes,
                    "message": msg.message,
                    "elapsed_time": msg.elapsed_time,
                    "node_statuses": node_statuses
                }
            
            self.node.create_subscription(TaskStatus, '/task_engine/status', callback, 10)
            print("✅ 任务状态订阅器创建成功: /task_engine/status")
        except Exception as e:
            print(f"⚠️  任务状态订阅器创建失败 (可能未安装 qyh_task_engine_msgs): {e}")
    
    def setup_publishers(self):
        """设置任务引擎服务客户端"""
        try:
            from qyh_task_engine_msgs.srv import (
                ExecuteTask, PauseTask, ResumeTask, CancelTask, GetTaskStatus
            )
            
            self.execute_client = self.node.create_client(ExecuteTask, '/task_engine/execute')
            self.pause_client = self.node.create_client(PauseTask, '/task_engine/pause')
            self.resume_client = self.node.create_client(ResumeTask, '/task_engine/resume')
            self.cancel_client = self.node.create_client(CancelTask, '/task_engine/cancel')
            self.status_client = self.node.create_client(GetTaskStatus, '/task_engine/get_status')
            
            print("✅ 任务引擎服务客户端创建成功")
        except Exception as e:
            print(f"⚠️  任务引擎服务客户端创建失败: {e}")
    
    def handle_command(self, cmd: Dict[str, Any]) -> bool:
        """处理任务引擎命令"""
        cmd_type = cmd.get('type', '')
        
        handlers = {
            'task_execute': self._handle_execute,
            'task_pause': self._handle_pause,
            'task_resume': self._handle_resume,
            'task_cancel': self._handle_cancel,
            'task_get_status': self._handle_get_status,
        }
        
        handler = handlers.get(cmd_type)
        if handler:
            handler(cmd)
            return True
        return False
    
    def _handle_execute(self, cmd: Dict[str, Any]):
        """处理任务执行"""
        if not self.execute_client:
            self._send_result(cmd, {'success': False, 'message': '任务引擎客户端未初始化'})
            return
        
        if not self.execute_client.wait_for_service(timeout_sec=2.0):
            self._send_result(cmd, {'success': False, 'message': '任务引擎服务不可用'})
            return
        
        from qyh_task_engine_msgs.srv import ExecuteTask
        params = cmd['params']
        req = ExecuteTask.Request()
        req.task_json = params['task_json']
        
        future = self.execute_client.call_async(req)
        
        def done_callback(f):
            try:
                resp = f.result()
                result = {
                    'success': resp.success,
                    'task_id': resp.task_id,
                    'message': resp.message
                }
            except Exception as e:
                result = {'success': False, 'task_id': '', 'message': str(e)}
            self._send_result(cmd, result)
        
        future.add_done_callback(done_callback)
    
    def _handle_pause(self, cmd: Dict[str, Any]):
        """处理任务暂停"""
        if not self.pause_client:
            self._send_result(cmd, {'success': False, 'message': '任务引擎客户端未初始化'})
            return
        
        if not self.pause_client.wait_for_service(timeout_sec=1.0):
            self._send_result(cmd, {'success': False, 'message': '任务暂停服务不可用'})
            return
        
        from qyh_task_engine_msgs.srv import PauseTask
        params = cmd['params']
        req = PauseTask.Request()
        req.task_id = params['task_id']
        
        future = self.pause_client.call_async(req)
        future.add_done_callback(lambda f: self._handle_simple_response(cmd, f))
    
    def _handle_resume(self, cmd: Dict[str, Any]):
        """处理任务恢复"""
        if not self.resume_client:
            self._send_result(cmd, {'success': False, 'message': '任务引擎客户端未初始化'})
            return
        
        if not self.resume_client.wait_for_service(timeout_sec=1.0):
            self._send_result(cmd, {'success': False, 'message': '任务恢复服务不可用'})
            return
        
        from qyh_task_engine_msgs.srv import ResumeTask
        params = cmd['params']
        req = ResumeTask.Request()
        req.task_id = params['task_id']
        
        future = self.resume_client.call_async(req)
        future.add_done_callback(lambda f: self._handle_simple_response(cmd, f))
    
    def _handle_cancel(self, cmd: Dict[str, Any]):
        """处理任务取消"""
        if not self.cancel_client:
            self._send_result(cmd, {'success': False, 'message': '任务引擎客户端未初始化'})
            return
        
        if not self.cancel_client.wait_for_service(timeout_sec=1.0):
            self._send_result(cmd, {'success': False, 'message': '任务取消服务不可用'})
            return
        
        from qyh_task_engine_msgs.srv import CancelTask
        params = cmd['params']
        req = CancelTask.Request()
        req.task_id = params['task_id']
        
        future = self.cancel_client.call_async(req)
        future.add_done_callback(lambda f: self._handle_simple_response(cmd, f))
    
    def _handle_get_status(self, cmd: Dict[str, Any]):
        """处理任务状态查询"""
        if not self.status_client:
            self._send_result(cmd, {'success': False, 'message': '任务引擎客户端未初始化'})
            return
        
        if not self.status_client.wait_for_service(timeout_sec=1.0):
            self._send_result(cmd, {'success': False, 'message': '任务状态服务不可用'})
            return
        
        from qyh_task_engine_msgs.srv import GetTaskStatus
        params = cmd['params']
        req = GetTaskStatus.Request()
        req.task_id = params['task_id']
        
        future = self.status_client.call_async(req)
        
        def done_callback(f):
            try:
                resp = f.result()
                result = {
                    'success': resp.success,
                    'task_id': resp.status.task_id,
                    'status': resp.status.status,
                    'progress': resp.status.progress,
                    'current_node': resp.status.current_node,
                    'message': resp.status.message,
                    'elapsed_time': resp.status.elapsed_time
                }
            except Exception as e:
                result = {'success': False, 'message': str(e)}
            self._send_result(cmd, result)
        
        future.add_done_callback(done_callback)
    
    def _handle_simple_response(self, cmd: Dict[str, Any], future):
        """处理简单服务响应"""
        try:
            resp = future.result()
            result = {'success': resp.success, 'message': resp.message}
        except Exception as e:
            result = {'success': False, 'message': str(e)}
        self._send_result(cmd, result)
    
    # ==================== 状态获取 ====================
    
    def get_cached_status(self) -> Optional[Dict[str, Any]]:
        """获取缓存的任务状态"""
        if self.mock_mode:
            return None
        return self.task_status
    
    # ==================== 异步接口 ====================
    
    async def execute(self, task_json: str) -> Optional[Dict[str, Any]]:
        """执行任务"""
        if self.mock_mode:
            return None
        
        if not self.execute_client:
            return None
        
        return await self._create_async_command('task_execute', {'task_json': task_json})
    
    async def pause(self, task_id: str) -> Optional[Dict[str, Any]]:
        """暂停任务"""
        if self.mock_mode:
            return None
        
        if not self.pause_client:
            return None
        
        return await self._create_async_command('task_pause', {'task_id': task_id})
    
    async def resume(self, task_id: str) -> Optional[Dict[str, Any]]:
        """恢复任务"""
        if self.mock_mode:
            return None
        
        if not self.resume_client:
            return None
        
        return await self._create_async_command('task_resume', {'task_id': task_id})
    
    async def cancel(self, task_id: str) -> Optional[Dict[str, Any]]:
        """取消任务"""
        if self.mock_mode:
            return None
        
        if not self.cancel_client:
            return None
        
        return await self._create_async_command('task_cancel', {'task_id': task_id})
    
    async def get_status(self, task_id: str) -> Optional[Dict[str, Any]]:
        """获取任务状态"""
        if self.mock_mode:
            return None
        
        return await self._create_async_command('task_get_status', {'task_id': task_id})
