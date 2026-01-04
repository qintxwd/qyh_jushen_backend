"""控制权管理 API"""
from fastapi import APIRouter, Depends, HTTPException, status
from pydantic import BaseModel
from app.dependencies import get_current_operator
from app.models.user import User
from app.core.control_lock import control_lock

router = APIRouter()


class AcquireRequest(BaseModel):
    """获取控制权请求"""
    duration: int = 300  # 默认5分钟


@router.post("/acquire")
async def acquire_control(
    request: AcquireRequest,
    current_user: User = Depends(get_current_operator)
):
    """获取机器人控制权"""
    success = control_lock.acquire(
        user_id=current_user.id,
        username=current_user.username,
        duration=request.duration
    )
    
    if success:
        holder = control_lock.get_holder()
        return {
            "success": True,
            "holder": holder
        }
    else:
        holder = control_lock.get_holder()
        raise HTTPException(
            status_code=status.HTTP_409_CONFLICT,
            detail=f"控制权已被用户 {holder['username']} 持有",
        )


@router.post("/release")
async def release_control(current_user: User = Depends(get_current_operator)):
    """释放控制权"""
    success = control_lock.release(current_user.id)
    
    if success:
        return {"success": True, "message": "控制权已释放"}
    else:
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail="您未持有控制权"
        )


@router.post("/renew")
async def renew_control(
    request: AcquireRequest,
    current_user: User = Depends(get_current_operator)
):
    """续约控制权"""
    if not control_lock.is_held_by(current_user.id):
        raise HTTPException(
            status_code=status.HTTP_403_FORBIDDEN,
            detail="您未持有控制权"
        )
    
    control_lock.acquire(current_user.id, current_user.username, request.duration)
    holder = control_lock.get_holder()
    
    return {
        "success": True,
        "holder": holder
    }


@router.get("/status")
async def get_control_status():
    """查询控制权状态"""
    holder = control_lock.get_holder()
    
    if holder:
        return {
            "locked": True,
            "holder": holder
        }
    else:
        return {
            "locked": False,
            "holder": None
        }
