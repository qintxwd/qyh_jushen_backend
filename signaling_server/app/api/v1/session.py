from fastapi import APIRouter, Depends
from pydantic import BaseModel

from app.core.security import get_current_user
from app.core.session_manager import session_manager

router = APIRouter(prefix="/api/v1", tags=["session"])


class AcquireRequest(BaseModel):
    robot_id: str


@router.post("/session/acquire")
def acquire_session(req: AcquireRequest, user=Depends(get_current_user)):
    session = session_manager.create(user_id=user.sub, robot_id=req.robot_id)
    return {"session_id": session.session_id, "robot_id": session.robot_id}


@router.delete("/session/release/{session_id}")
def release_session(session_id: str, user=Depends(get_current_user)):
    session = session_manager.get(session_id)
    if not session:
        return {"released": False}
    if session.user_id != user.sub:
        return {"released": False}
    return {"released": session_manager.delete(session_id)}
