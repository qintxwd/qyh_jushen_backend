"""Control lock API."""
from __future__ import annotations

from fastapi import APIRouter, Depends
from pydantic import BaseModel

from app.core.control_lock import control_lock
from app.dependencies import get_current_operator
from app.models.user import User
from app.safety.watchdog import watchdog

router = APIRouter(prefix="/api/v1/control", tags=["control"])


class AcquireRequest(BaseModel):
    duration: int = 300


@router.post("/acquire")
def acquire_control(
    request: AcquireRequest,
    current_user: User = Depends(get_current_operator),
):
    success = control_lock.acquire(
        user_id=current_user.id,
        username=current_user.username,
        duration=request.duration,
    )
    if success:
        return {"ok": True, "holder": control_lock.get_holder()}
    holder = control_lock.get_holder()
    return {
        "ok": False,
        "message": "control already held",
        "holder": holder,
    }


@router.post("/release")
def release_control(current_user: User = Depends(get_current_operator)):
    success = control_lock.release(current_user.id)
    return {"ok": success}


@router.post("/renew")
def renew_control(
    request: AcquireRequest,
    current_user: User = Depends(get_current_operator),
):
    if not control_lock.is_held_by(current_user.id):
        return {"ok": False, "message": "control not held"}
    control_lock.acquire(
        current_user.id,
        current_user.username,
        request.duration,
    )
    return {"ok": True, "holder": control_lock.get_holder()}


@router.get("/status")
def control_status():
    holder = control_lock.get_holder()
    return {"locked": holder is not None, "holder": holder}


@router.post("/heartbeat")
def control_heartbeat(current_user: User = Depends(get_current_operator)):
    watchdog.heartbeat()
    holder = control_lock.get_holder()
    return {"ok": True, "holder": holder}
