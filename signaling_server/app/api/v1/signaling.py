from fastapi import APIRouter, Depends, HTTPException
from pydantic import BaseModel
from typing import Literal

from app.core.security import get_caller
from app.core.robot_registry import robot_registry
from app.services.signaling_relay import SignalingRelay

router = APIRouter(prefix="/api/v1", tags=["signaling"])


class SignalingMessage(BaseModel):
    type: Literal["offer", "answer", "ice_candidate", "poll"]
    session_id: str | None = None
    sdp: str | None = None
    candidate: str | None = None
    sdp_mid: str | None = None
    sdp_mline_index: int | None = None


class SignalingRequest(BaseModel):
    robot_id: str
    session_id: str | None = None
    message: SignalingMessage
    timeout_ms: int | None = None


class SignalingResponse(BaseModel):
    session_id: str
    messages: list[dict]


@router.post("/signaling", response_model=SignalingResponse)
async def signaling_http_endpoint(req: SignalingRequest, caller=Depends(get_caller)):
    if caller["type"] == "robot":
        robot = robot_registry.get(req.robot_id)
        if not robot:
            robot_registry.register(req.robot_id, req.robot_id, [])
        else:
            robot_registry.heartbeat(req.robot_id)
    else:
        robot = robot_registry.get(req.robot_id)
        if not robot or not robot.online:
            raise HTTPException(status_code=404, detail="Robot offline")

    msg = req.message.model_dump(exclude_none=True)
    if req.session_id and "session_id" not in msg:
        msg["session_id"] = req.session_id

    if msg["type"] in ("offer", "answer") and "sdp" not in msg:
        raise HTTPException(status_code=400, detail="Missing sdp")
    if msg["type"] == "ice_candidate" and "candidate" not in msg:
        raise HTTPException(status_code=400, detail="Missing candidate")

    relay = SignalingRelay()
    session_id, outbox = await relay.handle_http_message(
        caller_type=caller["type"],
        caller_id=caller["id"],
        robot_id=req.robot_id,
        session_id=req.session_id,
        message=msg,
        timeout_ms=req.timeout_ms,
    )
    return SignalingResponse(session_id=session_id, messages=outbox)
