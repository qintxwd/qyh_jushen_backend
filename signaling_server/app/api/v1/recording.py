"""Recording control API (mock implementation)."""
from __future__ import annotations

from typing import List

from fastapi import APIRouter
from pydantic import BaseModel, Field

router = APIRouter(prefix="/api/v1/recording", tags=["recording"])


class StartRecordingRequest(BaseModel):
    action_name: str = Field(...)
    user_name: str = Field(...)
    version: str = Field(...)
    topics: List[str] = Field(default_factory=list)


class StartRecordingResponse(BaseModel):
    success: bool
    message: str
    bag_path: str = ""


class StopRecordingResponse(BaseModel):
    success: bool
    message: str
    duration_sec: float = 0.0
    bag_path: str = ""


class RecordingStatusResponse(BaseModel):
    is_recording: bool
    action_name: str = ""
    duration_sec: float = 0.0
    bag_path: str = ""
    topics: List[str] = []


_is_recording = False
_last_action = ""
_last_topics: List[str] = []


@router.post("/start", response_model=StartRecordingResponse)
async def start_recording(request: StartRecordingRequest):
    global _is_recording, _last_action, _last_topics
    _is_recording = True
    _last_action = request.action_name
    _last_topics = request.topics
    return StartRecordingResponse(
        success=True,
        message="Recording started (mock)",
        bag_path=f"/tmp/{request.action_name}_mock",
    )


@router.post("/stop", response_model=StopRecordingResponse)
async def stop_recording():
    global _is_recording
    _is_recording = False
    return StopRecordingResponse(
        success=True,
        message="Recording stopped (mock)",
        duration_sec=10.0,
        bag_path="/tmp/mock_bag",
    )


@router.get("/status", response_model=RecordingStatusResponse)
async def get_recording_status():
    return RecordingStatusResponse(
        is_recording=_is_recording,
        action_name=_last_action,
        duration_sec=0.0,
        bag_path="/tmp/mock_bag" if _is_recording else "",
        topics=_last_topics,
    )


@router.post("/discard", response_model=StopRecordingResponse)
async def discard_recording():
    global _is_recording
    _is_recording = False
    return StopRecordingResponse(
        success=True,
        message="Recording discarded (mock)",
        duration_sec=0.0,
        bag_path="",
    )


@router.get("/topics")
async def get_available_topics():
    return {
        "topics": [
            "/joint_states",
            "/tf",
            "/tf_static",
            "/left_arm/joint_states",
            "/right_arm/joint_states",
            "/left_gripper/state",
            "/right_gripper/state",
            "/head/joint_states",
            "/lift/state",
            "/chassis/odom",
        ]
    }
