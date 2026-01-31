from fastapi import APIRouter, Depends, HTTPException, Request
from pydantic import BaseModel

from app.core.security import get_current_user
from app.core.robot_registry import robot_registry
from app.config import settings

router = APIRouter(prefix="/api/v1", tags=["robots"])


class RegisterRobotRequest(BaseModel):
    name: str | None = None
    video_sources: list[str] = []


@router.get("/robots")
def list_robots(user=Depends(get_current_user)):
    robots = [
        {
            "robot_id": r.robot_id,
            "name": r.name,
            "online": r.online,
            "last_seen": r.last_seen,
            "video_sources": r.video_sources,
        }
        for r in robot_registry.list()
    ]
    return {"robots": robots, "count": len(robots)}


@router.get("/robots/{robot_id}")
def get_robot(robot_id: str, user=Depends(get_current_user)):
    robot = robot_registry.get(robot_id)
    if not robot:
        raise HTTPException(status_code=404, detail="Robot not found")
    return {
        "robot_id": robot.robot_id,
        "name": robot.name,
        "online": robot.online,
        "last_seen": robot.last_seen,
        "video_sources": robot.video_sources,
    }


@router.get("/robots/{robot_id}/webrtc-config")
def get_webrtc_config(robot_id: str, user=Depends(get_current_user)):
    return robot_registry.webrtc_config(robot_id)


@router.post("/robots/{robot_id}/register")
def register_robot(robot_id: str, req: RegisterRobotRequest, request: Request):
    secret = request.headers.get("X-Robot-Secret")
    if secret != settings.robot_shared_secret:
        raise HTTPException(status_code=401, detail="Invalid robot secret")
    info = robot_registry.register(robot_id, req.name or robot_id, req.video_sources)
    return {
        "robot_id": info.robot_id,
        "name": info.name,
        "online": info.online,
        "last_seen": info.last_seen,
        "video_sources": info.video_sources,
    }
