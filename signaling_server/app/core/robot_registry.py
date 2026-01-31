from __future__ import annotations
from dataclasses import dataclass, field
from typing import Dict, List
import time

from app.config import settings


@dataclass
class RobotInfo:
    robot_id: str
    name: str = ""
    online: bool = False
    last_seen: float = 0.0
    video_sources: List[str] = field(default_factory=list)


class RobotRegistry:
    def __init__(self) -> None:
        self._robots: Dict[str, RobotInfo] = {}

    def register(self, robot_id: str, name: str = "", video_sources: List[str] | None = None) -> RobotInfo:
        info = self._robots.get(robot_id) or RobotInfo(robot_id=robot_id)
        info.name = name or info.name or robot_id
        info.online = True
        info.last_seen = time.time()
        info.video_sources = video_sources or info.video_sources
        self._robots[robot_id] = info
        return info

    def heartbeat(self, robot_id: str) -> None:
        info = self._robots.get(robot_id)
        if info:
            info.online = True
            info.last_seen = time.time()

    def set_offline(self, robot_id: str) -> None:
        info = self._robots.get(robot_id)
        if info:
            info.online = False

    def get(self, robot_id: str) -> RobotInfo | None:
        return self._robots.get(robot_id)

    def list(self) -> List[RobotInfo]:
        return list(self._robots.values())

    def webrtc_config(self, robot_id: str) -> dict:
        info = self._robots.get(robot_id)
        return {
            "robot_id": robot_id,
            "signaling_url": "/signaling",
            "ice_servers": [s.model_dump() for s in settings.ice_servers],
            "video_sources": info.video_sources if info else [],
            "data_channels": [
                {"label": "control", "ordered": True, "max_retransmits": 0, "protocol": "protobuf"},
                {"label": "state", "ordered": False, "max_retransmits": 0, "protocol": "protobuf"},
                {"label": "event", "ordered": True, "max_retransmits": 3, "protocol": "protobuf"},
            ],
        }


robot_registry = RobotRegistry()
