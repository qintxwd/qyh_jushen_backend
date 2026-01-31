from __future__ import annotations
from dataclasses import dataclass, field
from typing import Dict, List
import time
import uuid
import asyncio


@dataclass
class Session:
    session_id: str
    user_id: str
    robot_id: str
    created_at: float = field(default_factory=time.time)
    messages_to_user: List[dict] = field(default_factory=list)
    messages_to_robot: List[dict] = field(default_factory=list)


class SessionManager:
    def __init__(self) -> None:
        self._sessions: Dict[str, Session] = {}
        self._waiters_user: Dict[str, asyncio.Event] = {}
        self._waiters_robot: Dict[str, asyncio.Event] = {}
        self._waiters_robot_any: Dict[str, asyncio.Event] = {}

    def create(self, user_id: str, robot_id: str) -> Session:
        session_id = str(uuid.uuid4())
        session = Session(session_id=session_id, user_id=user_id, robot_id=robot_id)
        self._sessions[session_id] = session
        return session

    def get(self, session_id: str) -> Session | None:
        return self._sessions.get(session_id)

    def delete(self, session_id: str) -> bool:
        removed = self._sessions.pop(session_id, None) is not None
        self._waiters_user.pop(session_id, None)
        self._waiters_robot.pop(session_id, None)
        return removed

    def get_or_create(self, user_id: str, robot_id: str, session_id: str | None) -> Session:
        if session_id and session_id in self._sessions:
            return self._sessions[session_id]
        return self.create(user_id, robot_id)

    def push_to_user(self, session_id: str, msg: dict) -> None:
        session = self._sessions.get(session_id)
        if session:
            session.messages_to_user.append(msg)
            self._waiters_user.setdefault(session_id, asyncio.Event()).set()

    def push_to_robot(self, session_id: str, msg: dict) -> None:
        session = self._sessions.get(session_id)
        if session:
            session.messages_to_robot.append(msg)
            self._waiters_robot.setdefault(session_id, asyncio.Event()).set()
            self._waiters_robot_any.setdefault(session.robot_id, asyncio.Event()).set()

    def pop_for_user(self, session_id: str) -> List[dict]:
        session = self._sessions.get(session_id)
        if not session:
            return []
        out = list(session.messages_to_user)
        session.messages_to_user.clear()
        return out

    def pop_for_robot(self, session_id: str) -> List[dict]:
        session = self._sessions.get(session_id)
        if not session:
            return []
        out = list(session.messages_to_robot)
        session.messages_to_robot.clear()
        return out

    def pop_for_robot_any(self, robot_id: str) -> List[dict]:
        out: List[dict] = []
        for session in self._sessions.values():
            if session.robot_id != robot_id:
                continue
            if session.messages_to_robot:
                out.extend(session.messages_to_robot)
                session.messages_to_robot.clear()
        return out

    async def wait_for_user(self, session_id: str, timeout: float) -> None:
        event = self._waiters_user.setdefault(session_id, asyncio.Event())
        try:
            await asyncio.wait_for(event.wait(), timeout=timeout)
        except asyncio.TimeoutError:
            return
        finally:
            event.clear()

    async def wait_for_robot(self, session_id: str, timeout: float) -> None:
        event = self._waiters_robot.setdefault(session_id, asyncio.Event())
        try:
            await asyncio.wait_for(event.wait(), timeout=timeout)
        except asyncio.TimeoutError:
            return
        finally:
            event.clear()

    async def wait_for_robot_any(self, robot_id: str, timeout: float) -> None:
        event = self._waiters_robot_any.setdefault(robot_id, asyncio.Event())
        try:
            await asyncio.wait_for(event.wait(), timeout=timeout)
        except asyncio.TimeoutError:
            return
        finally:
            event.clear()


session_manager = SessionManager()
