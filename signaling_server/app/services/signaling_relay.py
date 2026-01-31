from __future__ import annotations
from typing import Tuple, List

from app.core.session_manager import session_manager


class SignalingRelay:
    async def handle_http_message(
        self,
        caller_type: str,
        caller_id: str,
        robot_id: str,
        session_id: str | None,
        message: dict,
        timeout_ms: int | None = None,
    ) -> Tuple[str, List[dict]]:
        msg_type = message.get("type")

        if msg_type == "poll":
            if caller_type == "user":
                if not session_id:
                    return "", []
                out = session_manager.pop_for_user(session_id)
                if not out and timeout_ms:
                    await session_manager.wait_for_user(session_id, timeout_ms / 1000.0)
                    out = session_manager.pop_for_user(session_id)
                return session_id, out

            if not session_id:
                out = session_manager.pop_for_robot_any(robot_id)
                if not out and timeout_ms:
                    await session_manager.wait_for_robot_any(robot_id, timeout_ms / 1000.0)
                    out = session_manager.pop_for_robot_any(robot_id)
                return "", out

            out = session_manager.pop_for_robot(session_id)
            if not out and timeout_ms:
                await session_manager.wait_for_robot(session_id, timeout_ms / 1000.0)
                out = session_manager.pop_for_robot(session_id)
            return session_id, out

        session = session_manager.get_or_create(
            user_id=caller_id if caller_type == "user" else "unknown",
            robot_id=robot_id,
            session_id=session_id,
        )

        if caller_type == "user":
            if "session_id" not in message:
                message["session_id"] = session.session_id
            session_manager.push_to_robot(session.session_id, message)
            return session.session_id, []

        if "session_id" not in message:
            message["session_id"] = session.session_id
        session_manager.push_to_user(session.session_id, message)
        return session.session_id, []
