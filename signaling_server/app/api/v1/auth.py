from fastapi import APIRouter, HTTPException
from pydantic import BaseModel

from app.config import settings
from app.core.security import create_access_token

router = APIRouter(prefix="/api/v1", tags=["auth"])


class LoginRequest(BaseModel):
    username: str
    password: str


@router.post("/login")
def login(req: LoginRequest):
    expected = settings.default_users.get(req.username)
    if not expected or expected != req.password:
        raise HTTPException(status_code=401, detail="Invalid credentials")
    token = create_access_token(subject=req.username, role="user")
    return {"access_token": token, "token_type": "bearer"}
