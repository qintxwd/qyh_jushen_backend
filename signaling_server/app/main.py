from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware

from app.api.v1 import auth, robots, session, signaling
from app.config import settings
from app.core.security import get_password_hash
from app.database import Base, SessionLocal, engine
from app.models.user import User, UserRole

app = FastAPI(title="QYH Signaling Server", version="2.1")

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

app.include_router(auth.router)
app.include_router(robots.router)
app.include_router(session.router)
app.include_router(signaling.router)


@app.on_event("startup")
def setup_database() -> None:
    Base.metadata.create_all(bind=engine)
    db = SessionLocal()
    try:
        admin = (
            db.query(User)
            .filter(User.username == settings.default_admin_username)
            .first()
        )
        if not admin:
            admin = User(
                username=settings.default_admin_username,
                email=settings.default_admin_email,
                hashed_password=get_password_hash(
                    settings.default_admin_password,
                ),
                role=UserRole.ADMIN,
                is_active=True,
            )
            db.add(admin)
            db.commit()
    finally:
        db.close()


@app.get("/health")
def health():
    return {"status": "ok"}
