from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware

from app.api.v1 import auth, robots, session, signaling

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


@app.get("/health")
def health():
    return {"status": "ok"}
