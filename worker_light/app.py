import structlog
from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from contextlib import asynccontextmanager

from shared.logging import configure_logging, log_marker_middleware
from worker_light.api.routes import light_router
from worker_light.utils.git import sync_skills
from worker_light.config import settings

# Configure structured logging
configure_logging("worker-light")

logger = structlog.get_logger(__name__)


@asynccontextmanager
async def lifespan(app: FastAPI):
    # Startup: Sync skills
    sync_skills(
        repo_url=settings.git_repo_url,
        pat=settings.git_pat,
        skills_dir=settings.skills_dir,
    )
    yield
    # Shutdown
    pass


app = FastAPI(
    title="Problemologist Worker API (Light)",
    description="Sandboxed execution and filesystem API for agentic CAD tasks",
    version="0.1.0",
    lifespan=lifespan,
)
app.add_middleware(log_marker_middleware())

# Add CORS middleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

app.include_router(light_router, tags=["worker-light"])


@app.get("/health")
async def health_check():
    """Health check endpoint."""
    return {"status": "healthy"}


@app.get("/")
async def root():
    """Root endpoint returning API information."""
    return {
        "name": "Problemologist Worker (Light)",
        "version": "0.1.0",
        "docs": "/docs",
    }
