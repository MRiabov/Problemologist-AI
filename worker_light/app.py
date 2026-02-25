import asyncio
import os
from contextlib import asynccontextmanager

import structlog
from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware

from shared.logging import configure_logging, log_marker_middleware
from worker_heavy.api.routes import heavy_router
from worker_light.api.routes import light_router
from worker_light.config import settings
from worker_light.utils.git import sync_skills

# Configure structured logging
configure_logging("worker-light")

logger = structlog.get_logger(__name__)


@asynccontextmanager
async def lifespan(app: FastAPI):
    # Startup: Sync skills (skipped in integration tests for speed/reliability)
    if not settings.is_integration_test and settings.git_repo_url:
        sync_skills(
            repo_url=settings.git_repo_url,
            pat=settings.git_pat,
            skills_dir=settings.skills_dir,
        )

    # WP11: Support Temporal worker in unified mode
    if os.getenv("WORKER_TYPE") == "unified":
        try:
            from worker_heavy.app import start_temporal_worker

            app.state.temporal_task = asyncio.create_task(start_temporal_worker())
        except ImportError:
            pass

    yield
    # Shutdown
    if hasattr(app.state, "temporal_task"):
        app.state.temporal_task.cancel()


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

# Support unified mode for integration tests
if os.getenv("WORKER_TYPE") == "unified":
    app.include_router(heavy_router, tags=["worker-heavy"])


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
