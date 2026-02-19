from contextlib import asynccontextmanager

import structlog
from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware

from shared.logging import configure_logging

import os
from .api.routes import light_router, heavy_router, router as api_router
from .skills.sync import sync_skills

# Configure structured logging
configure_logging("worker")

logger = structlog.get_logger(__name__)


async def start_temporal_worker():
    """Start Temporal worker for heavy tasks."""
    try:
        from temporalio.client import Client
        from temporalio.worker import Worker
        from .activities.heavy_tasks import (
            preview_design_activity,
            run_simulation_activity,
            validate_design_activity,
        )

        temporal_url = os.getenv("TEMPORAL_URL", "temporal:7233")
        client = await Client.connect(temporal_url)

        worker = Worker(
            client,
            task_queue="heavy-tasks-queue",
            activities=[
                run_simulation_activity,
                validate_design_activity,
                preview_design_activity,
            ],
        )
        logger.info("temporal_worker_started", queue="heavy-tasks-queue")
        await worker.run()
    except Exception as e:
        logger.error("temporal_worker_failed", error=str(e))


@asynccontextmanager
async def lifespan(app: FastAPI):
    # Startup: Sync skills
    sync_skills()

    worker_type = os.getenv("WORKER_TYPE")
    if worker_type == "heavy":
        import asyncio

        app.state.temporal_task = asyncio.create_task(start_temporal_worker())

    yield
    # Shutdown
    if hasattr(app.state, "temporal_task"):
        app.state.temporal_task.cancel()


app = FastAPI(
    title="Problemologist Worker API",
    description="Sandboxed execution and filesystem API for agentic CAD tasks",
    version="0.1.0",
    lifespan=lifespan,
)

# Add CORS middleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # In production, this should be restricted
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Include API routes based on worker type
worker_type = os.getenv("WORKER_TYPE")
if worker_type == "light":
    app.include_router(light_router, tags=["worker-light"])
elif worker_type == "heavy":
    app.include_router(heavy_router, tags=["worker-heavy"])
else:
    # Default/combined mode
    app.include_router(api_router, tags=["worker"])


@app.get("/health")
async def health_check():
    """Health check endpoint."""
    return {"status": "healthy"}


@app.get("/")
async def root():
    """Root endpoint returning API information."""
    return {"name": "Problemologist Worker", "version": "0.1.0", "docs": "/docs"}
