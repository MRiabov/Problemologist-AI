import os
import structlog
import asyncio
from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from contextlib import asynccontextmanager

from shared.logging import configure_logging
from worker_heavy.api.routes import heavy_router

# Configure structured logging
configure_logging("worker-heavy")

logger = structlog.get_logger(__name__)


async def start_temporal_worker():
    """Start Temporal worker for heavy tasks."""
    try:
        from temporalio.client import Client
        from temporalio.worker import Worker
        from worker_heavy.activities.heavy_tasks import (
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
    # Heavy worker always runs Temporal worker
    app.state.temporal_task = asyncio.create_task(start_temporal_worker())
    yield
    # Shutdown
    if hasattr(app.state, "temporal_task"):
        app.state.temporal_task.cancel()


app = FastAPI(
    title="Problemologist Worker API (Heavy)",
    description="Heavy compute API (simulation, validation) for agentic CAD tasks",
    version="0.1.0",
    lifespan=lifespan,
)

# Add CORS middleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

app.include_router(heavy_router, tags=["worker-heavy"])


@app.get("/health")
async def health_check():
    """Health check endpoint."""
    return {"status": "healthy"}


@app.get("/")
async def root():
    """Root endpoint returning API information."""
    return {"name": "Problemologist Worker (Heavy)", "version": "0.1.0", "docs": "/docs"}
