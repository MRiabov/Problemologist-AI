import structlog
from contextlib import asynccontextmanager
from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware

from shared.logging import configure_logging

from .api.routes import router as api_router
from .filesystem.watchdog import start_watchdog
from .skills.sync import sync_skills

# Configure structured logging
configure_logging("worker")

logger = structlog.get_logger(__name__)


@asynccontextmanager
async def lifespan(app: FastAPI):
    # Startup: Sync skills
    sync_skills()

    # Startup: Start the filesystem watchdog
    # We watch the current directory as it's likely the workspace root in the container
    observer = start_watchdog(".")
    yield
    # Shutdown: Stop the watchdog
    if observer:
        observer.stop()
        observer.join()
        logger.info("watchdog_stopped")


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

# Include API routes
app.include_router(api_router, tags=["worker"])


@app.get("/health")
async def health_check():
    """Health check endpoint."""
    return {"status": "healthy"}


@app.get("/")
async def root():
    """Root endpoint returning API information."""
    return {"name": "Problemologist Worker", "version": "0.1.0", "docs": "/docs"}