import asyncio
import os
import shutil
from contextlib import asynccontextmanager
from pathlib import Path

import structlog
from fastapi import FastAPI, Request
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import JSONResponse

from shared.logging import configure_logging, log_marker_middleware
from worker_heavy.api.routes import heavy_router
from worker_light.api.routes import light_router
from worker_light.config import settings
from worker_light.utils.git import sync_skills

# Configure structured logging
configure_logging("worker-light")

logger = structlog.get_logger(__name__)


def _seed_integration_skills() -> None:
    """Populate worker skills mount for integration tests."""
    skills_dir = settings.skills_dir
    skills_dir.mkdir(parents=True, exist_ok=True)

    copied = 0
    repo_skill_root = Path(__file__).resolve().parents[1] / "skills"
    if repo_skill_root.exists():
        for skill_dir in sorted(repo_skill_root.iterdir(), key=lambda p: p.name):
            if not skill_dir.is_dir():
                continue
            skill_md = skill_dir / "SKILL.md"
            if not skill_md.exists():
                continue
            target = skills_dir / skill_dir.name / "SKILL.md"
            target.parent.mkdir(parents=True, exist_ok=True)
            shutil.copyfile(skill_md, target)
            copied += 1

    if copied == 0:
        placeholder = skills_dir / "build123d_cad_drafting_skill" / "SKILL.md"
        placeholder.parent.mkdir(parents=True, exist_ok=True)
        placeholder.write_text("", encoding="utf-8")

    # Keep deterministic content for integration contract assertions.
    canonical = skills_dir / "build123d_cad_drafting_skill" / "SKILL.md"
    canonical.parent.mkdir(parents=True, exist_ok=True)
    canonical.write_text(
        "# Build123d CAD Drafting Skill\n\nPlaceholder skill for integration mode.\n",
        encoding="utf-8",
    )


@asynccontextmanager
async def lifespan(app: FastAPI):
    # Startup: Sync skills (skipped in integration tests for speed/reliability)
    if settings.is_integration_test:
        _seed_integration_skills()
    elif settings.git_repo_url:
        sync_skills(
            repo_url=settings.git_repo_url,
            pat=settings.git_pat,
            skills_dir=settings.skills_dir,
            session_id="system",
        )

    # WP11: Support Temporal worker in unified mode
    if os.getenv("WORKER_TYPE") == "unified":
        try:
            from worker_heavy.temporal_worker import main as heavy_temporal_worker_main

            app.state.temporal_task = asyncio.create_task(heavy_temporal_worker_main())
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


@app.exception_handler(PermissionError)
async def permission_error_handler(request: Request, exc: PermissionError):
    """Handle path traversal and other permission errors with 403."""
    return JSONResponse(
        status_code=403,
        content={"detail": str(exc)},
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


@app.middleware("http")
async def log_unhandled_http_exceptions(request: Request, call_next):
    try:
        return await call_next(request)
    except Exception:
        logger.exception(
            "unhandled_http_exception",
            method=request.method,
            path=request.url.path,
            query=str(request.url.query),
        )
        raise


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
