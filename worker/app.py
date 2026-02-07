import os
import structlog
from pathlib import Path
from contextlib import asynccontextmanager
from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from git import Repo

from shared.logging import configure_logging

from .api.routes import router as api_router
from .filesystem.watchdog import start_watchdog

# Configure structured logging
configure_logging("worker")

logger = structlog.get_logger(__name__)


def _get_auth_url(repo_url: str, pat: str | None) -> str:
    """Inject PAT into URL if available."""
    if pat and "https://" in repo_url:
        return repo_url.replace("https://", f"https://{pat}@")
    return repo_url


def _sync_skills():
    """Clone or pull skills from git repo on startup."""
    repo_url = os.getenv("GIT_REPO_URL")
    pat = os.getenv("GIT_PAT")

    if not repo_url:
        logger.info("GIT_REPO_URL not set, skipping skills sync.")
        return

    skills_dir = Path(__file__).parent / "skills"
    skills_dir.mkdir(parents=True, exist_ok=True)

    auth_url = _get_auth_url(repo_url, pat)

    try:
        if (skills_dir / ".git").exists():
            logger.info("Pulling latest skills...")
            repo = Repo(skills_dir)
            repo.remotes.origin.pull()
            logger.info("Skills pulled successfully.")
        else:
            # Ensure dir is empty or git will complain, unless we clone into .
            # If it's not empty (e.g. __init__.py), we might have an issue.
            # But Repo.clone_from might fail if dir is not empty.
            # We can try to init and pull.
            logger.info(f"Cloning skills from {repo_url}...")
            # If directory exists and is not empty, clone_from will fail.
            # Assuming standard container setup where it might be empty or mapped.
            # If it has __init__.py, we should probably use init + remote add + pull
            if any(skills_dir.iterdir()):
                repo = Repo.init(skills_dir)
                if "origin" not in repo.remotes:
                    repo.create_remote("origin", auth_url)
                repo.remotes.origin.pull("master")  # or main
            else:
                Repo.clone_from(auth_url, skills_dir)
            logger.info("Skills cloned successfully.")
    except Exception as e:
        logger.error(f"Failed to sync skills: {e}")


@asynccontextmanager
async def lifespan(app: FastAPI):
    # Startup: Sync skills
    _sync_skills()

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
