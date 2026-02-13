import uuid
from contextlib import asynccontextmanager

from fastapi import FastAPI, HTTPException
from temporalio.client import Client

from controller.api import ops
from controller.api.manager import task_tracker
from controller.api.routes import benchmark, cots, episodes, skills
from controller.config.settings import settings
from controller.persistence.db import get_sessionmaker
from controller.persistence.models import Episode
from shared.enums import EpisodeStatus, ResponseStatus
from shared.logging import configure_logging, get_logger

from controller.observability.langfuse import get_langfuse_client

# Configure logging
configure_logging("controller")
logger = get_logger(__name__)

# Initialize Langfuse client globally early to avoid "uninitialized" warnings
try:
    get_langfuse_client()
except Exception as e:
    logger.warning("failed_to_initialize_langfuse_globally", error=str(e))

TEMPORAL_URL = settings.temporal_url
WORKER_URL = settings.worker_url

# Temporal client
temporal_client_instance: Client = None


@asynccontextmanager
async def lifespan(app: FastAPI):
    global temporal_client_instance
    try:
        temporal_client_instance = await Client.connect(TEMPORAL_URL)
        app.state.temporal_client = temporal_client_instance
        logger.info("connected_to_temporal", url=TEMPORAL_URL)
    except Exception as e:
        logger.error("failed_to_connect_to_temporal", error=str(e))

    yield

    # Clean up if needed
    pass


app = FastAPI(title="Problemologist Controller", lifespan=lifespan)


@app.get("/")
async def read_root():
    """Root endpoint for service discovery."""
    return {"status": "ok", "service": "controller"}


app.include_router(episodes.router)
app.include_router(benchmark.router)
app.include_router(skills.router)
app.include_router(ops.router)
app.include_router(cots.router)
from controller.api.routes import simulation

app.include_router(simulation.router)


from controller.api.tasks import AgentRunRequest, execute_agent_task


@app.post("/test/episodes", status_code=201)
async def create_test_episode(request: AgentRunRequest):
    """Create a dummy episode for testing purposes (no agent run)."""
    if not settings.is_integration_test:
        raise HTTPException(status_code=403, detail="Only allowed in integration tests")

    session_factory = get_sessionmaker()
    async with session_factory() as db:
        episode = Episode(
            id=uuid.uuid4(),
            task=request.task,
            status=EpisodeStatus.RUNNING,
            metadata_vars=request.metadata_vars,
            skill_git_hash=request.skill_git_hash,
        )
        db.add(episode)
        await db.commit()
        await db.refresh(episode)
        return {"episode_id": episode.id}


@app.get("/health")
async def health_check():
    """Health check endpoint."""
    return {
        "status": "healthy",
        "is_integration_test": settings.is_integration_test,
    }


@app.post("/agent/run", status_code=202)
async def run_agent(request: AgentRunRequest):
    # Note: We removed BackgroundTasks - we use asyncio.create_task for granular control
    session_factory = get_sessionmaker()
    async with session_factory() as db:
        episode = Episode(
            id=uuid.uuid4(),
            task=request.task,
            status=EpisodeStatus.RUNNING,
            metadata_vars=request.metadata_vars,
            skill_git_hash=request.skill_git_hash,
        )
        db.add(episode)
        await db.commit()
        await db.refresh(episode)

        episode_id = episode.id

    # Create task and register it
    import asyncio

    task = asyncio.create_task(
        execute_agent_task(
            episode_id,
            request.task,
            request.session_id,
            agent_name=request.agent_name,
        )
    )
    task_tracker.register_task(episode_id, task)

    return {"status": ResponseStatus.ACCEPTED, "episode_id": episode_id}
