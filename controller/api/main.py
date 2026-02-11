import uuid
from contextlib import asynccontextmanager

from fastapi import FastAPI
from temporalio.client import Client

from controller.api import ops
from controller.api.manager import task_tracker
from controller.api.routes import benchmark, episodes, skills
from controller.config.settings import settings
from controller.persistence.db import get_sessionmaker
from controller.persistence.models import Episode
from shared.enums import EpisodeStatus, ResponseStatus
from shared.logging import configure_logging, get_logger

# Configure logging
configure_logging("controller")
logger = get_logger(__name__)

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


@app.get("/health")
async def health_check():
    """Health check endpoint."""
    return {"status": "healthy"}


from controller.api.tasks import AgentRunRequest, execute_agent_task


@app.post("/agent/run")
async def run_agent(request: AgentRunRequest):
    # Note: We removed BackgroundTasks - we use asyncio.create_task for granular control
    session_factory = get_sessionmaker()
    async with session_factory() as db:
        episode = Episode(
            id=uuid.uuid4(),
            task=request.task,
            status=EpisodeStatus.RUNNING,
        )
        db.add(episode)
        await db.commit()
        await db.refresh(episode)

        episode_id = episode.id

    # Create task and register it
    import asyncio

    task = asyncio.create_task(
        execute_agent_task(episode_id, request.task, request.session_id)
    )
    task_tracker.register_task(episode_id, task)

    return {"status": ResponseStatus.ACCEPTED, "episode_id": episode_id}
