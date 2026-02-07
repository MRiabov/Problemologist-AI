import os
import uuid
from fastapi import FastAPI, BackgroundTasks
from pydantic import BaseModel, Field, StrictStr
from temporalio.client import Client

from src.controller.persistence.db import get_sessionmaker
from src.controller.persistence.models import Episode
from src.controller.clients.worker import WorkerClient
from src.controller.graph.agent import create_agent_graph
from src.controller.middleware.remote_fs import RemoteFilesystemMiddleware
from src.controller.workflows.simulation import SimulationWorkflow
from src.controller.api.routes import episodes, skills
from src.shared.enums import ResponseStatus, EpisodeStatus
from src.shared.logging import configure_logging, get_logger

# Configure logging
configure_logging("controller")
logger = get_logger(__name__)

TEMPORAL_URL = os.getenv("TEMPORAL_URL", "temporal:7233")
WORKER_URL = os.getenv("WORKER_URL", "http://worker:8001")

app = FastAPI(title="Problemologist Controller")

app.include_router(episodes.router)
app.include_router(skills.router)

# Temporal client
temporal_client_instance: Client = None


@app.on_event("startup")
async def startup_event():
    global temporal_client_instance
    try:
        temporal_client_instance = await Client.connect(TEMPORAL_URL)
    except Exception as e:
        logger.error("failed_to_connect_to_temporal", error=str(e))


class AgentRunRequest(BaseModel):
    task: StrictStr = Field(..., description="The task for the agent to perform.")
    session_id: StrictStr = Field(..., description="Session ID for the worker.")


class SimulationRequest(BaseModel):
    session_id: StrictStr = Field(..., description="Session ID for the worker.")
    compound_json: StrictStr = Field(default="{}", description="Component data.")


def get_worker_client(session_id: str) -> WorkerClient:
    return WorkerClient(base_url=WORKER_URL, session_id=session_id)


async def execute_agent_task(episode_id: uuid.UUID, task: str, session_id: str):
    session_factory = get_sessionmaker()
    async with session_factory() as db:
        episode = await db.get(Episode, episode_id)
        if not episode:
            logger.error("episode_not_found_for_background_task", episode_id=episode_id)
            return

        try:
            client = get_worker_client(session_id)
            fs_middleware = RemoteFilesystemMiddleware(
                client, temporal_client=temporal_client_instance
            )
            agent = create_agent_graph(fs_middleware)

            # Run the agent
            result = await agent.ainvoke({"messages": [("user", task)]})
            
            # Re-fetch episode to avoid detached session issues
            episode = await db.get(Episode, episode_id)
            episode.status = EpisodeStatus.COMPLETED
            # We could also save result["messages"] here if we had a place for it
            await db.commit()
            logger.info("agent_run_completed", episode_id=episode_id)
        except Exception as e:
            episode = await db.get(Episode, episode_id)
            episode.status = EpisodeStatus.FAILED
            await db.commit()
            logger.error("agent_run_failed", error=str(e), episode_id=episode_id)


@app.get("/")
async def read_root():
    return {"status": ResponseStatus.OK, "service": "controller"}


@app.get("/health")
async def health():
    return {"status": ResponseStatus.HEALTHY}


@app.post("/simulation/run")
async def run_simulation(request: SimulationRequest):
    if not temporal_client_instance:
        return {
            "status": ResponseStatus.ERROR,
            "message": "Temporal client not connected",
        }

    handle = await temporal_client_instance.start_workflow(
        SimulationWorkflow.run,
        request.compound_json,
        id=f"sim-{request.session_id}-{os.urandom(4).hex()}",
        task_queue="simulation-task-queue",
    )
    return {"status": ResponseStatus.ACCEPTED, "workflow_id": handle.id}


@app.post("/agent/run")
async def run_agent(request: AgentRunRequest, background_tasks: BackgroundTasks):
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

    background_tasks.add_task(execute_agent_task, episode_id, request.task, request.session_id)

    return {
        "status": ResponseStatus.ACCEPTED,
        "episode_id": episode_id
    }

