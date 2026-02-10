import asyncio
import uuid
from contextlib import asynccontextmanager, suppress

from fastapi import FastAPI
from pydantic import BaseModel, Field, StrictStr
from temporalio.client import Client

from controller.api import ops
from controller.api.manager import task_tracker
from controller.api.routes import episodes, skills
from controller.clients.backend import RemoteFilesystemBackend
from controller.clients.worker import WorkerClient
from controller.config.settings import settings
from controller.graph.agent import create_agent_graph
from controller.observability.database import DatabaseCallbackHandler
from controller.persistence.db import get_sessionmaker
from controller.persistence.models import Asset, Episode, Trace
from shared.enums import AssetType, EpisodeStatus, ResponseStatus, TraceType
from shared.logging import configure_logging, get_logger
from controller.agent.initialization import initialize_agent_files

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

app.include_router(episodes.router)
app.include_router(skills.router)
app.include_router(ops.router)


@app.get("/health")
async def health_check():
    """Health check endpoint."""
    return {"status": "healthy"}


class AgentRunRequest(BaseModel):
    task: StrictStr = Field(..., description="The task for the agent to perform.")
    session_id: StrictStr = Field(..., description="Session ID for the worker.")


def get_worker_client(session_id: str) -> WorkerClient:
    return WorkerClient(base_url=WORKER_URL, session_id=session_id)


async def execute_agent_task(episode_id: uuid.UUID, task: str, session_id: str):
    session_factory = get_sessionmaker()

    try:
        async with session_factory() as db:
            episode = await db.get(Episode, episode_id)
            if not episode:
                logger.error(
                    "episode_not_found_for_background_task", episode_id=episode_id
                )
                return

            try:
                # Use a 32-char hex trace_id for OTEL/Langfuse v3 compatibility
                trace_id = uuid.uuid4().hex

                client = get_worker_client(session_id)
                backend = RemoteFilesystemBackend(client)

                # Initialize agent files (templates, directories)
                await initialize_agent_files(
                    backend, agent_name="engineer_coder"
                )  # FIXME: dynamic agent name

                agent, langfuse_callback = create_agent_graph(
                    backend,
                    agent_name="engineer_coder",  # FIXME: dynamic agent name from task/request?
                    # For now hardcoded in original code too.
                    trace_id=trace_id,
                )

                # Add initial trace
                initial_trace = Trace(
                    episode_id=episode_id,
                    trace_type=TraceType.LOG,
                    content=f"Agent starting execution for task: {task}",
                    langfuse_trace_id=trace_id,
                    metadata_vars={"task": task},
                )
                db.add(initial_trace)
                await db.commit()

                # Setup real-time tracing to DB, pass langfuse_callback for linking
                db_callback = DatabaseCallbackHandler(
                    episode_id, langfuse_callback=langfuse_callback
                )

                # Prepare callbacks for agent run
                callbacks = [db_callback]
                if langfuse_callback:
                    callbacks.append(langfuse_callback)

                # Run the agent with tracing
                result = await agent.ainvoke(
                    {"messages": [("user", task)], "session_id": session_id},
                    config={
                        "callbacks": callbacks,
                        "metadata": {"episode_id": str(episode_id)},
                    },
                )

                # Final trace
                final_output = result["messages"][-1].content
                final_trace = Trace(
                    episode_id=episode_id,
                    trace_type=TraceType.LOG,
                    content=f"Agent finished execution: {final_output[:200]}...",
                    langfuse_trace_id=trace_id,
                    metadata_vars={"output": final_output},
                )
                db.add(final_trace)

                # Sync assets from worker
                try:
                    files = await backend.als_info("/")
                    for file_info in files:
                        if not file_info["is_dir"]:
                            path = file_info["path"]
                            asset_type = AssetType.OTHER
                            if path.endswith(".py"):
                                asset_type = AssetType.PYTHON
                            elif path.endswith(".xml") or path.endswith(".mjcf"):
                                asset_type = AssetType.MJCF

                            # Read content for small files
                            content = None
                            with suppress(Exception):
                                content = await backend.aread(path)

                            asset = Asset(
                                episode_id=episode_id,
                                asset_type=asset_type,
                                s3_path=path,
                                content=content,
                            )
                            db.add(asset)
                except Exception as e:
                    logger.error("failed_to_sync_assets", error=str(e))

                # Update episode
                episode = await db.get(Episode, episode_id)
                # Check if it was cancelled in the meantime
                if episode.status != EpisodeStatus.CANCELLED:
                    episode.status = EpisodeStatus.COMPLETED
                    episode.plan = f"Agent completed task: {task}\n\nResult: {final_output[:500]}..."
                    await db.commit()
                    logger.info("agent_run_completed", episode_id=episode_id)

            except asyncio.CancelledError:
                logger.info("agent_run_cancelled", episode_id=episode_id)
                # Re-fetch to ensure session is valid
                episode = await db.get(Episode, episode_id)
                if episode:
                    episode.status = EpisodeStatus.CANCELLED
                    final_trace = Trace(
                        episode_id=episode_id,
                        trace_type=TraceType.LOG,
                        content="Episode cancelled by user",
                    )
                    db.add(final_trace)
                    await db.commit()
                raise  # Re-raise to ensure proper task cancellation
            except Exception as e:
                # We reuse the existing session if possible, or handle generic error
                logger.error("agent_run_failed", error=str(e), episode_id=episode_id)
                episode = await db.get(Episode, episode_id)
                if episode:
                    episode.status = EpisodeStatus.FAILED
                    await db.commit()

    finally:
        # Always remove task from tracker
        task_tracker.remove_task(episode_id)


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
