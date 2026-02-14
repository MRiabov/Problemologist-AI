import asyncio
import uuid
from contextlib import suppress

from pydantic import BaseModel, Field, StrictStr, field_validator

from controller.agent.initialization import initialize_agent_files
from controller.api.manager import task_tracker
from controller.clients.backend import RemoteFilesystemBackend
from controller.config.settings import settings
from controller.graph.agent import create_agent_graph
from controller.middleware.remote_fs import RemoteFilesystemMiddleware
from controller.observability.database import DatabaseCallbackHandler
from controller.persistence.db import get_sessionmaker
from controller.persistence.models import Asset, Episode, Trace
from shared.enums import AssetType, EpisodeStatus, TraceType
from shared.logging import get_logger

logger = get_logger(__name__)
WORKER_URL = settings.worker_url


class AgentRunRequest(BaseModel):
    task: StrictStr = Field(..., description="The task for the agent to perform.")
    session_id: StrictStr = Field(..., description="Session ID for the worker.")
    metadata_vars: dict | None = Field(
        None, description="Additional metadata for the episode."
    )
    skill_git_hash: str | None = Field(
        None, description="Git hash of the skills used for this run."
    )
    agent_name: str = Field(
        "engineer_coder", description="The name of the agent to run."
    )

    @field_validator("task", "session_id", "agent_name")
    @classmethod
    def strip_null_bytes(cls, v: str) -> str:
        return v.replace("\u0000", "")


def get_worker_client(session_id: str):
    from controller.clients.worker import WorkerClient

    return WorkerClient(base_url=WORKER_URL, session_id=session_id)


async def execute_agent_task(
    episode_id: uuid.UUID,
    task: str,
    session_id: str,
    agent_name: str = "engineer_coder",
):
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
                middleware = RemoteFilesystemMiddleware(client)
                backend = RemoteFilesystemBackend(middleware)

                # Initialize agent files (templates, directories)
                await initialize_agent_files(backend, agent_name=agent_name)

                # If benchmark_id is present, copy benchmark assets to the session
                if episode.metadata_vars and "benchmark_id" in episode.metadata_vars:
                    benchmark_id_str = episode.metadata_vars["benchmark_id"]
                    try:
                        benchmark_id = uuid.UUID(benchmark_id_str)
                        async with session_factory() as db_inner:
                            from sqlalchemy import select

                            stmt = select(Asset).where(Asset.episode_id == benchmark_id)
                            res = await db_inner.execute(stmt)
                            benchmark_assets = res.scalars().all()

                            for asset in benchmark_assets:
                                if asset.content:
                                    # Copy to worker
                                    await backend.awrite(asset.s3_path, asset.content)
                                    logger.info(
                                        "copied_benchmark_asset",
                                        episode_id=episode_id,
                                        benchmark_id=benchmark_id,
                                        path=asset.s3_path,
                                    )
                    except Exception as e:
                        logger.error(
                            "failed_to_copy_benchmark_assets",
                            episode_id=episode_id,
                            benchmark_id=benchmark_id_str,
                            error=str(e),
                        )

                agent, langfuse_callback = create_agent_graph(
                    backend,
                    agent_name=agent_name,
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
                        "run_name": agent_name,
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

                    async def sync_dir(dir_path: str):
                        files = await client.list_files(dir_path)
                        for file_info in files:
                            path = file_info.path
                            if file_info.is_dir:
                                # Recursively sync directories, but skip some obvious ones
                                if not any(
                                    path.endswith(s)
                                    for s in ["__pycache__", ".git", ".venv", "renders"]
                                ):
                                    await sync_dir(path)
                                continue

                            asset_type = None
                            if path.endswith(".py"):
                                asset_type = AssetType.PYTHON
                            elif path.endswith((".xml", ".mjcf")):
                                asset_type = AssetType.MJCF
                            elif path.endswith(".glb"):
                                asset_type = AssetType.GLB
                            elif path.endswith(".stl"):
                                asset_type = AssetType.STL
                            elif path.endswith((".png", ".jpg", ".jpeg", ".webp")):
                                asset_type = AssetType.IMAGE
                            else:
                                asset_type = AssetType.OTHER

                            # Read content
                            content = None
                            # Only store content in DB for text-like or small metadata files
                            if asset_type in [
                                AssetType.PYTHON,
                                AssetType.MJCF,
                                AssetType.OTHER,
                            ]:
                                with suppress(Exception):
                                    raw_content = await client.read_file(path)
                                    content = raw_content

                            # For GLB/STL/IMAGE, we don't store content in 'content' column (it's for text)
                            # In a real S3 setup, we'd have a real S3 URL in s3_path.
                            # For now, s3_path is used by the frontend to fetch from worker via /api/v1/episodes/{id}/assets/{path}

                            asset = Asset(
                                episode_id=episode_id,
                                asset_type=asset_type,
                                s3_path=path,
                                content=content,
                            )
                            db.add(asset)

                    await sync_dir("/")
                    await db.commit()
                except Exception as e:
                    logger.error("failed_to_sync_assets", error=str(e))

                # Update episode
                await db.refresh(episode)
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
                import traceback

                logger.error(
                    "agent_run_failed",
                    error=str(e),
                    traceback=traceback.format_exc(),
                    episode_id=episode_id,
                )
                episode = await db.get(Episode, episode_id)
                if episode:
                    episode.status = EpisodeStatus.FAILED
                    await db.commit()

    finally:
        # Always remove task from tracker
        task_tracker.remove_task(episode_id)
