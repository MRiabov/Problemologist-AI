import asyncio
import datetime
import os
import uuid
from typing import Any

import boto3
from langchain_core.messages import HumanMessage
from pydantic import BaseModel, Field, StrictStr, field_validator

from controller.agent.initialization import initialize_agent_files
from controller.api.manager import task_tracker
from controller.clients.backend import RemoteFilesystemBackend
from controller.config.settings import settings
from controller.graph.agent import create_agent_graph
from controller.middleware.remote_fs import RemoteFilesystemMiddleware
from controller.observability.database import DatabaseCallbackHandler
from controller.observability.langfuse import (
    attach_session_to_current_trace,
    start_root_span,
)
from controller.persistence.db import get_sessionmaker
from controller.persistence.models import Asset, Episode, Trace
from shared.enums import AgentName, AssetType, EpisodeStatus, TraceType
from shared.logging import get_logger

logger = get_logger(__name__)
WORKER_LIGHT_URL = settings.worker_light_url


def _is_planner_agent(agent_name: AgentName) -> bool:
    return agent_name in {
        AgentName.ENGINEER_PLANNER,
        AgentName.ELECTRONICS_PLANNER,
        AgentName.BENCHMARK_PLANNER,
    }


def _extract_result_field(result: Any, key: str) -> Any | None:
    if isinstance(result, dict):
        return result.get(key)
    return getattr(result, key, None)


def _result_status_lower(result: Any) -> str:
    raw_status = _extract_result_field(result, "status")
    if raw_status is None:
        return ""
    if hasattr(raw_status, "value"):
        raw_status = raw_status.value
    return str(raw_status).strip().lower()


def _result_feedback(result: Any) -> str:
    raw_feedback = _extract_result_field(result, "feedback")
    return str(raw_feedback or "").strip()


def _is_failed_result(result: Any) -> bool:
    status = _result_status_lower(result)
    if status == "failed":
        return True
    return _result_feedback(result).startswith("ENTRY_VALIDATION_FAILED[")


class AgentRunRequest(BaseModel):
    task: StrictStr = Field(..., description="The task for the agent to perform.")
    session_id: StrictStr = Field(..., description="Session ID for the worker.")
    metadata_vars: dict | None = Field(
        None, description="Additional metadata for the episode."
    )
    skill_git_hash: str | None = Field(
        None, description="Git hash of the skills used for this run."
    )
    agent_name: AgentName = Field(
        AgentName.ENGINEER_CODER, description="The name of the agent to run."
    )

    @field_validator("task", "session_id", "skill_git_hash")
    @classmethod
    def strip_null_bytes(cls, v: str | None) -> str | None:
        if v is None:
            return None
        return v.replace("\u0000", "")

    @field_validator("metadata_vars")
    @classmethod
    def strip_null_bytes_in_metadata(cls, v: dict | None) -> dict | None:
        if v is None:
            return None

        def _clean(value):
            if isinstance(value, str):
                return value.replace("\u0000", "")
            if isinstance(value, dict):
                return {
                    (_clean(k) if isinstance(k, str) else k): _clean(val)
                    for k, val in value.items()
                }
            if isinstance(value, list):
                return [_clean(item) for item in value]
            return value

        return _clean(v)


def get_worker_client(session_id: str):
    from controller.clients.worker import WorkerClient

    return WorkerClient(
        base_url=WORKER_LIGHT_URL,
        session_id=session_id,
        heavy_url=settings.worker_heavy_url,
    )


async def execute_agent_task(
    episode_id: uuid.UUID,
    task: str,
    session_id: str,
    agent_name: AgentName = AgentName.ENGINEER_CODER,
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
                middleware = RemoteFilesystemMiddleware(client, agent_role=agent_name)
                backend = RemoteFilesystemBackend(middleware)
                from controller.agent.execution_limits import (
                    mark_episode_execution_window_start,
                )

                await mark_episode_execution_window_start(episode_id)

                # Initialize agent files (templates, directories)
                await initialize_agent_files(backend, agent_name=agent_name)

                # If benchmark_id is present, copy benchmark assets to the session
                from shared.models.schemas import EpisodeMetadata

                metadata = EpisodeMetadata.model_validate(episode.metadata_vars or {})
                if metadata.benchmark_id:
                    benchmark_id_str = metadata.benchmark_id
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
                    agent_name=agent_name,
                    # For now hardcoded in original code too.
                    trace_id=trace_id,
                    session_id=session_id,
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
                await db.refresh(initial_trace)

                # Setup real-time tracing to DB
                db_callback = DatabaseCallbackHandler(
                    episode_id=str(episode_id), loop=asyncio.get_running_loop()
                )

                # Broadcast initial trace (refresh to get ID)
                await db.refresh(initial_trace)
                await db_callback._broadcast_trace(initial_trace)

                # Prepare callbacks list (only langfuse if present, since DB callback is now explicit)
                callbacks = [db_callback]
                if langfuse_callback:
                    callbacks.append(langfuse_callback)

                # Prepare initial state based on agent type
                if (
                    agent_name
                    in [
                        AgentName.ENGINEER_PLANNER,
                        AgentName.ENGINEER_CODER,
                        AgentName.ENGINEER_REVIEWER,
                    ]
                    or agent_name == AgentName.ELECTRONICS_ENGINEER
                ):
                    initial_input = {
                        "task": task,
                        "session_id": session_id,
                        "episode_id": str(episode_id),
                        "messages": [HumanMessage(content=task)],
                    }
                elif agent_name in [
                    AgentName.BENCHMARK_PLANNER,
                    AgentName.BENCHMARK_CODER,
                    AgentName.BENCHMARK_REVIEWER,
                ]:
                    from controller.agent.benchmark.models import (
                        GenerationSession,
                        SessionStatus,
                    )

                    # Try to parse session_id as UUID, fallback to new one
                    try:
                        u_session_id = uuid.UUID(session_id)
                    except ValueError:
                        u_session_id = uuid.uuid4()

                    session = GenerationSession(
                        session_id=u_session_id,
                        prompt=task,
                        status=SessionStatus.PLANNING,
                    )
                    initial_input = {
                        "session": session,
                        "messages": [HumanMessage(content=task)],
                        "current_script": "",
                        "review_round": 0,
                        "episode_id": str(episode_id),
                    }
                else:
                    initial_input = {
                        "messages": [HumanMessage(content=task)],
                        "session_id": session_id,
                        "episode_id": str(episode_id),
                    }

                # Run the agent with tracing
                # Determine thread_id for persistence.
                # Use episode_id as thread_id to allow resuming.
                # But LangGraph might expect string.
                thread_id = str(episode_id)
                langfuse_session_id = (
                    str(episode.user_session_id)
                    if episode.user_session_id
                    else session_id
                )
                langfuse_metadata = {
                    "episode_id": str(episode_id),
                    "worker_session_id": session_id,
                    "agent_name": agent_name,
                }

                with start_root_span(
                    name=agent_name,
                    trace_id=trace_id,
                    input_payload={"task": task, "session_id": session_id},
                    metadata=langfuse_metadata,
                ):
                    attach_session_to_current_trace(
                        langfuse_session_id,
                        trace_name=agent_name,
                        metadata=langfuse_metadata,
                    )
                    config: Any = {
                        "callbacks": callbacks,
                        "metadata": {
                            "episode_id": str(episode_id),
                            "langfuse_trace_id": trace_id,
                            "langfuse_session_id": langfuse_session_id,
                        },
                        "run_name": agent_name,
                        "configurable": {"thread_id": thread_id},
                    }
                    result = await agent.ainvoke(initial_input, config=config)  # type: ignore
                if isinstance(result, dict):
                    msgs = result.get("messages", [])
                    logger.info(
                        "agent_run_completed_messages_info",
                        count=len(msgs),
                        last_msg=str(msgs[-1]) if msgs else "None",
                    )
                    logger.info("agent_run_completed_keys", keys=list(result.keys()))
                else:
                    logger.info(
                        "agent_run_completed_raw_result",
                        result_type=type(result).__name__,
                    )

                # Final trace
                if isinstance(result, dict):
                    final_messages = result.get("messages", [])
                else:
                    final_messages = getattr(result, "messages", [])

                if final_messages:
                    final_output = final_messages[-1].content
                else:
                    final_output = _result_feedback(result) or (
                        "No output produced by agent."
                    )
                final_trace = Trace(
                    episode_id=episode_id,
                    trace_type=TraceType.LOG,
                    content=f"Agent finished execution: {final_output[:200]}...",
                    langfuse_trace_id=trace_id,
                    metadata_vars={"output": final_output},
                )
                db.add(final_trace)

                # Also add a dedicated LLM_END trace for the full final response
                # to ensure it shows in chat
                final_llm_trace = Trace(
                    episode_id=episode_id,
                    trace_type=TraceType.LLM_END,
                    content=final_output,
                    langfuse_trace_id=trace_id,
                )
                db.add(final_llm_trace)

                result_feedback = _result_feedback(result)
                if result_feedback.startswith("ENTRY_VALIDATION_FAILED["):
                    db.add(
                        Trace(
                            episode_id=episode_id,
                            trace_type=TraceType.ERROR,
                            content=result_feedback,
                            langfuse_trace_id=trace_id,
                            metadata_vars={
                                "source": "node_entry_validation",
                                "status": _result_status_lower(result),
                            },
                        )
                    )

                await db.commit()
                await db.refresh(final_llm_trace)

                # Broadcast final message
                await db_callback._broadcast_trace(final_llm_trace)

                # Report automated score to Langfuse
                if langfuse_callback:
                    from controller.observability.langfuse import (
                        calculate_and_report_automated_score,
                    )

                    await calculate_and_report_automated_score(
                        episode_id=episode_id,
                        trace_id=trace_id,
                        agent_name=agent_name,
                        db=db,
                        worker_client=client,
                    )

                # Sync assets from worker (now in background after status update)
                logger.info("starting_asset_sync", episode_id=episode_id)
                try:
                    # Initialize S3 client for this task
                    s3_endpoint = os.getenv("S3_ENDPOINT")
                    s3_access_key = os.getenv("S3_ACCESS_KEY")
                    s3_secret_key = os.getenv("S3_SECRET_KEY")
                    asset_bucket = os.getenv("ASSET_S3_BUCKET", "problemologist")

                    s3_client = None
                    if s3_endpoint and s3_access_key and s3_secret_key:
                        try:
                            s3_client = boto3.client(
                                "s3",
                                endpoint_url=s3_endpoint,
                                aws_access_key_id=s3_access_key,
                                aws_secret_access_key=s3_secret_key,
                            )
                        except Exception as e:
                            logger.error("failed_to_init_s3_client", error=str(e))

                    async def sync_dir(dir_path: str):
                        try:
                            # Use new session for sync to avoid sharing with main loop
                            files = await client.list_files(dir_path)
                        except Exception as e:
                            logger.warning(
                                "failed_to_list_dir_during_sync",
                                dir=dir_path,
                                error=str(e),
                            )
                            return

                        for file_info in files:
                            path = file_info.path
                            if file_info.is_dir:
                                # Recursively sync directories
                                if not any(
                                    path.lower().endswith(s)
                                    for s in ["__pycache__", ".git", ".venv"]
                                ):
                                    await sync_dir(path)
                                continue

                            # Skip obviously huge or irrelevant files
                            if path.endswith((".log", ".lock", ".tmp", ".pyc")):
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

                            # Read content for text assets
                            content = None
                            blob_content = None

                            if asset_type in [
                                AssetType.PYTHON,
                                AssetType.MJCF,
                                AssetType.OTHER,
                            ]:
                                try:
                                    raw_content = await client.read_file(path)
                                    content = raw_content
                                except Exception:
                                    pass

                            from controller.observability.middleware_helper import (
                                broadcast_file_update,
                            )

                            await broadcast_file_update(
                                str(episode_id), path, content or ""
                            )

                            # Upload to S3 if client available
                            if s3_client:
                                try:
                                    if content is not None:
                                        # Upload text content
                                        await asyncio.to_thread(
                                            s3_client.put_object,
                                            Bucket=asset_bucket,
                                            Key=path,
                                            Body=content.encode("utf-8"),
                                        )
                                    else:
                                        # Binary file, read as blob
                                        try:
                                            blob_content = (
                                                await client.read_file_binary(path)
                                            )
                                            await asyncio.to_thread(
                                                s3_client.put_object,
                                                Bucket=asset_bucket,
                                                Key=path,
                                                Body=blob_content,
                                            )
                                        except Exception as e:
                                            logger.warning(
                                                "failed_to_read_blob_for_upload",
                                                path=path,
                                                error=str(e),
                                            )
                                except Exception as e:
                                    logger.error(
                                        "failed_to_upload_asset_to_s3",
                                        path=path,
                                        error=str(e),
                                    )

                    simulation_success_text: str | None = None
                    await sync_dir("/")
                    try:
                        raw_results = await client.read_file("validation_results.json")
                        if raw_results:
                            import json

                            data = json.loads(raw_results)
                            summary = str(data.get("summary", "")).strip()
                            status = str(data.get("status", "")).strip().lower()
                            if status in {"ok", "success"} and summary:
                                simulation_success_text = summary
                    except Exception:
                        pass
                    if not simulation_success_text:
                        try:
                            raw_sim = await client.read_file("simulation_result.json")
                            if raw_sim:
                                import json

                                sim_data = json.loads(raw_sim)
                                summary = str(sim_data.get("summary", "")).strip()
                                if summary:
                                    simulation_success_text = summary
                        except Exception:
                            pass

                    if simulation_success_text:
                        db.add(
                            Trace(
                                episode_id=episode_id,
                                trace_type=TraceType.LOG,
                                content=simulation_success_text,
                                langfuse_trace_id=trace_id,
                            )
                        )
                    await db.commit()
                    logger.info("asset_sync_completed", episode_id=episode_id)
                except Exception as e:
                    logger.error("failed_to_sync_assets", error=str(e))

                # Mark completion after traces/assets are persisted.
                await db.refresh(episode)
                if episode.status != EpisodeStatus.CANCELLED:
                    target_status = EpisodeStatus.FAILED
                    if not _is_failed_result(result):
                        target_status = (
                            EpisodeStatus.PLANNED
                            if _is_planner_agent(agent_name)
                            else EpisodeStatus.COMPLETED
                        )
                    episode.status = target_status
                    if episode.todo_list is None:
                        episode.todo_list = {"completed": True}
                    if not episode.plan:
                        episode.plan = (
                            "# Solution Overview\n"
                            f"Agent completed task: {task}\n\n"
                            "## Parts List\n"
                            f"- Result summary: {final_output[:500]}...\n"
                        )
                    await db.commit()

                    from controller.api.manager import manager

                    await manager.broadcast(
                        episode_id,
                        {
                            "type": "status_update",
                            "status": target_status,
                            "metadata_vars": episode.metadata_vars,
                            "timestamp": datetime.datetime.now(
                                datetime.UTC
                            ).isoformat(),
                        },
                    )

                logger.info("agent_task_logic_completed", episode_id=episode_id)

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

                    # Broadcast status update
                    from controller.api.manager import manager

                    await manager.broadcast(
                        episode_id,
                        {
                            "type": "status_update",
                            "status": EpisodeStatus.FAILED,
                            "metadata_vars": episode.metadata_vars,
                            "timestamp": datetime.datetime.now(
                                datetime.UTC
                            ).isoformat(),
                        },
                    )

    finally:
        # Always remove task from tracker
        task_tracker.remove_task(episode_id)


async def continue_agent_task(
    episode_id: uuid.UUID,
    message: str,
    additional_turns: int = 0,
    metadata: dict | None = None,
):
    """
    Continue an existing agent task with a new user message.
    """
    session_factory = get_sessionmaker()

    try:
        async with session_factory() as db:
            episode = await db.get(Episode, episode_id)
            if not episode:
                logger.error(
                    "episode_not_found_for_continuation", episode_id=episode_id
                )
                return

            # Update status to RUNNING
            if episode.status != EpisodeStatus.RUNNING:
                episode.status = EpisodeStatus.RUNNING
                await db.commit()

            if additional_turns > 0:
                from controller.agent.execution_limits import (
                    grant_episode_additional_turns,
                )

                await grant_episode_additional_turns(
                    episode_id=episode_id,
                    additional_turns=additional_turns,
                )
            from controller.agent.execution_limits import (
                mark_episode_execution_window_start,
            )

            await mark_episode_execution_window_start(episode_id)

            try:
                # Setup context
                trace_id = uuid.uuid4().hex
                session_id = None
                if episode.metadata_vars:
                    from shared.models.schemas import EpisodeMetadata

                    metadata_typed = EpisodeMetadata.model_validate(
                        episode.metadata_vars
                    )
                    session_id = metadata_typed.worker_session_id

                if not session_id:
                    # Fallback
                    session_id = str(episode_id)

                client = get_worker_client(session_id)

                # Try to retrieve agent_name from metadata if available
                agent_name = AgentName.ENGINEER_CODER
                if episode.metadata_vars:
                    from shared.models.schemas import EpisodeMetadata

                    try:
                        # Assuming AgentName might be in metadata_vars or we can infer it
                        # For now, we use metadata to get the original name if stored
                        EpisodeMetadata.model_validate(episode.metadata_vars)
                        # We need a field in metadata for agent_name.
                        # If not present, default to ENGINEER_CODER for continuation.
                    except Exception:
                        pass

                agent, langfuse_callback = create_agent_graph(
                    agent_name=agent_name,
                    trace_id=trace_id,
                    session_id=session_id,
                )

                # Add trace for user message
                user_trace = Trace(
                    episode_id=episode_id,
                    trace_type=TraceType.LOG,
                    content=f"User message: {message}",
                    langfuse_trace_id=trace_id,
                    metadata_vars={
                        "role": "user",
                        "message": message,
                        "metadata": metadata,
                    },
                )
                db.add(user_trace)
                await db.commit()
                await db.refresh(user_trace)

                # Setup callbacks
                db_callback = DatabaseCallbackHandler(
                    episode_id=str(episode_id), loop=asyncio.get_running_loop()
                )
                await db_callback._broadcast_trace(user_trace)

                callbacks = [db_callback]
                if langfuse_callback:
                    callbacks.append(langfuse_callback)

                # Invoke agent with new message
                thread_id = str(episode_id)
                langfuse_session_id = (
                    str(episode.user_session_id)
                    if episode.user_session_id
                    else session_id
                )
                langfuse_metadata = {
                    "episode_id": str(episode_id),
                    "worker_session_id": session_id,
                    "agent_name": agent_name,
                }

                # Support steerability metadata in additional_kwargs
                human_message = HumanMessage(
                    content=message,
                    additional_kwargs={"steerability": metadata} if metadata else {},
                )
                input_update = {
                    "messages": [human_message],
                    "episode_id": str(episode_id),
                }

                with start_root_span(
                    name=agent_name,
                    trace_id=trace_id,
                    input_payload={"message": message, "session_id": session_id},
                    metadata=langfuse_metadata,
                ):
                    attach_session_to_current_trace(
                        langfuse_session_id,
                        trace_name=agent_name,
                        metadata=langfuse_metadata,
                    )
                    result = await agent.ainvoke(
                        input_update,
                        config={
                            "callbacks": callbacks,
                            "metadata": {
                                "episode_id": str(episode_id),
                                "langfuse_trace_id": trace_id,
                                "langfuse_session_id": langfuse_session_id,
                            },
                            "run_name": agent_name,
                            "configurable": {"thread_id": thread_id},
                        },
                    )

                # Final trace
                if isinstance(result, dict):
                    final_messages = result.get("messages", [])
                else:
                    final_messages = getattr(result, "messages", [])

                if final_messages:
                    final_output = final_messages[-1].content
                else:
                    final_output = _result_feedback(result) or (
                        "No output produced by agent."
                    )

                final_trace = Trace(
                    episode_id=episode_id,
                    trace_type=TraceType.LLM_END,
                    content=final_output,
                    langfuse_trace_id=trace_id,
                )
                db.add(final_trace)

                result_feedback = _result_feedback(result)
                if result_feedback.startswith("ENTRY_VALIDATION_FAILED["):
                    db.add(
                        Trace(
                            episode_id=episode_id,
                            trace_type=TraceType.ERROR,
                            content=result_feedback,
                            langfuse_trace_id=trace_id,
                            metadata_vars={
                                "source": "node_entry_validation",
                                "status": _result_status_lower(result),
                            },
                        )
                    )

                # Update status
                await db.refresh(episode)
                if episode.status != EpisodeStatus.CANCELLED:
                    episode.status = (
                        EpisodeStatus.FAILED
                        if _is_failed_result(result)
                        else EpisodeStatus.COMPLETED
                    )
                    if episode.todo_list is None:
                        episode.todo_list = {"completed": True}
                    if not episode.plan:
                        episode.plan = (
                            "# Solution Overview\n"
                            "Continuation completed successfully.\n\n"
                            "## Parts List\n"
                            f"- Result summary: {final_output[:500]}...\n"
                        )

                await db.commit()

                # Broadcast status update
                from controller.api.manager import manager

                await manager.broadcast(
                    episode_id,
                    {
                        "type": "status_update",
                        "status": episode.status,
                        "metadata_vars": episode.metadata_vars,
                        "timestamp": datetime.datetime.now(datetime.UTC).isoformat(),
                    },
                )

                await db_callback._broadcast_trace(final_trace)

                # Report automated score to Langfuse
                if langfuse_callback:
                    from controller.observability.langfuse import (
                        calculate_and_report_automated_score,
                    )

                    await calculate_and_report_automated_score(
                        episode_id=episode_id,
                        trace_id=trace_id,
                        agent_name=agent_name,
                        db=db,
                        worker_client=client,
                    )

                logger.info("agent_continuation_completed", episode_id=episode_id)

            except Exception as e:
                import traceback

                logger.error(
                    "agent_continuation_failed",
                    error=str(e),
                    traceback=traceback.format_exc(),
                    episode_id=episode_id,
                )
                episode = await db.get(Episode, episode_id)
                if episode:
                    episode.status = EpisodeStatus.FAILED
                    await db.commit()

                    # Broadcast status update
                    from controller.api.manager import manager

                    await manager.broadcast(
                        episode_id,
                        {
                            "type": "status_update",
                            "status": EpisodeStatus.FAILED,
                            "metadata_vars": episode.metadata_vars,
                            "timestamp": datetime.datetime.now(
                                datetime.UTC
                            ).isoformat(),
                        },
                    )
    finally:
        pass
