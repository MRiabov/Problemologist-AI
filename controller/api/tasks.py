import asyncio
import datetime
import os
import uuid
from contextlib import suppress
from typing import Any

import boto3
from langchain_core.messages import HumanMessage

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
from shared.enums import (
    AgentName,
    AssetType,
    EpisodeStatus,
    FailureClass,
    TerminalReason,
    TraceType,
)
from shared.logging import get_logger
from shared.models.schemas import EpisodeMetadata

logger = get_logger(__name__)
WORKER_LIGHT_URL = settings.worker_light_url
SYSTEM_TOOL_RETRY_EXHAUSTED_MARKER = "SYSTEM_TOOL_RETRY_EXHAUSTED"


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
    return bool(_extract_result_field(result, "entry_validation_terminal"))


def _extract_entry_validation_context(result: Any) -> dict[str, Any]:
    from shared.models.schemas import EntryValidationContext

    raw_errors = _extract_result_field(result, "entry_validation_errors")
    errors: list[dict[str, Any]] = []
    if isinstance(raw_errors, list):
        for item in raw_errors:
            if isinstance(item, dict):
                errors.append(dict(item))
            else:
                errors.append({"message": str(item)})

    node = _extract_result_field(result, "entry_validation_target_node")
    disposition = _extract_result_field(result, "entry_validation_disposition")
    reason_code = _extract_result_field(result, "entry_validation_reason_code")
    reroute_target = _extract_result_field(result, "entry_validation_reroute_target")

    # Keep entry-validation metadata stable even if later nodes clear transient state.
    if node is None or disposition is None or reason_code is None:
        import re

        feedback = _result_feedback(result)
        match = re.search(
            r"ENTRY_VALIDATION_FAILED\[(?P<reason>[^\]]+)\]\s+"
            r"target=(?P<node>[^\s]+)\s+"
            r"(?:(?:reroute=(?P<reroute>[^\s]+))|(?:disposition=(?P<disposition>[^\s]+)))"
            r"\s+\|\s*(?P<detail>.*)$",
            feedback,
        )
        if match:
            reason_code = reason_code or match.group("reason")
            node = node or match.group("node")
            disposition = disposition or match.group("disposition")
            reroute_target = reroute_target or match.group("reroute")

            if not errors:
                detail = (match.group("detail") or "").strip()
                if detail:
                    errors.append(
                        {
                            "code": reason_code or "state_invalid",
                            "message": detail,
                            "source": "policy",
                            "artifact_path": None,
                        }
                    )

        if disposition is None:
            journal = str(_extract_result_field(result, "journal") or "")
            journal_match = re.search(
                r"\[Entry Validation\].*disposition=(?P<disposition>[^\s]+)",
                journal,
            )
            if journal_match:
                disposition = journal_match.group("disposition")

    raw_context = {
        "node": node,
        "disposition": disposition,
        "reason_code": reason_code,
        "reroute_target": reroute_target,
        "errors": errors,
    }
    try:
        return EntryValidationContext.model_validate(raw_context).model_dump(
            mode="json"
        )
    except Exception:
        return raw_context


def _update_episode_entry_validation_metadata(
    *, episode: Episode, result: Any, result_feedback: str
) -> dict[str, Any] | None:
    if not result_feedback.startswith("ENTRY_VALIDATION_FAILED["):
        return None

    metadata = EpisodeMetadata.model_validate(episode.metadata_vars or {})
    if result_feedback not in metadata.validation_logs:
        metadata.validation_logs.append(result_feedback)

    entry_context = _extract_entry_validation_context(result)
    additional = dict(metadata.additional_info or {})
    additional["entry_validation"] = entry_context
    additional["entry_validation_terminal"] = (
        bool(_extract_result_field(result, "entry_validation_terminal"))
        or entry_context.get("disposition") == "fail_fast"
    )
    metadata.additional_info = additional
    episode.metadata_vars = metadata.model_dump()
    return entry_context


async def _episode_has_system_tool_retry_exhausted(
    *, db, episode_id: uuid.UUID
) -> bool:
    """Detect infra retry exhaustion marker from any persisted trace payload."""
    from sqlalchemy import select

    rows = await db.execute(select(Trace).where(Trace.episode_id == episode_id))
    for trace_row in rows.scalars():
        metadata = trace_row.metadata_vars or {}
        error = str(metadata.get("error") or "")
        # Retry-exhaustion marker may appear in different trace families depending
        # on where the failure is surfaced (tool start/end, error log, etc.).
        if (
            SYSTEM_TOOL_RETRY_EXHAUSTED_MARKER in error
            or SYSTEM_TOOL_RETRY_EXHAUSTED_MARKER in str(trace_row.content or "")
            or SYSTEM_TOOL_RETRY_EXHAUSTED_MARKER in str(metadata)
        ):
            return True
    return False


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
    start_node: AgentName | None = None,
):
    session_factory = get_sessionmaker()

    try:
        async with session_factory() as db:
            episode = await db.get(Episode, episode_id)
            if not episode:
                logger.error(
                    "episode_not_found_for_background_task",
                    episode_id=episode_id,
                    session_id=session_id,
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
                                    # Copy as a system operation: benchmark-to-engineer
                                    # handoff must not be gated by agent-role FS policy.
                                    await client.write_file(
                                        asset.s3_path,
                                        asset.content,
                                        overwrite=True,
                                        bypass_agent_permissions=True,
                                    )
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
                            session_id=session_id,
                            benchmark_id=benchmark_id_str,
                            error=str(e),
                        )

                agent, langfuse_callback = create_agent_graph(
                    agent_name=agent_name,
                    # For now hardcoded in original code too.
                    trace_id=trace_id,
                    session_id=session_id,
                    start_node=start_node,
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

                # Prepare callbacks list and add Langfuse callback if present.
                callbacks = [db_callback]
                if langfuse_callback:
                    callbacks.append(langfuse_callback)

                # Prepare initial state based on agent type
                if agent_name in [
                    AgentName.ENGINEER_PLANNER,
                    AgentName.ENGINEER_CODER,
                    AgentName.ENGINEER_PLAN_REVIEWER,
                    AgentName.ENGINEER_EXECUTION_REVIEWER,
                    AgentName.COTS_SEARCH,
                ]:
                    initial_input = {
                        "task": task,
                        "session_id": session_id,
                        "episode_id": str(episode_id),
                        "start_node": start_node.value if start_node else None,
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
                    "start_node": start_node.value if start_node else None,
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
                if result_feedback.startswith("ENTRY_VALIDATION_FAILED[") and not bool(
                    _extract_result_field(result, "entry_validation_trace_emitted")
                ):
                    entry_context = _extract_entry_validation_context(result)
                    trace_metadata = {
                        "source": "node_entry_validation",
                        "status": _result_status_lower(result),
                        **entry_context,
                    }
                    db.add(
                        Trace(
                            episode_id=episode_id,
                            trace_type=TraceType.EVENT,
                            name="node_entry_validation_failed",
                            content=result_feedback,
                            langfuse_trace_id=trace_id,
                            metadata_vars=trace_metadata,
                        )
                    )
                    db.add(
                        Trace(
                            episode_id=episode_id,
                            trace_type=TraceType.ERROR,
                            name="node_entry_validation_failed",
                            content=result_feedback,
                            langfuse_trace_id=trace_id,
                            metadata_vars=trace_metadata,
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
                            logger.error(
                                "failed_to_init_s3_client",
                                error=str(e),
                                session_id=session_id,
                            )

                    async def sync_dir(dir_path: str):
                        try:
                            # Use new session for sync to avoid sharing with main loop
                            files = await client.list_files(dir_path)
                        except Exception as e:
                            logger.error(
                                "failed_to_list_dir_during_sync",
                                dir=dir_path,
                                error=str(e),
                                session_id=session_id,
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
                                            logger.error(
                                                "failed_to_read_blob_for_upload",
                                                path=path,
                                                error=str(e),
                                                session_id=session_id,
                                            )
                                except Exception as e:
                                    logger.error(
                                        "failed_to_upload_asset_to_s3",
                                        path=path,
                                        error=str(e),
                                        session_id=session_id,
                                    )

                    simulation_success_text: str | None = None
                    await sync_dir("/")
                    # .manifests is system-owned metadata and may be denied via role
                    # policy through routed backend calls. Read directly with
                    # permission bypass so reviewer manifests still appear in assets.
                    try:
                        from controller.observability.middleware_helper import (
                            broadcast_file_update,
                        )

                        manifest_paths = (
                            ".manifests/benchmark_review_manifest.json",
                            ".manifests/engineering_plan_review_manifest.json",
                            ".manifests/engineering_execution_review_manifest.json",
                            ".manifests/electronics_review_manifest.json",
                        )
                        for manifest_path in manifest_paths:
                            if await client.exists(
                                manifest_path, bypass_agent_permissions=True
                            ):
                                manifest_content = await client.read_file(
                                    manifest_path, bypass_agent_permissions=True
                                )
                                await broadcast_file_update(
                                    str(episode_id), manifest_path, manifest_content
                                )
                    except Exception:
                        pass
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
                    logger.error(
                        "failed_to_sync_assets",
                        error=str(e),
                        session_id=str(episode_id),
                    )

                # Mark completion after traces/assets are persisted.
                await db.refresh(episode)
                if episode.status != EpisodeStatus.CANCELLED:
                    result_feedback = _result_feedback(result)
                    _update_episode_entry_validation_metadata(
                        episode=episode,
                        result=result,
                        result_feedback=result_feedback,
                    )
                    system_retry_exhausted = (
                        await _episode_has_system_tool_retry_exhausted(
                            db=db, episode_id=episode_id
                        )
                    )
                    target_status = EpisodeStatus.FAILED
                    if not _is_failed_result(result):
                        target_status = (
                            EpisodeStatus.PLANNED
                            if _is_planner_agent(agent_name)
                            else EpisodeStatus.COMPLETED
                        )
                    if system_retry_exhausted:
                        target_status = EpisodeStatus.FAILED
                        metadata = EpisodeMetadata.model_validate(
                            episode.metadata_vars or {}
                        )
                        metadata.terminal_reason = (
                            TerminalReason.SYSTEM_TOOL_RETRY_EXHAUSTED
                        )
                        metadata.failure_class = FailureClass.INFRA_DEVOPS_FAILURE
                        metadata.error = SYSTEM_TOOL_RETRY_EXHAUSTED_MARKER
                        episode.metadata_vars = metadata.model_dump(mode="json")
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
                    session_id=session_id,
                )
                episode = await db.get(Episode, episode_id)
                if episode:
                    episode.status = EpisodeStatus.FAILED
                    metadata = EpisodeMetadata.model_validate(
                        episode.metadata_vars or {}
                    )
                    if SYSTEM_TOOL_RETRY_EXHAUSTED_MARKER in str(e):
                        metadata.terminal_reason = (
                            TerminalReason.SYSTEM_TOOL_RETRY_EXHAUSTED
                        )
                        metadata.failure_class = FailureClass.INFRA_DEVOPS_FAILURE
                        metadata.error = SYSTEM_TOOL_RETRY_EXHAUSTED_MARKER
                        episode.metadata_vars = metadata.model_dump(mode="json")
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
                    "episode_not_found_for_continuation",
                    episode_id=episode_id,
                    session_id=str(episode_id),
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

                    with suppress(Exception):
                        # Validate metadata shape if present.
                        EpisodeMetadata.model_validate(episode.metadata_vars)

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
                if result_feedback.startswith("ENTRY_VALIDATION_FAILED[") and not bool(
                    _extract_result_field(result, "entry_validation_trace_emitted")
                ):
                    entry_context = _extract_entry_validation_context(result)
                    trace_metadata = {
                        "source": "node_entry_validation",
                        "status": _result_status_lower(result),
                        **entry_context,
                    }
                    db.add(
                        Trace(
                            episode_id=episode_id,
                            trace_type=TraceType.EVENT,
                            name="node_entry_validation_failed",
                            content=result_feedback,
                            langfuse_trace_id=trace_id,
                            metadata_vars=trace_metadata,
                        )
                    )
                    db.add(
                        Trace(
                            episode_id=episode_id,
                            trace_type=TraceType.ERROR,
                            name="node_entry_validation_failed",
                            content=result_feedback,
                            langfuse_trace_id=trace_id,
                            metadata_vars=trace_metadata,
                        )
                    )

                # Update status
                await db.refresh(episode)
                if episode.status != EpisodeStatus.CANCELLED:
                    result_feedback = _result_feedback(result)
                    _update_episode_entry_validation_metadata(
                        episode=episode,
                        result=result,
                        result_feedback=result_feedback,
                    )
                    system_retry_exhausted = (
                        await _episode_has_system_tool_retry_exhausted(
                            db=db, episode_id=episode_id
                        )
                    )
                    episode.status = (
                        EpisodeStatus.FAILED
                        if (_is_failed_result(result) or system_retry_exhausted)
                        else EpisodeStatus.COMPLETED
                    )
                    if system_retry_exhausted:
                        metadata = EpisodeMetadata.model_validate(
                            episode.metadata_vars or {}
                        )
                        metadata.terminal_reason = (
                            TerminalReason.SYSTEM_TOOL_RETRY_EXHAUSTED
                        )
                        metadata.failure_class = FailureClass.INFRA_DEVOPS_FAILURE
                        metadata.error = SYSTEM_TOOL_RETRY_EXHAUSTED_MARKER
                        episode.metadata_vars = metadata.model_dump(mode="json")
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
                    session_id=session_id,
                )
                episode = await db.get(Episode, episode_id)
                if episode:
                    episode.status = EpisodeStatus.FAILED
                    metadata = EpisodeMetadata.model_validate(
                        episode.metadata_vars or {}
                    )
                    if SYSTEM_TOOL_RETRY_EXHAUSTED_MARKER in str(e):
                        metadata.terminal_reason = (
                            TerminalReason.SYSTEM_TOOL_RETRY_EXHAUSTED
                        )
                        metadata.failure_class = FailureClass.INFRA_DEVOPS_FAILURE
                        metadata.error = SYSTEM_TOOL_RETRY_EXHAUSTED_MARKER
                        episode.metadata_vars = metadata.model_dump(mode="json")
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
