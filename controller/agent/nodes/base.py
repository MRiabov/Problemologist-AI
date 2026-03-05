import ast
import asyncio
import json
import re
from collections.abc import Callable
from contextlib import suppress
from dataclasses import dataclass
from pathlib import Path
from typing import TYPE_CHECKING, Any

import dspy
import structlog

from controller.agent.config import settings
from controller.agent.prompt_manager import PromptManager
from controller.clients.worker import WorkerClient
from controller.middleware.remote_fs import RemoteFilesystemMiddleware
from controller.observability.database import DatabaseCallbackHandler
from controller.observability.langfuse import init_tracing
from controller.persistence.models import Trace
from shared.enums import AgentName, TraceType
from shared.models.schemas import CodeReference, TraceMetadata

if TYPE_CHECKING:
    from controller.agent.state import AgentState

logger = structlog.get_logger(__name__)


@dataclass
class SharedNodeContext:
    """Shared dependencies and state for agent nodes."""

    worker_light_url: str
    session_id: str  # Langfuse/Worker session (string)
    episode_id: str  # Database episode ID (UUID string)
    pm: PromptManager
    dspy_lm: Any
    worker_client: WorkerClient
    fs: RemoteFilesystemMiddleware
    main_loop: asyncio.AbstractEventLoop

    @classmethod
    def create(
        cls,
        worker_light_url: str,
        session_id: str,
        episode_id: str | None = None,
        worker_client: WorkerClient | None = None,
        fs: RemoteFilesystemMiddleware | None = None,
        agent_role: AgentName = AgentName.ENGINEER_CODER,
    ) -> "SharedNodeContext":
        main_loop = asyncio.get_running_loop()
        # Fallback for episode_id if not provided
        eid = episode_id or session_id

        if not worker_client:
            worker_client = WorkerClient(
                base_url=worker_light_url,
                session_id=session_id,
                heavy_url=settings.worker_heavy_url,
            )

        if not fs:
            fs = RemoteFilesystemMiddleware(worker_client, agent_role=agent_role)

        # WP05: Support mock LLMs for integration tests
        if settings.is_integration_test:
            from controller.agent.mock_llm import MockDSPyLM

            logger.info("using_mock_llms_for_integration_test", session_id=session_id)
            dspy_lm = MockDSPyLM(session_id=session_id)
        else:
            # T025: Initialize native tracing
            init_tracing()

            # T012: Initialize DSPy LM for Agent support
            api_key = "dummy"
            if settings.openai_api_key:
                api_key = settings.openai_api_key

            dspy_lm = dspy.LM(
                f"openai/{settings.llm_model}",
                api_key=api_key,
                cache=False,
                timeout=settings.llm_timeout_seconds,
                max_tokens=settings.llm_max_tokens,
            )

        return cls(
            worker_light_url=worker_light_url,
            session_id=session_id,
            episode_id=eid,
            pm=PromptManager(),
            dspy_lm=dspy_lm,
            worker_client=worker_client,
            fs=fs,
            main_loop=main_loop,
        )

    def get_database_recorder(
        self, episode_id: str | None = None
    ) -> DatabaseCallbackHandler:
        """Creates a database recorder for traces."""
        eid = episode_id or self.episode_id
        return DatabaseCallbackHandler(episode_id=eid, loop=self.main_loop)

    def get_callbacks(self) -> list[DatabaseCallbackHandler]:
        """WP08: Support manual callback retrieval for integration tests."""
        recorder = self.get_database_recorder()
        return [recorder]


class BaseNode:
    """Base class for agent nodes providing common utilities."""

    def __init__(self, context: SharedNodeContext):
        self.ctx = context

    def _get_tool_functions(
        self,
        tool_factory: Callable,
        db_callback: DatabaseCallbackHandler | None = None,
        node_name: AgentName | None = None,
    ) -> dict[str, Callable]:
        """Collects tool functions for DSPy compatibility."""
        tools = tool_factory(self.ctx.fs, self.ctx.session_id)
        tool_fns = {}
        live_reasoning_step_idx = 0

        def extract_live_thought(*args: Any, **kwargs: Any) -> str | None:
            """Try to pull a current-thought string from ReAct tool call payload."""
            direct_keys = ("next_thought", "thought", "reasoning")
            for key in direct_keys:
                value = kwargs.get(key)
                if isinstance(value, str) and value.strip():
                    return value.strip()

            for arg in args:
                if isinstance(arg, dict):
                    for key in direct_keys:
                        value = arg.get(key)
                        if isinstance(value, str) and value.strip():
                            return value.strip()

            return None

        for t in tools:
            # Now tools are already raw functions or wrapped in search_cots_catalog
            name = getattr(t, "name", t.__name__ if hasattr(t, "__name__") else str(t))

            import functools

            def make_traced(func, tool_name):
                @functools.wraps(func)
                def sync_wrapper(*args, **kwargs):
                    nonlocal live_reasoning_step_idx
                    if db_callback and node_name:
                        live_thought = extract_live_thought(*args, **kwargs)
                        if live_thought:
                            db_callback.record_reasoning_text_sync(
                                node_name=str(node_name),
                                reasoning_text=live_thought,
                                step_index=live_reasoning_step_idx,
                                source="live_tool_loop",
                            )
                            live_reasoning_step_idx += 1

                    # WP10: Explicitly record tool start
                    trace_id = 0
                    if db_callback:
                        try:
                            # ReAct often passes complex objects, try to
                            # serialize or stringify
                            input_dict = {}
                            if args:
                                input_dict["args"] = [str(a) for a in args]
                            if kwargs:
                                input_dict["kwargs"] = {
                                    k: str(v) for k, v in kwargs.items()
                                }
                            input_str = json.dumps(input_dict)
                        except Exception:
                            input_str = str(args) + " " + str(kwargs)

                        # WP10: Special handling for common tools to ensure success
                        # in mocks
                        if tool_name == "write_file":
                            if "overwrite" not in kwargs:
                                kwargs["overwrite"] = True
                        elif tool_name == "edit_file":
                            # Ensure we don't fail on missing files if we can help it
                            pass

                        try:
                            # WP10: Use synchronous wrapper which handles loop isolation
                            trace_id = db_callback.record_tool_start_sync(
                                tool_name, input_str
                            )
                        except Exception as e:
                            logger.warning("tool_start_trace_failed", error=str(e))

                    try:
                        if asyncio.iscoroutinefunction(func):
                            # WP10: Use run_coroutine_threadsafe on the main loop
                            future = asyncio.run_coroutine_threadsafe(
                                func(*args, **kwargs), self.ctx.main_loop
                            )
                            result = future.result(timeout=60)
                        else:
                            result = func(*args, **kwargs)

                        # WP10: Explicitly record tool success
                        if db_callback and trace_id:
                            db_callback.record_tool_end_sync(
                                trace_id, self._serialize_tool_observation(result)
                            )

                        return result
                    except Exception as e:
                        # WP10: Explicitly record tool error
                        if db_callback and trace_id:
                            db_callback.record_tool_end_sync(
                                trace_id, str(e), is_error=True
                            )
                        raise e

                return sync_wrapper

            wrapped = make_traced(t, name)
            wrapped.__name__ = name
            tool_fns[name] = wrapped
        return tool_fns

    @staticmethod
    def _serialize_tool_observation(result: Any) -> str:
        """Serialize tool outputs to a parseable string for downstream gating."""
        with suppress(Exception):
            if hasattr(result, "model_dump"):
                dumped = result.model_dump(mode="json")
                return json.dumps(dumped)
            if (
                isinstance(result, (dict, list, bool, int, float, str))
                or result is None
            ):
                return json.dumps(result)
        return str(result)

    def _get_database_recorder(self, session_id: str) -> DatabaseCallbackHandler:
        return self.ctx.get_database_recorder(session_id)

    async def _get_latest_submit_plan_result(
        self, expected_node_type: AgentName
    ) -> tuple[Any | None, str | None]:
        """
        Read latest submit_plan result from persisted traces for this episode.

        Returns (PlannerSubmissionResult | None, error_message | None).
        """
        from sqlalchemy import select

        from shared.models.schemas import PlannerSubmissionResult

        episode_id = getattr(self.ctx, "episode_id", None)
        if not episode_id:
            return None, "missing episode_id for submission trace lookup"

        recorder = self.ctx.get_database_recorder(episode_id)
        async with recorder.session_factory() as db:
            query = (
                select(Trace)
                .where(
                    Trace.episode_id == recorder.episode_id,
                    Trace.trace_type == TraceType.TOOL_START,
                    Trace.name == "submit_plan",
                )
                .order_by(Trace.id.desc())
                .limit(1)
            )
            result = await db.execute(query)
            trace_row = result.scalars().first()

        if trace_row is None:
            return None, "submit_plan() tool trace not found"

        metadata_vars = trace_row.metadata_vars or {}
        observation_raw = metadata_vars.get("observation")
        if not observation_raw:
            error_raw = metadata_vars.get("error")
            if isinstance(error_raw, str) and error_raw.strip():
                return None, f"submit_plan() execution failed: {error_raw.strip()}"
            return None, "submit_plan() observation is missing"

        payload: dict[str, Any] | None = None
        if isinstance(observation_raw, dict):
            payload = observation_raw
        elif isinstance(observation_raw, str):
            text = observation_raw.strip()
            with suppress(Exception):
                parsed = ast.literal_eval(text)
                if isinstance(parsed, dict):
                    payload = parsed

        if payload is None:
            return None, "submit_plan() observation is not a structured payload"

        with suppress(Exception):
            submission = PlannerSubmissionResult.model_validate(payload)
            if submission.node_type != expected_node_type:
                return (
                    None,
                    "submit_plan() node_type mismatch: "
                    f"expected {expected_node_type.value}, got {submission.node_type.value}",
                )
            return submission, None
        return None, "submit_plan() observation does not match PlannerSubmissionResult"

    async def _get_latest_tool_trace_metadata(
        self, tool_name: str
    ) -> tuple[TraceMetadata | None, str | None]:
        """
        Read the latest TOOL_START trace metadata for a given tool name.

        Returns (TraceMetadata | None, error_message | None).
        """
        from sqlalchemy import select

        episode_id = getattr(self.ctx, "episode_id", None)
        if not episode_id:
            return None, "missing episode_id for tool trace lookup"

        recorder = self.ctx.get_database_recorder(episode_id)
        async with recorder.session_factory() as db:
            query = (
                select(Trace)
                .where(
                    Trace.episode_id == recorder.episode_id,
                    Trace.trace_type == TraceType.TOOL_START,
                    Trace.name == tool_name,
                )
                .order_by(Trace.id.desc())
                .limit(1)
            )
            result = await db.execute(query)
            trace_row = result.scalars().first()

        if trace_row is None:
            return None, f"{tool_name}() tool trace not found"

        metadata = TraceMetadata.model_validate(trace_row.metadata_vars or {})
        if metadata.error and metadata.error.strip():
            return None, f"{tool_name}() execution failed: {metadata.error.strip()}"
        if not metadata.observation or not metadata.observation.strip():
            return None, f"{tool_name}() observation is missing"
        return metadata, None

    def _get_skills_context(self) -> str:
        # Note: Skills are currently local to the controller
        skills_dir = Path(".agent/skills")
        skills = []
        if skills_dir.exists():
            skills = [d.name for d in skills_dir.iterdir() if d.is_dir()]
        return "\n".join([f"- {s}" for s in skills])

    async def _get_steer_context(self, messages: list[Any]) -> str:
        if not messages:
            return ""
        last_msg = messages[-1]
        steer_data = (
            last_msg.additional_kwargs.get("steerability")
            if hasattr(last_msg, "additional_kwargs")
            else None
        )

        lines = []

        # T015: Parse @file:line-line from message content if not
        # already in code_references
        content = last_msg.content if isinstance(last_msg.content, str) else ""
        # Matches @path/to/file.ext:start-end
        matches = re.finditer(r"@([\w\./\-]+\.\w+):(\d+)-(\d+)", content)
        parsed_refs = []
        for match in matches:
            parsed_refs.append(
                CodeReference(
                    file_path=match.group(1),
                    start_line=int(match.group(2)),
                    end_line=int(match.group(3)),
                )
            )

        if not steer_data and not parsed_refs:
            return ""

        steer_data = steer_data or {}
        raw_code_refs = steer_data.get("code_references", [])
        code_refs = [
            CodeReference.model_validate(r) if isinstance(r, dict) else r
            for r in raw_code_refs
        ] + parsed_refs

        if steer_data.get("selections"):
            lines.append("Geometric Selections:")
            for sel in steer_data["selections"]:
                center_str = ", ".join([f"{v:.2f}" for v in sel["center"]])
                normal_str = ""
                if sel.get("normal"):
                    normal_val = ", ".join([f"{v:.2f}" for v in sel["normal"]])
                    normal_str = f", Normal: ({normal_val})"
                lines.append(
                    f"  - Level: {sel['level']}, ID: {sel['target_id']}, "
                    f"Center: ({center_str}){normal_str}"
                )

        if code_refs:
            lines.append("Code References:")
            for ref in code_refs:
                file_path = ref.file_path
                start = ref.start_line
                end = ref.end_line
                try:
                    # Resolve snippet from worker
                    file_content = await self.ctx.fs.read_file(file_path)
                    file_lines = file_content.splitlines()
                    if start < 1 or end > len(file_lines) or start > end:
                        lines.append(
                            f"  - File: {file_path}, Lines: {start}-{end} "
                            f"[ERROR: Invalid line range. "
                            f"File has {len(file_lines)} lines]"
                        )
                    else:
                        snippet = "\n".join(file_lines[start - 1 : end])
                        lines.append(f"  - File: {file_path}, Lines: {start}-{end}:")
                        lines.append("```python")
                        lines.append(snippet)
                        lines.append("```")
                except Exception as e:
                    lines.append(
                        f"  - File: {file_path}, Lines: {start}-{end} [ERROR: {e}]"
                    )

        if steer_data.get("mentions"):
            lines.append(f"Mentions: {', '.join(steer_data['mentions'])}")

        return "\n".join(lines)

    async def _run_program(
        self,
        program_cls: type[dspy.Module],
        signature_cls: type[dspy.Signature],
        state: "AgentState",
        inputs: dict[str, Any],
        tool_factory: Callable,
        validate_files: list[str],
        node_type: AgentName,
        max_retries: int | None = None,
    ) -> tuple[Any, dict[str, Any], str]:
        """
        Reusable execution loop for DSPy nodes with retries and validation.
        Returns (prediction, artifacts, journal_entry).
        """
        if max_retries is None:
            max_retries = 1 if settings.is_integration_test else 3

        from controller.agent.dspy_utils import WorkerInterpreter
        from worker_heavy.utils.file_validation import validate_node_output

        # Prepare explicit DB logger for UI events
        db_callback = None
        episode_id = getattr(state, "episode_id", None)

        if not episode_id:
            # Try session.session_id (BenchmarkGeneratorState) or self.ctx.episode_id
            session = getattr(state, "session", None)
            if session:
                episode_id = getattr(session, "session_id", None)

            if not episode_id:
                episode_id = self.ctx.episode_id

        if episode_id and str(episode_id).strip():
            db_callback = DatabaseCallbackHandler(
                episode_id=str(episode_id), loop=self.ctx.main_loop
            )

        interpreter = WorkerInterpreter(
            worker_client=self.ctx.worker_client, session_id=self.ctx.session_id
        )
        tool_fns = self._get_tool_functions(
            tool_factory,
            db_callback=db_callback,
            node_name=node_type,
        )

        # WP07: Inject system instructions from prompts.yaml into signature
        # Template names are now standardized to match node_type or explicit mappings
        instructions = self.ctx.pm.render(node_type)
        if instructions:
            # If using ReAct, prepend our instructions to the existing ones
            # to preserve the "Action: tool_name(args)" structural prompt.
            current_instr = getattr(signature_cls, "instructions", "")
            if program_cls is dspy.ReAct:
                # Prepend for context, but keep structural instructions at the end
                signature_cls = signature_cls.with_instructions(
                    f"{instructions}\n\n{current_instr}"
                )
            else:
                signature_cls = signature_cls.with_instructions(instructions)

        if program_cls is dspy.ReAct:
            max_iters = settings.react_max_iters
            if node_type in {
                AgentName.ENGINEER_PLANNER,
                AgentName.ELECTRONICS_PLANNER,
                AgentName.BENCHMARK_PLANNER,
            }:
                max_iters = settings.react_planner_max_iters
            program = program_cls(
                signature_cls, tools=list(tool_fns.values()), max_iters=max_iters
            )
        else:
            program = program_cls(signature_cls, tools=list(tool_fns.values()))

        # WP07: Try to load compiled prompt if available
        self.ctx.pm.load_compiled_program(node_type, program)

        # WP08: Set node_type on mock LM if available for explicit lookup
        if hasattr(self.ctx.dspy_lm, "node_type"):
            self.ctx.dspy_lm.node_type = node_type

        retry_count = 0
        journal_entry = ""
        prediction = None
        artifacts = {}

        dspy_timeout = float(settings.dspy_program_timeout_seconds)

        try:
            while retry_count < max_retries:
                try:
                    # WP10: Explicitly record node start for UI
                    if db_callback:
                        node_input = (
                            inputs.get("task")
                            or inputs.get("prompt")
                            or inputs.get("journal")
                            or ""
                        )
                        await db_callback.record_node_start(
                            node_type, input_data=str(node_input)
                        )

                    with dspy.settings.context(lm=self.ctx.dspy_lm):
                        logger.info(
                            f"{node_type}_dspy_invoke_start",
                            session_id=self.ctx.session_id,
                        )
                        # Add a timeout to prevent infinite hangs in DSPy ReAct
                        try:
                            prediction = await asyncio.wait_for(
                                asyncio.to_thread(program, **inputs),
                                timeout=dspy_timeout,
                            )
                            logger.info(
                                f"{node_type}_raw_prediction",
                                prediction=str(prediction),
                                prediction_type=str(type(prediction)),
                                session_id=self.ctx.session_id,
                            )
                        except TimeoutError as err:
                            logger.error(
                                f"{node_type}_dspy_timeout",
                                session_id=self.ctx.session_id,
                            )
                            msg = (
                                f"DSPy program {node_type} "
                                f"timed out after {dspy_timeout}s"
                            )
                            raise RuntimeError(msg) from err
                        logger.info(
                            f"{node_type}_dspy_invoke_complete",
                            session_id=self.ctx.session_id,
                        )

                    # WP10: Explicitly record node end for UI
                    if db_callback:
                        await db_callback.record_node_end(
                            node_type,
                            output_data=str(prediction),
                            output_obj=prediction,
                        )

                    results = await asyncio.gather(
                        *[self.ctx.fs.read_file(f) for f in validate_files],
                        return_exceptions=True,
                    )
                    artifacts = {
                        f: res
                        for f, res in zip(validate_files, results, strict=False)
                        if not isinstance(res, Exception)
                    }

                    is_valid, validation_errors = validate_node_output(
                        node_type, artifacts
                    )

                    if not is_valid:
                        logger.warning(
                            f"{node_type}_validation_failed", errors=validation_errors
                        )
                        retry_count += 1
                        if retry_count >= max_retries:
                            # Record the last (invalid) attempt for UI before failing
                            if db_callback and prediction:
                                async with db_callback.session_factory() as db:
                                    trace_obj = Trace(
                                        episode_id=db_callback.episode_id,
                                        trace_type=TraceType.ERROR,
                                        name=f"{node_type}_validation_error",
                                        content=(
                                            f"Validation errors: {validation_errors}"
                                        ),
                                        metadata_vars={"prediction": str(prediction)},
                                    )
                                    db.add(trace_obj)
                                    await db.commit()

                            raise ValueError(
                                f"Node {node_type} failed to produce valid "
                                f"output after {max_retries} retries. "
                                f"Errors: {validation_errors}"
                            )
                        await asyncio.sleep(1)  # Backoff to prevent log explosion
                        continue

                    return prediction, artifacts, journal_entry

                except Exception as err:
                    logger.error(f"{node_type}_dspy_failed", error=str(err))
                    journal_entry += f"\n[System Error] {err}"
                    retry_count += 1
                    await asyncio.sleep(1)  # Backoff to prevent log explosion

            journal_entry += "\nMax retries reached."
            return None, artifacts, journal_entry

        finally:
            interpreter.shutdown()
