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
from pydantic import TypeAdapter
from sqlalchemy import func, select

from controller.agent.config import settings
from controller.agent.context_usage import update_episode_context_usage
from controller.agent.execution_limits import (
    AgentHardFailError,
    accumulate_episode_credit_usage,
    evaluate_agent_hard_fail,
)
from controller.agent.prompt_manager import PromptManager
from controller.clients.worker import WorkerClient
from controller.middleware.remote_fs import RemoteFilesystemMiddleware
from controller.observability.database import DatabaseCallbackHandler
from controller.observability.langfuse import (
    init_tracing,
    report_usage_to_current_observation,
)
from controller.persistence.models import Trace
from shared.enums import AgentName, TraceType
from shared.models.schemas import CodeReference, TraceMetadata

if TYPE_CHECKING:
    from controller.agent.state import AgentState

logger = structlog.get_logger(__name__)
_SYSTEM_TOOL_RETRY_EXHAUSTED_MARKER = "SYSTEM_TOOL_RETRY_EXHAUSTED"


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

            # T012: Initialize DSPy LM (LiteLLM-backed) for Agent support.
            # Prefer explicit OpenRouter provider wiring over OpenAI-compatible routing.
            api_base = settings.openai_api_base
            use_openrouter = bool(settings.openrouter_api_key) or bool(
                api_base and "openrouter.ai" in api_base
            )
            model_name = settings.llm_model
            if "/" in model_name and model_name.split("/", 1)[0] in {
                "openai",
                "openrouter",
                "anthropic",
                "azure",
                "gemini",
            }:
                litellm_model = model_name
            elif use_openrouter:
                litellm_model = f"openrouter/{model_name}"
            else:
                litellm_model = f"openai/{model_name}"

            api_key = (
                (settings.openrouter_api_key or settings.openai_api_key)
                if use_openrouter
                else settings.openai_api_key
            ) or "dummy"

            # Respect repository configuration (config/agents_config.yaml) directly.
            lm_max_tokens = settings.llm_max_tokens

            lm_kwargs: dict[str, Any] = {
                "api_key": api_key,
                "cache": False,
                "timeout": settings.llm_timeout_seconds,
                "max_tokens": lm_max_tokens,
            }
            # Keep support for custom gateways and explicit compatibility endpoints.
            if api_base:
                lm_kwargs["api_base"] = api_base

            logger.info(
                "lm_client_initialized",
                provider=("openrouter" if use_openrouter else "openai_compatible"),
                model=litellm_model,
                api_base=api_base,
                planner_token_cap=lm_max_tokens
                if "planner" in agent_role.value
                else None,
            )
            dspy_lm = dspy.LM(litellm_model, **lm_kwargs)

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

    def _create_dspy_adapter(
        self, *, use_native_function_calling: bool = False
    ) -> dspy.ChatAdapter:
        """Use the explicit ChatAdapter contract and fail closed on parse mismatch."""
        return dspy.ChatAdapter(
            use_native_function_calling=use_native_function_calling,
            use_json_adapter_fallback=False,
        )

    @staticmethod
    def _field_desc(field_info: Any) -> str | None:
        extra = getattr(field_info, "json_schema_extra", None) or {}
        desc = extra.get("desc")
        return desc if isinstance(desc, str) and desc.strip() else None

    @staticmethod
    def _field_type(field_info: Any, fallback: type[Any] = str) -> type[Any]:
        annotation = getattr(field_info, "annotation", None)
        return annotation if isinstance(annotation, type) or annotation else fallback

    def _should_use_native_tool_loop(
        self, *, node_type: AgentName, program_cls: type[dspy.Module]
    ) -> bool:
        """Native tool calls are enabled incrementally, starting with benchmark planner."""
        return (
            program_cls is dspy.ReAct
            and node_type == AgentName.BENCHMARK_PLANNER
            and not settings.is_integration_test
        )

    def _build_native_tool_signature(
        self,
        *,
        signature_cls: type[dspy.Signature],
        inputs: dict[str, Any],
        node_type: AgentName,
    ) -> type[dspy.Signature]:
        output_fields = ", ".join(signature_cls.output_fields.keys())
        instructions = getattr(signature_cls, "instructions", "") or ""
        native_instructions = (
            "Use provider-native tool calls.\n"
            "Call the provided tools to inspect and modify the workspace.\n"
            "Always call `submit_plan` before finishing planner work.\n"
            "When all required work is complete, call `finish` with these output "
            f"fields: {output_fields}.\n"
            "Do not emit textual tool-call markers such as `next_tool_name` or "
            "`next_tool_args`."
        )

        field_defs: dict[str, tuple[type[Any], Any]] = {}
        for name, value in inputs.items():
            field_info = signature_cls.input_fields.get(name)
            field_defs[name] = (
                self._field_type(
                    field_info,
                    fallback=(type(value) if value is not None else str),
                ),
                dspy.InputField(desc=self._field_desc(field_info)),
            )

        field_defs["trajectory"] = (
            str,
            dspy.InputField(
                desc="Prior tool calls and observations for the current node."
            ),
        )
        field_defs["tools"] = (
            list[dspy.Tool],
            dspy.InputField(desc="Available tools for native function calling."),
        )
        field_defs["tool_calls"] = (
            dspy.ToolCalls,
            dspy.OutputField(desc="Native tool call(s) to execute this turn."),
        )

        signature_name = (
            f"{node_type.value.title().replace(' ', '').replace('_', '')}"
            "NativeToolSignature"
        )
        return dspy.make_signature(
            field_defs,
            instructions=(
                f"{instructions}\n\n{native_instructions}"
                if instructions
                else native_instructions
            ),
            signature_name=signature_name,
        )

    def _build_finish_tool(
        self, signature_cls: type[dspy.Signature]
    ) -> tuple[dspy.Tool, list[str]]:
        output_field_names = list(signature_cls.output_fields.keys())
        properties: dict[str, Any] = {}
        for name, field_info in signature_cls.output_fields.items():
            field_type = self._field_type(field_info)
            try:
                schema = TypeAdapter(field_type).json_schema()
            except Exception:
                schema = {"type": "string"}
            desc = self._field_desc(field_info)
            if desc:
                schema["description"] = desc
            properties[name] = schema

        def finish(**kwargs: Any) -> dict[str, Any]:
            return kwargs

        finish_tool = dspy.Tool(
            finish,
            name="finish",
            desc=(
                "Call this exactly once when the planner is done. "
                "Provide the final output fields for the node."
            ),
            args=properties,
        )
        return finish_tool, output_field_names

    def _run_native_tool_loop(
        self,
        *,
        signature_cls: type[dspy.Signature],
        inputs: dict[str, Any],
        tool_fns: dict[str, Callable],
        node_type: AgentName,
        max_iters: int,
    ) -> dspy.Prediction:
        tool_signature = self._build_native_tool_signature(
            signature_cls=signature_cls,
            inputs=inputs,
            node_type=node_type,
        )
        predictor = dspy.Predict(tool_signature)
        tool_objects = [dspy.Tool(func) for func in tool_fns.values()]
        finish_tool, finish_fields = self._build_finish_tool(signature_cls)
        tool_objects.append(finish_tool)
        tool_lookup = {tool.name: tool for tool in tool_objects}
        trajectory_parts: list[str] = []

        for iteration in range(max_iters):
            prediction = predictor(
                **inputs,
                trajectory=(
                    "\n\n".join(trajectory_parts)
                    if trajectory_parts
                    else "No tools have been called yet."
                ),
                tools=tool_objects,
            )
            tool_calls = getattr(prediction, "tool_calls", None)
            if not isinstance(tool_calls, dspy.ToolCalls) or not tool_calls.tool_calls:
                msg = (
                    f"Native tool loop for {node_type.value} produced no tool calls "
                    f"on iteration {iteration + 1}."
                )
                raise ValueError(msg)

            for call in tool_calls.tool_calls:
                if call.name == "finish":
                    payload = call.args or {}
                    missing_fields = [
                        field_name
                        for field_name in finish_fields
                        if field_name not in payload
                    ]
                    if missing_fields:
                        msg = (
                            "finish tool call is missing required output fields: "
                            + ", ".join(missing_fields)
                        )
                        raise ValueError(msg)
                    return dspy.Prediction.from_completions(
                        {
                            field_name: [payload.get(field_name)]
                            for field_name in finish_fields
                        },
                        signature=signature_cls,
                    )

                tool = tool_lookup.get(call.name)
                if tool is None:
                    raise ValueError(f"Unknown tool requested: {call.name}")

                observation = call.execute(functions=tool_objects)
                trajectory_parts.append(
                    "\n".join(
                        [
                            f"Tool: {call.name}",
                            f"Args: {json.dumps(call.args or {}, sort_keys=True)}",
                            f"Observation: {self._serialize_tool_observation(observation)}",
                        ]
                    )
                )

        msg = (
            f"Native tool loop for {node_type.value} exhausted {max_iters} iterations."
        )
        raise RuntimeError(msg)

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
            direct_keys = ("thought", "reasoning", "reasoning_content")
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
                    reasoning_keys = ("thought", "reasoning", "reasoning_content")
                    sanitized_kwargs = dict(kwargs)

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
                    for key in reasoning_keys:
                        sanitized_kwargs.pop(key, None)

                    # WP10: Explicitly record tool start
                    trace_id = 0
                    if db_callback:
                        try:
                            # ReAct often passes complex objects, try to
                            # serialize or stringify
                            input_dict = {}
                            if args:
                                input_dict["args"] = [str(a) for a in args]
                            if sanitized_kwargs:
                                input_dict["kwargs"] = {
                                    k: str(v) for k, v in sanitized_kwargs.items()
                                }
                            input_str = json.dumps(input_dict)
                        except Exception:
                            input_str = str(args) + " " + str(kwargs)

                        # WP10: Special handling for common tools to ensure success
                        # in mocks
                        if tool_name == "write_file":
                            if "overwrite" not in sanitized_kwargs:
                                sanitized_kwargs["overwrite"] = True
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

                    tool_timeout_seconds = 60.0
                    if tool_name in {"simulate", "validate"}:
                        # Physics/validation calls can legitimately exceed 60s in integration runs.
                        tool_timeout_seconds = float(
                            settings.dspy_program_timeout_seconds
                        )

                    try:
                        if asyncio.iscoroutinefunction(func):
                            # WP10: Use run_coroutine_threadsafe on the main loop
                            future = asyncio.run_coroutine_threadsafe(
                                func(*args, **sanitized_kwargs), self.ctx.main_loop
                            )
                            result = future.result(timeout=tool_timeout_seconds)
                        else:
                            result = func(*args, **sanitized_kwargs)

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

    @staticmethod
    def _serialize_llm_output(output: Any) -> str | dict[str, Any] | list[Any]:
        """Serialize LM output payloads into structlog-friendly primitives."""
        if isinstance(output, (str, int, float, bool)) or output is None:
            return str(output)
        if isinstance(output, (dict, list)):
            return output
        with suppress(Exception):
            if hasattr(output, "model_dump"):
                return output.model_dump(mode="json")
        return str(output)

    @staticmethod
    def _extract_completion_tokens(usage: Any) -> int | None:
        """Extract output/completion tokens from usage payloads."""
        if usage is None:
            return None
        if hasattr(usage, "model_dump"):
            with suppress(Exception):
                usage = usage.model_dump(mode="json")
        if not isinstance(usage, dict):
            return None

        for key in ("completion_tokens", "output_tokens"):
            value = usage.get(key)
            if isinstance(value, int):
                return value
            if isinstance(value, float):
                return int(value)

        total_tokens = usage.get("total_tokens")
        prompt_tokens = usage.get("prompt_tokens")
        if isinstance(total_tokens, (int, float)) and isinstance(
            prompt_tokens, (int, float)
        ):
            diff = int(total_tokens - prompt_tokens)
            if diff >= 0:
                return diff
        return None

    @staticmethod
    def _extract_prompt_tokens(usage: Any) -> int | None:
        """Extract prompt/input tokens from usage payloads."""
        if usage is None:
            return None
        if hasattr(usage, "model_dump"):
            with suppress(Exception):
                usage = usage.model_dump(mode="json")
        if not isinstance(usage, dict):
            return None

        for key in ("prompt_tokens", "input_tokens"):
            value = usage.get(key)
            if isinstance(value, int):
                return value
            if isinstance(value, float):
                return int(value)
        return None

    @staticmethod
    def _extract_total_tokens(usage: Any) -> int | None:
        """Extract total tokens from usage payloads."""
        if usage is None:
            return None
        if hasattr(usage, "model_dump"):
            with suppress(Exception):
                usage = usage.model_dump(mode="json")
        if not isinstance(usage, dict):
            return None

        total = usage.get("total_tokens")
        if isinstance(total, int):
            return total
        if isinstance(total, float):
            return int(total)

        prompt = BaseNode._extract_prompt_tokens(usage)
        completion = BaseNode._extract_completion_tokens(usage)
        if prompt is not None and completion is not None:
            return prompt + completion
        return None

    @staticmethod
    def _extract_latency_ms(response: Any) -> float | None:
        """Extract per-call latency in milliseconds from provider response."""
        if response is None:
            return None

        response_ms = getattr(response, "response_ms", None)
        if isinstance(response_ms, (int, float)):
            return float(response_ms)

        hidden_params = getattr(response, "_hidden_params", None)
        if isinstance(hidden_params, dict):
            hidden_response_ms = hidden_params.get("response_ms")
            if isinstance(hidden_response_ms, (int, float)):
                return float(hidden_response_ms)
        return None

    @classmethod
    def _compute_tpm(cls, *, usage: Any, response: Any) -> float | None:
        """Compute tokens-per-minute using completion/output tokens and latency."""
        completion_tokens = cls._extract_completion_tokens(usage)
        latency_ms = cls._extract_latency_ms(response)
        if completion_tokens is None or latency_ms is None or latency_ms <= 0:
            return None
        return round((completion_tokens * 60000.0) / latency_ms, 2)

    @staticmethod
    def _extract_reasoning_content(response: Any) -> list[str]:
        """Extract provider-native reasoning content (for example litellm message.reasoning_content)."""
        chunks: list[str] = []
        if response is None:
            return chunks

        response_dict: dict[str, Any] | None = None
        if hasattr(response, "model_dump"):
            with suppress(Exception):
                response_dict = response.model_dump(mode="json")
        elif isinstance(response, dict):
            response_dict = response

        # Typed/object path first (common for litellm responses).
        choices_obj = getattr(response, "choices", None)
        if isinstance(choices_obj, list):
            for choice in choices_obj:
                message = getattr(choice, "message", None)
                if message is None and isinstance(choice, dict):
                    message = choice.get("message")
                value = getattr(message, "reasoning_content", None)
                if value is None and isinstance(message, dict):
                    value = message.get("reasoning_content")
                if isinstance(value, str) and value.strip():
                    chunks.append(value.strip())

        # Dict path fallback.
        if isinstance(response_dict, dict):
            choices = response_dict.get("choices")
            if isinstance(choices, list):
                for choice in choices:
                    if not isinstance(choice, dict):
                        continue
                    message = choice.get("message")
                    if not isinstance(message, dict):
                        continue
                    value = message.get("reasoning_content")
                    if isinstance(value, str) and value.strip():
                        chunks.append(value.strip())

        return chunks

    def _log_lm_history_delta(
        self,
        *,
        node_type: AgentName,
        attempt: int,
        history_start_idx: int | None,
        db_callback: DatabaseCallbackHandler | None = None,
    ) -> None:
        """Emit one log event per newly completed LM generation."""
        if history_start_idx is None:
            return

        history = getattr(self.ctx.dspy_lm, "history", None)
        if not isinstance(history, list):
            return
        if history_start_idx >= len(history):
            return

        for absolute_idx, entry in enumerate(
            history[history_start_idx:], history_start_idx + 1
        ):
            if not isinstance(entry, dict):
                logger.info(
                    "llm_response_generated",
                    node_type=str(node_type),
                    session_id=self.ctx.session_id,
                    attempt=attempt,
                    lm_call_index=absolute_idx,
                    output=str(entry),
                )
                continue

            raw_outputs = entry.get("outputs")
            if isinstance(raw_outputs, list):
                outputs = [self._serialize_llm_output(item) for item in raw_outputs]
            elif raw_outputs is None:
                outputs = []
            else:
                outputs = [self._serialize_llm_output(raw_outputs)]

            logger.info(
                "llm_response_generated",
                node_type=str(node_type),
                session_id=self.ctx.session_id,
                attempt=attempt,
                lm_call_index=absolute_idx,
                model=entry.get("model"),
                response_model=entry.get("response_model"),
                usage=entry.get("usage"),
                cost=entry.get("cost"),
                latency_ms=self._extract_latency_ms(entry.get("response")),
                tpm=self._compute_tpm(
                    usage=entry.get("usage"),
                    response=entry.get("response"),
                ),
                outputs=outputs,
            )

            report_usage_to_current_observation(
                usage=entry.get("usage"),
                model=entry.get("response_model") or entry.get("model"),
                cost=entry.get("cost"),
            )

            prompt_tokens = self._extract_prompt_tokens(entry.get("usage"))
            if prompt_tokens is not None and self.ctx.episode_id:
                with suppress(Exception):
                    loop = asyncio.get_running_loop()
                    loop.create_task(
                        update_episode_context_usage(
                            episode_id=self.ctx.episode_id,
                            used_tokens=prompt_tokens,
                            max_tokens=settings.context_compaction_threshold_tokens,
                        )
                    )
            if self.ctx.episode_id:
                with suppress(Exception):
                    usage = entry.get("usage")
                    loop = asyncio.get_running_loop()
                    loop.create_task(
                        accumulate_episode_credit_usage(
                            episode_id=self.ctx.episode_id,
                            input_tokens=self._extract_prompt_tokens(usage),
                            output_tokens=self._extract_completion_tokens(usage),
                            total_tokens=self._extract_total_tokens(usage),
                        )
                    )

            if not db_callback:
                continue

            # Emit live reasoning traces from LM outputs so UI can show
            # intermediate thinking even before node completion.
            for output in outputs:
                reasoning_chunks: list[str] = []
                if isinstance(output, str):
                    for key in ("Thought", "Reasoning"):
                        matches = re.findall(
                            rf"{key}\s*:\s*(.+?)(?=\n[A-Za-z_]+\s*:|\Z)",
                            output,
                            flags=re.IGNORECASE | re.DOTALL,
                        )
                        reasoning_chunks.extend(
                            [
                                m.strip()
                                for m in matches
                                if isinstance(m, str) and m.strip()
                            ]
                        )
                elif isinstance(output, dict):
                    for key in ("thought", "reasoning", "reasoning_content"):
                        value = output.get(key)
                        if isinstance(value, str) and value.strip():
                            reasoning_chunks.append(value.strip())

                for chunk in reasoning_chunks:
                    text = chunk.strip()
                    if not text:
                        continue
                    if len(text) > 2000:
                        text = text[:2000] + "..."
                    db_callback.record_reasoning_text_sync(
                        node_name=str(node_type),
                        reasoning_text=text,
                        source="lm_history",
                    )

            provider_reasoning = self._extract_reasoning_content(entry.get("response"))
            for chunk in provider_reasoning:
                text = chunk.strip()
                if not text:
                    continue
                if len(text) > 2000:
                    text = text[:2000] + "..."
                db_callback.record_reasoning_text_sync(
                    node_name=str(node_type),
                    reasoning_text=text,
                    source="lm_history",
                )

    def _get_database_recorder(self, session_id: str) -> DatabaseCallbackHandler:
        return self.ctx.get_database_recorder(session_id)

    async def _get_latest_trace_id(self, recorder: DatabaseCallbackHandler) -> int:
        """Return latest trace id for an episode (0 if none)."""
        async with recorder.session_factory() as db:
            result = await db.execute(
                select(func.max(Trace.id)).where(
                    Trace.episode_id == recorder.episode_id
                )
            )
            latest = result.scalar_one_or_none()
            return int(latest or 0)

    async def _count_reasoning_traces_since(
        self,
        recorder: DatabaseCallbackHandler,
        *,
        node_type: AgentName,
        min_trace_id: int,
    ) -> int:
        """Count persisted reasoning traces for a node after a trace-id watermark."""
        async with recorder.session_factory() as db:
            result = await db.execute(
                select(func.count(Trace.id)).where(
                    Trace.episode_id == recorder.episode_id,
                    Trace.trace_type == TraceType.LLM_END,
                    Trace.name == str(node_type),
                    Trace.id > min_trace_id,
                )
            )
            return int(result.scalar_one() or 0)

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
                parsed_json = json.loads(text)
                if isinstance(parsed_json, dict):
                    payload = parsed_json
            with suppress(Exception):
                if payload is None:
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
            # Fail closed on repeated invalid/malformed outputs instead of
            # retrying until the hard timeout window is exhausted.
            max_retries = max(1, int(settings.dspy_program_max_retries))

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
            current_instr = getattr(signature_cls, "instructions", "")
            if current_instr:
                signature_cls = signature_cls.with_instructions(
                    f"{instructions}\n\n{current_instr}"
                )
            else:
                signature_cls = signature_cls.with_instructions(instructions)

        use_native_tool_loop = self._should_use_native_tool_loop(
            node_type=node_type,
            program_cls=program_cls,
        )

        program = None
        if program_cls is dspy.ReAct and not use_native_tool_loop:
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
        elif not use_native_tool_loop:
            program = program_cls(signature_cls, tools=list(tool_fns.values()))

        # WP07: Try to load compiled prompt if available
        if not use_native_tool_loop:
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
                    hard_fail = await evaluate_agent_hard_fail(
                        agent_name=node_type,
                        episode_id=episode_id,
                        turn_count=getattr(state, "turn_count", None),
                    )
                    if hard_fail.should_fail:
                        raise AgentHardFailError(
                            hard_fail.code or "hard_fail",
                            hard_fail.message or "Agent hard-fail limit reached.",
                        )

                    trace_watermark = (
                        await self._get_latest_trace_id(db_callback)
                        if db_callback
                        else 0
                    )
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

                    adapter = self._create_dspy_adapter(
                        use_native_function_calling=use_native_tool_loop
                    )
                    logger.info(
                        "dspy_adapter_selected",
                        session_id=self.ctx.session_id,
                        episode_id=episode_id,
                        node_type=str(node_type),
                        runtime_mode=(
                            "native_tool_loop" if use_native_tool_loop else "react"
                        ),
                        adapter=type(adapter).__name__,
                        use_json_adapter_fallback=getattr(
                            adapter, "use_json_adapter_fallback", None
                        ),
                        use_native_function_calling=getattr(
                            adapter, "use_native_function_calling", None
                        ),
                    )
                    with dspy.settings.context(lm=self.ctx.dspy_lm, adapter=adapter):
                        logger.info(
                            f"{node_type}_dspy_invoke_start",
                            session_id=self.ctx.session_id,
                        )
                        lm_history = getattr(self.ctx.dspy_lm, "history", None)
                        history_start_idx = (
                            len(lm_history) if isinstance(lm_history, list) else None
                        )
                        history_stream_stop = asyncio.Event()
                        history_stream_task: asyncio.Task | None = None

                        final_history_idx = history_start_idx

                        async def stream_lm_history_live(start_idx: int | None) -> None:
                            """Continuously flush newly generated LM history rows."""
                            nonlocal final_history_idx
                            if start_idx is None:
                                return
                            next_idx = start_idx
                            while not history_stream_stop.is_set():
                                self._log_lm_history_delta(
                                    node_type=node_type,
                                    attempt=retry_count + 1,
                                    history_start_idx=next_idx,
                                    db_callback=db_callback,
                                )
                                current_history = getattr(
                                    self.ctx.dspy_lm, "history", None
                                )
                                if isinstance(current_history, list):
                                    next_idx = len(current_history)
                                    final_history_idx = next_idx
                                await asyncio.sleep(0.2)

                        history_stream_task = asyncio.create_task(
                            stream_lm_history_live(history_start_idx)
                        )
                        # Add a timeout to prevent infinite hangs in DSPy ReAct
                        try:
                            if use_native_tool_loop:
                                max_iters = settings.react_planner_max_iters
                                prediction = await asyncio.wait_for(
                                    asyncio.to_thread(
                                        self._run_native_tool_loop,
                                        signature_cls=signature_cls,
                                        inputs=inputs,
                                        tool_fns=tool_fns,
                                        node_type=node_type,
                                        max_iters=max_iters,
                                    ),
                                    timeout=dspy_timeout,
                                )
                            else:
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
                        finally:
                            history_stream_stop.set()
                            if history_stream_task:
                                with suppress(Exception):
                                    await history_stream_task
                            self._log_lm_history_delta(
                                node_type=node_type,
                                attempt=retry_count + 1,
                                history_start_idx=final_history_idx,
                                db_callback=db_callback,
                            )
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

                    if settings.require_reasoning_traces and db_callback:
                        reasoning_count = await self._count_reasoning_traces_since(
                            db_callback,
                            node_type=node_type,
                            min_trace_id=trace_watermark,
                        )
                        if reasoning_count == 0:
                            logger.error(
                                "reasoning_telemetry_missing",
                                node_type=str(node_type),
                                episode_id=str(episode_id),
                                session_id=str(
                                    episode_id
                                ),  # session_id is episode_id here
                                attempt=retry_count + 1,
                            )
                            retry_count += 1
                            if retry_count >= max_retries:
                                raise ValueError(
                                    f"Node {node_type} produced no reasoning traces "
                                    "while REQUIRE_REASONING_TRACES is enabled."
                                )
                            await asyncio.sleep(1)
                            continue

                    return prediction, artifacts, journal_entry

                except AgentHardFailError:
                    raise
                except Exception as err:
                    if _SYSTEM_TOOL_RETRY_EXHAUSTED_MARKER in str(err):
                        # Fail-fast for infra retry exhaustion; do not continue
                        # node-level retry loops.
                        raise RuntimeError(str(err)) from err
                    last_lm_preview = None
                    lm_history = getattr(self.ctx.dspy_lm, "history", None)
                    if isinstance(lm_history, list) and lm_history:
                        last_entry = lm_history[-1]
                        if isinstance(last_entry, dict):
                            last_lm_preview = str(last_entry.get("response"))[:600]
                    logger.error(
                        f"{node_type}_dspy_failed",
                        session_id=self.ctx.session_id,
                        error=str(err),
                        retry_count=retry_count + 1,
                        max_retries=max_retries,
                        last_lm_preview=last_lm_preview,
                    )
                    journal_entry += f"\n[System Error] {err}"
                    retry_count += 1
                    await asyncio.sleep(1)  # Backoff to prevent log explosion

            journal_entry += "\nMax retries reached."
            return None, artifacts, journal_entry

        finally:
            interpreter.shutdown()
