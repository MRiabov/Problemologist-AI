import os
from contextlib import AbstractContextManager, nullcontext
from inspect import Parameter, signature
from typing import Any

from langfuse import Langfuse
from openinference.instrumentation.dspy import DSPyInstrumentor
from pydantic import BaseModel

from controller.config.settings import settings
from shared.enums import AgentName, TraceType
from shared.logging import get_logger

logger = get_logger(__name__)


class LangfuseUsageDetails(BaseModel):
    """Normalized token/cost usage payload forwarded from DSPy/LiteLLM."""

    input_tokens: int | None = None
    output_tokens: int | None = None
    total_tokens: int | None = None
    total_cost: float | None = None
    model: str | None = None


def init_tracing() -> None:
    """
    Initialize native DSPy tracing for Langfuse using OpenInference.
    """
    if settings.is_integration_test:
        return

    public_key = settings.langfuse_public_key
    secret_key = settings.langfuse_secret_key
    host = settings.langfuse_host

    if public_key and secret_key:
        try:
            # OpenInference DSPy instrumentor picks up Langfuse env vars
            os.environ["LANGFUSE_PUBLIC_KEY"] = public_key
            os.environ["LANGFUSE_SECRET_KEY"] = secret_key
            os.environ["LANGFUSE_HOST"] = host

            DSPyInstrumentor().instrument()
            logger.info("Native DSPy tracing (Langfuse) initialized.")
        except Exception as e:
            logger.error(f"Failed to initialize native DSPy instrumentation: {e}")


def get_langfuse_client() -> Langfuse | None:
    """
    Initialize and return a Langfuse client if credentials are provided.
    """
    if settings.is_integration_test:
        return None

    public_key = settings.langfuse_public_key
    secret_key = settings.langfuse_secret_key
    host = settings.langfuse_host

    if public_key and secret_key:
        return Langfuse(public_key=public_key, secret_key=secret_key, host=host)
    return None


def report_score(
    trace_id: str, name: str, value: float, comment: str | None = None
) -> None:
    """
    Report a score to Langfuse trace.
    """
    try:
        client = get_langfuse_client()
        if client:
            client.score(trace_id=trace_id, name=name, value=value, comment=comment)
    except Exception as e:
        logger.warning(f"Failed to report score to Langfuse: {e}")


def start_root_span(
    *,
    name: str,
    trace_id: str | None = None,
    input_payload: Any | None = None,
    metadata: dict[str, Any] | None = None,
) -> AbstractContextManager[Any]:
    """Start a Langfuse root span for the current workflow when available."""
    client = get_langfuse_client()
    if not client:
        return nullcontext(None)

    trace_context = {"trace_id": trace_id} if trace_id else None
    try:
        return client.start_as_current_span(
            name=name,
            trace_context=trace_context,
            input=input_payload,
            metadata=metadata,
        )
    except Exception as e:
        logger.warning("failed_to_start_langfuse_root_span", error=str(e), name=name)
        return nullcontext(None)


def attach_session_to_current_trace(
    session_id: str | None,
    *,
    trace_name: str | None = None,
    metadata: dict[str, Any] | None = None,
) -> None:
    """Attach a session ID to the active Langfuse trace."""
    if not session_id:
        return

    client = get_langfuse_client()
    if not client:
        return

    try:
        client.update_current_trace(
            session_id=session_id,
            name=trace_name,
            metadata=metadata,
        )
    except Exception as e:
        logger.warning(
            "failed_to_attach_langfuse_session",
            error=str(e),
            session_id=session_id,
        )


def _to_int(value: Any) -> int | None:
    if isinstance(value, bool):
        return None
    if isinstance(value, int):
        return value
    if isinstance(value, float):
        return int(value)
    return None


def _to_float(value: Any) -> float | None:
    if isinstance(value, bool):
        return None
    if isinstance(value, (int, float)):
        return float(value)
    return None


def _build_usage_details(
    usage: Any,
    *,
    model: str | None,
    total_cost: float | None,
) -> LangfuseUsageDetails | None:
    usage_payload = usage
    if usage_payload is None:
        return None

    if hasattr(usage_payload, "model_dump"):
        try:
            usage_payload = usage_payload.model_dump(mode="json")
        except Exception:
            return None

    if not isinstance(usage_payload, dict):
        return None

    input_tokens = _to_int(
        usage_payload.get("prompt_tokens", usage_payload.get("input_tokens"))
    )
    output_tokens = _to_int(
        usage_payload.get("completion_tokens", usage_payload.get("output_tokens"))
    )
    total_tokens = _to_int(usage_payload.get("total_tokens"))

    if total_tokens is None and input_tokens is not None and output_tokens is not None:
        total_tokens = input_tokens + output_tokens

    payload = LangfuseUsageDetails(
        input_tokens=input_tokens,
        output_tokens=output_tokens,
        total_tokens=total_tokens,
        total_cost=total_cost,
        model=model,
    )

    if (
        payload.input_tokens is None
        and payload.output_tokens is None
        and payload.total_tokens is None
        and payload.total_cost is None
    ):
        return None

    return payload


def report_usage_to_current_observation(
    *,
    usage: Any,
    model: str | None = None,
    cost: Any | None = None,
) -> None:
    """Best-effort usage forwarding to the active Langfuse observation.

    Usage data is sourced from DSPy LM history entries, which are backed by LiteLLM.
    """
    client = get_langfuse_client()
    if not client:
        return

    usage_details = _build_usage_details(
        usage,
        model=model,
        total_cost=_to_float(cost),
    )
    if usage_details is None:
        return

    method = getattr(client, "update_current_observation", None)
    if method is None:
        return

    payload = usage_details.model_dump(exclude_none=True)
    token_payload = {
        k: v
        for k, v in {
            "input": usage_details.input_tokens,
            "output": usage_details.output_tokens,
            "total": usage_details.total_tokens,
        }.items()
        if v is not None
    }

    try:
        params = signature(method).parameters
        has_var_kwargs = any(p.kind == Parameter.VAR_KEYWORD for p in params.values())

        kwargs: dict[str, Any] = {}

        if "usage_details" in params or has_var_kwargs:
            kwargs["usage_details"] = token_payload
        elif "usage" in params:
            kwargs["usage"] = token_payload
        else:
            for key, value in (
                ("input_tokens", usage_details.input_tokens),
                ("output_tokens", usage_details.output_tokens),
                ("total_tokens", usage_details.total_tokens),
            ):
                if value is not None and key in params:
                    kwargs[key] = value

        if usage_details.model is not None and ("model" in params or has_var_kwargs):
            kwargs["model"] = usage_details.model

        if usage_details.total_cost is not None:
            if "total_cost" in params or has_var_kwargs:
                kwargs["total_cost"] = usage_details.total_cost
            elif "cost" in params:
                kwargs["cost"] = usage_details.total_cost

        method(**kwargs)
    except Exception as e:
        logger.warning(
            "failed_to_report_langfuse_usage",
            error=str(e),
            usage=payload,
        )


async def calculate_and_report_automated_score(
    episode_id: Any,  # uuid.UUID
    trace_id: str,
    agent_name: AgentName,
    db: Any,  # AsyncSession
    worker_client: Any,  # WorkerClient
) -> None:
    """Calculates the automated score for an episode and reports it to Langfuse."""
    try:
        import yaml
        from dspy import Prediction
        from sqlalchemy import select

        from controller.agent.dspy_utils import (
            cad_simulation_metric,
            map_events_to_prediction,
        )
        from controller.persistence.models import Episode, Trace

        # 1. Get all events recorded for this episode
        stmt = select(Trace).where(
            Trace.episode_id == episode_id,
            Trace.trace_type == TraceType.EVENT,
        )
        res = await db.execute(stmt)
        event_traces = res.scalars().all()
        events = [t.metadata_vars for t in event_traces if t.metadata_vars]

        # 2. Get objectives for context if possible
        objectives_data = None
        try:
            from shared.models.schemas import ObjectivesYaml

            content = await worker_client.read_file("objectives.yaml")
            raw_data = yaml.safe_load(content)
            obj_model = ObjectivesYaml(**raw_data)
            objectives_data = obj_model.model_dump()
        except Exception:
            pass

        # 3. Calculate score
        pred_dict = map_events_to_prediction(events)

        # Get episode to check metadata for fallback caps
        stmt = select(Episode).where(Episode.id == episode_id)
        res = await db.execute(stmt)
        episode = res.scalar_one_or_none()

        max_unit_cost = 1000.0
        max_weight = 10.0

        if episode and episode.metadata_vars:
            from shared.models.schemas import EpisodeMetadata

            metadata = EpisodeMetadata.model_validate(episode.metadata_vars)
            if metadata.custom_objectives:
                if metadata.custom_objectives.max_unit_cost is not None:
                    max_unit_cost = metadata.custom_objectives.max_unit_cost
                if metadata.custom_objectives.max_weight is not None:
                    max_weight = metadata.custom_objectives.max_weight

        gold = Prediction(
            agent_name=agent_name,
            objectives=objectives_data,
            # Provide basic data if objectives.yaml is missing
            max_unit_cost=max_unit_cost,
            max_weight=max_weight,
        )
        prediction = Prediction(**pred_dict)

        metric_result = cad_simulation_metric(gold, prediction)

        # 4. Report to Langfuse
        report_score(
            trace_id=trace_id,
            name="automated_score",
            value=metric_result.score,
            comment=metric_result.feedback,
        )
        logger.info(
            "automated_score_reported",
            episode_id=str(episode_id),
            score=metric_result.score,
        )
    except Exception as e:
        logger.warning(
            "failed_to_calculate_automated_score",
            episode_id=str(episode_id),
            error=str(e),
        )
