import os
from contextlib import AbstractContextManager, nullcontext
from typing import Any

from langfuse import Langfuse
from openinference.instrumentation.dspy import DSPyInstrumentor

from controller.config.settings import settings
from shared.enums import AgentName, TraceType
from shared.logging import get_logger

logger = get_logger(__name__)


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
