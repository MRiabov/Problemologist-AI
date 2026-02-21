import os
from typing import Any

from langfuse import Langfuse
from openinference.instrumentation.dspy import DSPyInstrumentor

from controller.config.settings import settings
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


async def calculate_and_report_automated_score(
    episode_id: Any,  # uuid.UUID
    trace_id: str,
    agent_name: str,
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
        from shared.enums import TraceType

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
            content = await worker_client.read_file("objectives.yaml")
            objectives_data = yaml.safe_load(content)
        except Exception:
            pass

        # 3. Calculate score
        pred_dict = map_events_to_prediction(events)

        # Get episode to check metadata for fallback caps
        stmt = select(Episode).where(Episode.id == episode_id)
        res = await db.execute(stmt)
        episode = res.scalar_one_or_none()

        gold = Prediction(
            agent_name=agent_name,
            objectives=objectives_data,
            # Provide basic data if objectives.yaml is missing
            max_unit_cost=episode.metadata_vars.get("max_unit_cost")
            if episode and episode.metadata_vars
            else 1000.0,
            max_weight=episode.metadata_vars.get("max_weight")
            if episode and episode.metadata_vars
            else 10.0,
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
