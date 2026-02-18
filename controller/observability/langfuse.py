import inspect
import os
from typing import Any

from langchain_core.callbacks import BaseCallbackHandler
from langchain_core.messages import BaseMessage
from langchain_core.outputs import LLMResult
from langfuse import Langfuse
from langfuse.langchain import CallbackHandler

from controller.config.settings import settings
from shared.logging import get_logger

logger = get_logger(__name__)


class SafeCallbackHandler(BaseCallbackHandler):
    """
    Wrapper around a CallbackHandler to suppress all errors.
    """

    def __init__(self, handler: BaseCallbackHandler):
        self.handler = handler

    def _safe_await(self, result: Any, method_name: str) -> Any:
        if inspect.isawaitable(result):

            async def safe_await_wrapper():
                try:
                    return await result
                except Exception as e:
                    logger.warning(
                        f"Langfuse async callback error in {method_name} "
                        f"suppressed: {e}"
                    )

            return safe_await_wrapper()
        return result

    def __getattr__(self, name: str) -> Any:
        # Wrap any method of the underlying handler
        attr = getattr(self.handler, name)
        if callable(attr):

            def safe_call(*args, **kwargs):
                try:
                    result = attr(*args, **kwargs)
                    return self._safe_await(result, name)
                except Exception as e:
                    logger.warning(f"Langfuse callback error in {name} suppressed: {e}")
                    return None

            return safe_call
        return attr

    # Explicitly implement common methods to ensure they are wrapped if __getattr__
    # isn't enough for base class (BaseCallbackHandler methods are defined, so
    # __getattr__ might not be called for them if we trace via type).
    # However, LangChain checks attributes.
    # To be safe, we should inherit from the specific class or just trust duck typing
    # if we weren't using strict types.
    # But get_langfuse_callback returns CallbackHandler? No, BaseCallbackHandler.

    def on_llm_start(
        self, serialized: dict[str, Any], prompts: list[str], **kwargs: Any
    ) -> Any:
        try:
            # Inject model_name if possible to help Langfuse parse it
            if "metadata" not in kwargs:
                kwargs["metadata"] = {}
            if "model_name" not in kwargs["metadata"]:
                model_name = None
                if "invocation_params" in kwargs:
                    model_name = kwargs["invocation_params"].get(
                        "model_name"
                    ) or kwargs["invocation_params"].get("model")
                # Fallback to global settings if not found in invocation
                kwargs["metadata"]["model_name"] = model_name or settings.llm_model

            return self._safe_await(
                self.handler.on_llm_start(serialized, prompts, **kwargs), "on_llm_start"
            )
        except Exception as e:
            logger.warning(f"Langfuse callback error in on_llm_start suppressed: {e}")

    def on_chat_model_start(
        self,
        serialized: dict[str, Any],
        messages: list[list[BaseMessage]],
        **kwargs: Any,
    ) -> Any:
        try:
            # Inject model_name if possible to help Langfuse parse it
            if "metadata" not in kwargs:
                kwargs["metadata"] = {}
            if "model_name" not in kwargs["metadata"]:
                model_name = None
                if "invocation_params" in kwargs:
                    model_name = kwargs["invocation_params"].get(
                        "model_name"
                    ) or kwargs["invocation_params"].get("model")
                # Fallback to global settings if not found in invocation
                kwargs["metadata"]["model_name"] = model_name or settings.llm_model

            return self._safe_await(
                self.handler.on_chat_model_start(serialized, messages, **kwargs),
                "on_chat_model_start",
            )
        except Exception as e:
            logger.warning(
                f"Langfuse callback error in on_chat_model_start suppressed: {e}"
            )

    def on_llm_new_token(self, token: str, **kwargs: Any) -> Any:
        try:
            return self._safe_await(
                self.handler.on_llm_new_token(token, **kwargs), "on_llm_new_token"
            )
        except Exception as e:
            logger.warning(
                f"Langfuse callback error in on_llm_new_token suppressed: {e}"
            )

    def on_llm_end(self, response: LLMResult, **kwargs: Any) -> Any:
        try:
            return self._safe_await(
                self.handler.on_llm_end(response, **kwargs), "on_llm_end"
            )
        except Exception as e:
            logger.warning(f"Langfuse callback error in on_llm_end suppressed: {e}")

    def on_llm_error(self, error: BaseException, **kwargs: Any) -> Any:
        try:
            return self._safe_await(
                self.handler.on_llm_error(error, **kwargs), "on_llm_error"
            )
        except Exception as e:
            logger.warning(f"Langfuse callback error in on_llm_error suppressed: {e}")

    def on_chain_start(
        self, serialized: dict[str, Any], inputs: dict[str, Any], **kwargs: Any
    ) -> Any:
        try:
            return self._safe_await(
                self.handler.on_chain_start(serialized, inputs, **kwargs),
                "on_chain_start",
            )
        except Exception as e:
            logger.warning(f"Langfuse callback error in on_chain_start suppressed: {e}")

    def on_chain_end(self, outputs: dict[str, Any], **kwargs: Any) -> Any:
        try:
            return self._safe_await(
                self.handler.on_chain_end(outputs, **kwargs), "on_chain_end"
            )
        except Exception as e:
            logger.warning(f"Langfuse callback error in on_chain_end suppressed: {e}")

    def on_chain_error(self, error: BaseException, **kwargs: Any) -> Any:
        try:
            return self._safe_await(
                self.handler.on_chain_error(error, **kwargs), "on_chain_error"
            )
        except Exception as e:
            logger.warning(f"Langfuse callback error in on_chain_error suppressed: {e}")

    def on_tool_start(
        self, serialized: dict[str, Any], input_str: str, **kwargs: Any
    ) -> Any:
        try:
            return self._safe_await(
                self.handler.on_tool_start(serialized, input_str, **kwargs),
                "on_tool_start",
            )
        except Exception as e:
            logger.warning(f"Langfuse callback error in on_tool_start suppressed: {e}")

    def on_tool_end(self, output: str, **kwargs: Any) -> Any:
        try:
            return self._safe_await(
                self.handler.on_tool_end(output, **kwargs), "on_tool_end"
            )
        except Exception as e:
            logger.warning(f"Langfuse callback error in on_tool_end suppressed: {e}")

    def on_tool_error(self, error: BaseException, **kwargs: Any) -> Any:
        try:
            return self._safe_await(
                self.handler.on_tool_error(error, **kwargs), "on_tool_error"
            )
        except Exception as e:
            logger.warning(f"Langfuse callback error in on_tool_error suppressed: {e}")


def get_langfuse_callback(
    trace_id: str | None = None, **kwargs
) -> BaseCallbackHandler | None:
    """
    Initialize and return a Langfuse CallbackHandler if credentials are provided.
    """
    if settings.is_integration_test:
        return None

    public_key = settings.langfuse_public_key
    secret_key = settings.langfuse_secret_key
    host = settings.langfuse_host

    if public_key and secret_key:
        trace_context = kwargs.copy()
        if trace_id:
            trace_context["trace_id"] = trace_id

        # Sanitize trace_context
        clean_context = {
            "trace_id": trace_context.get("trace_id"),
            "name": trace_context.get("name"),
            "user_id": trace_context.get("user_id"),
            "session_id": trace_context.get("session_id"),
            "tags": trace_context.get("tags")
            if isinstance(trace_context.get("tags"), list)
            else [],
            "metadata": trace_context.get("metadata")
            if isinstance(trace_context.get("metadata"), dict)
            else {},
        }
        clean_context = {k: v for k, v in clean_context.items() if v is not None}

        # We MUST use os.environ for secret_key and host as some versions of
        # CallbackHandler don't accept them in the constructor.
        if secret_key:
            os.environ["LANGFUSE_SECRET_KEY"] = secret_key
        if host:
            os.environ["LANGFUSE_HOST"] = host
        if public_key:
            os.environ["LANGFUSE_PUBLIC_KEY"] = public_key

        try:
            # We pass credentials directly to avoid global state issues
            handler = CallbackHandler(
                public_key=public_key, trace_context=clean_context
            )
            return SafeCallbackHandler(handler)
        except Exception as e:
            logger.error(f"Failed to initialize Langfuse CallbackHandler: {e}")
            return None
    return None


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
    session_id: str,
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
