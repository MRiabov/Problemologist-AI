import os

from langfuse import Langfuse
from langfuse.langchain import CallbackHandler


from typing import Any, Dict, List, Optional
from uuid import UUID

from langchain_core.callbacks import BaseCallbackHandler
from langchain_core.messages import BaseMessage
from langchain_core.outputs import LLMResult


import inspect
import asyncio
import logging

logger = logging.getLogger(__name__)


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
                        f"Langfuse async callback error in {method_name} suppressed: {e}"
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

    # Explicitly implement common methods to ensure they are wrapped if __getattr__ isn't enough for base class
    # (BaseCallbackHandler methods are defined, so __getattr__ might not be called for them if we trace via type)
    # However, LangChain checks attributes.
    # To be safe, we should inherit from the specific class or just trust duck typing if we weren't using strict types.
    # But get_langfuse_callback returns CallbackHandler? No, BaseCallbackHandler.

    def on_llm_start(
        self, serialized: Dict[str, Any], prompts: List[str], **kwargs: Any
    ) -> Any:
        try:
            return self._safe_await(
                self.handler.on_llm_start(serialized, prompts, **kwargs), "on_llm_start"
            )
        except Exception as e:
            logger.warning(f"Langfuse callback error in on_llm_start suppressed: {e}")

    def on_chat_model_start(
        self,
        serialized: Dict[str, Any],
        messages: List[List[BaseMessage]],
        **kwargs: Any,
    ) -> Any:
        try:
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
        self, serialized: Dict[str, Any], inputs: Dict[str, Any], **kwargs: Any
    ) -> Any:
        try:
            return self._safe_await(
                self.handler.on_chain_start(serialized, inputs, **kwargs),
                "on_chain_start",
            )
        except Exception as e:
            logger.warning(f"Langfuse callback error in on_chain_start suppressed: {e}")

    def on_chain_end(self, outputs: Dict[str, Any], **kwargs: Any) -> Any:
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
        self, serialized: Dict[str, Any], input_str: str, **kwargs: Any
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
    Compatible with Langfuse v3+ using trace_context.
    """
    public_key = os.getenv("LANGFUSE_PUBLIC_KEY")
    secret_key = os.getenv("LANGFUSE_SECRET_KEY")
    host = os.getenv("LANGFUSE_HOST", "https://cloud.langfuse.com")

    if public_key and secret_key:
        trace_context: dict = kwargs.copy()
        if trace_id:
            trace_context["trace_id"] = trace_id

        # Ensure tags is present as an array to avoid server-side forEach error
        tags = trace_context.get("tags")
        if tags is None or not isinstance(tags, list):
            trace_context["tags"] = []

        # Ensure metadata is a dict if present, or initialize to empty dict
        # This prevents server-side errors if metadata is missing or invalid
        metadata = trace_context.get("metadata")
        if metadata is None or not isinstance(metadata, dict):
            trace_context["metadata"] = {}

        # We MUST use os.environ for secret_key and host as the CallbackHandler
        # constructor doesn't accept them in this version (confirmed via inspect).
        os.environ["LANGFUSE_PUBLIC_KEY"] = public_key
        os.environ["LANGFUSE_SECRET_KEY"] = secret_key
        os.environ["LANGFUSE_HOST"] = host

        # Strictly sanitize trace_context to prevent server-side 500 errors.
        # Langfuse server can crash if these are not exactly what it expects.
        tags = trace_context.get("tags")
        metadata = trace_context.get("metadata")
        clean_context = {
            "trace_id": trace_context.get("trace_id"),
            "name": trace_context.get("name"),
            "user_id": trace_context.get("user_id"),
            "session_id": trace_context.get("session_id"),
            "tags": tags if isinstance(tags, list) else [],
            "metadata": metadata if isinstance(metadata, dict) else {},
        }
        # Remove None values
        clean_context = {k: v for k, v in clean_context.items() if v is not None}

        try:
            handler = CallbackHandler(
                public_key=public_key, trace_context=clean_context
            )
            return SafeCallbackHandler(handler)
        except Exception as e:
            import logging

            logging.getLogger(__name__).error(
                f"Failed to initialize Langfuse CallbackHandler: {e}"
            )
            return None
    return None


def get_langfuse_client() -> Langfuse | None:
    """
    Initialize and return a Langfuse client if credentials are provided.
    """
    public_key = os.getenv("LANGFUSE_PUBLIC_KEY")
    secret_key = os.getenv("LANGFUSE_SECRET_KEY")
    host = os.getenv("LANGFUSE_HOST", "http://localhost:13000")

    if public_key and secret_key:
        return Langfuse(public_key=public_key, secret_key=secret_key, host=host)
    return None
