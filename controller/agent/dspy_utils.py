import asyncio
import os
from typing import Any

import dspy

from controller.config.settings import settings
from shared.logging import get_logger

logger = get_logger(__name__)

def init_dspy(session_id: str | None = None):
    """
    Initializes DSPy with the configured language model and optional Langfuse tracing.
    """
    try:
        # Use OpenAI-compatible interface for DSPy
        lm = dspy.LM(
            model=settings.llm_model,
            api_key=settings.openai_api_key,
            api_base=settings.openai_api_base,
            cache=False,
        )

        config_kwargs = {"lm": lm}

        # Integrate Langfuse if configured
        if settings.langfuse_public_key and settings.langfuse_secret_key:
            try:
                from langfuse.dspy import LangfuseDSPy
                # Ensure credentials are in env for the integration
                os.environ["LANGFUSE_PUBLIC_KEY"] = settings.langfuse_public_key
                os.environ["LANGFUSE_SECRET_KEY"] = settings.langfuse_secret_key
                os.environ["LANGFUSE_HOST"] = settings.langfuse_host

                langfuse_dspy = LangfuseDSPy(session_id=session_id)
                config_kwargs["trace"] = [langfuse_dspy]
                logger.info("dspy_langfuse_tracing_enabled", session_id=session_id)
            except Exception as e:
                logger.warning("dspy_langfuse_integration_failed", error=str(e))

        dspy.settings.configure(**config_kwargs)
        logger.info("dspy_initialized", model=settings.llm_model)
        return lm
    except Exception as e:
        logger.error("dspy_initialization_failed", error=str(e))
        raise

def wrap_tool_for_dspy(lc_tool: Any) -> Any:
    """
    Wraps a LangChain async tool for DSPy CodeAct/ReAct which expects synchronous calls.
    """
    def sync_tool_wrapper(**kwargs):
        try:
            loop = asyncio.get_event_loop()
        except RuntimeError:
            loop = asyncio.new_event_loop()
            asyncio.set_event_loop(loop)

        if loop.is_running():
            from concurrent.futures import ThreadPoolExecutor
            with ThreadPoolExecutor() as executor:
                # Run the async tool in a new event loop in the thread
                future = executor.submit(lambda: asyncio.run(lc_tool.ainvoke(kwargs)))
                return future.result()
        else:
            return loop.run_until_complete(lc_tool.ainvoke(kwargs))

    # DSPy CodeAct expects functions with proper metadata
    sync_tool_wrapper.__name__ = lc_tool.name
    sync_tool_wrapper.__doc__ = lc_tool.description
    return sync_tool_wrapper
