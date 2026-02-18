import logging
from collections.abc import Callable
from contextlib import asynccontextmanager
from dataclasses import dataclass
from pathlib import Path

import dspy
from langchain_core.messages import BaseMessage
from langchain_openai import ChatOpenAI

from controller.agent.config import settings
from controller.agent.prompt_manager import PromptManager
from controller.clients.worker import WorkerClient
from controller.middleware.remote_fs import RemoteFilesystemMiddleware
from controller.observability.database import DatabaseCallbackHandler
from controller.observability.langfuse import get_langfuse_callback

logger = logging.getLogger(__name__)


@dataclass
class SharedNodeContext:
    """Consolidates shared dependencies for agent nodes."""

    worker_url: str
    session_id: str
    pm: PromptManager
    llm: ChatOpenAI
    dspy_lm: dspy.LM
    worker_client: WorkerClient
    fs: RemoteFilesystemMiddleware

    @classmethod
    def create(cls, worker_url: str, session_id: str) -> "SharedNodeContext":
        worker_client = WorkerClient(base_url=worker_url, session_id=session_id)
        llm = ChatOpenAI(model=settings.llm_model, temperature=0)

        # T012: Initialize DSPy LM for CodeAct support
        api_key = "dummy"
        if settings.openai_api_key:
            api_key = settings.openai_api_key

        dspy_lm = dspy.LM(
            f"openai/{settings.llm_model}",
            api_key=api_key,
            cache=False,
        )
        return cls(
            worker_url=worker_url,
            session_id=session_id,
            pm=PromptManager(),
            llm=llm,
            dspy_lm=dspy_lm,
            worker_client=worker_client,
            fs=RemoteFilesystemMiddleware(worker_client),
        )

    def get_callbacks(self, name: str, session_id: str | None = None):
        """Creates observability callbacks."""
        sid = session_id or self.session_id
        langfuse_callback = get_langfuse_callback(name=name, session_id=sid)
        db_callback = DatabaseCallbackHandler(
            episode_id=sid, langfuse_callback=langfuse_callback
        )
        callbacks = [db_callback]
        if langfuse_callback:
            callbacks.append(langfuse_callback)
        return callbacks

    @asynccontextmanager
    async def lifecycle(self):
        """Context manager for node lifecycle and resource cleanup."""
        try:
            yield self
        finally:
            await self.worker_client.aclose()


class BaseNode:
    """Base class for agent nodes providing common utilities."""

    def __init__(self, context: SharedNodeContext):
        self.ctx = context

    def _get_tool_functions(self, tool_factory: Callable) -> dict[str, Callable]:
        """Extracts raw functions from LangChain tools for DSPy compatibility."""
        tools = tool_factory(self.ctx.fs, self.ctx.session_id)
        tool_fns = {}
        for t in tools:
            # LangChain tools often wrap the actual function
            func = getattr(t, "func", getattr(t, "_run", None))
            if func:
                # WP06: Ensure the function signature is compatible with DSPy
                # We might need to wrap it to remove 'callbacks' etc. if they exist
                tool_fns[t.name] = func
        return tool_fns

    def _get_callbacks(self, name: str, session_id: str):
        return self.ctx.get_callbacks(name, session_id)

    def _get_skills_context(self) -> str:
        # Note: Skills are currently local to the controller
        skills_dir = Path(".agent/skills")
        skills = []
        if skills_dir.exists():
            skills = [d.name for d in skills_dir.iterdir() if d.is_dir()]
        return "\n".join([f"- {s}" for s in skills])

    def _get_steer_context(self, messages: list[BaseMessage]) -> str:
        if not messages:
            return ""
        last_msg = messages[-1]
        steer_data = (
            last_msg.additional_kwargs.get("steerability")
            if hasattr(last_msg, "additional_kwargs")
            else None
        )
        if not steer_data:
            return ""

        lines = []
        if steer_data.get("selections"):
            lines.append("Geometric Selections:")
            for sel in steer_data["selections"]:
                center_str = ", ".join([f"{v:.2f}" for v in sel["center"]])
                lines.append(
                    f"  - Level: {sel['level']}, ID: {sel['target_id']}, Center: ({center_str})"
                )

        if steer_data.get("code_references"):
            lines.append("Code References:")
            for ref in steer_data["code_references"]:
                lines.append(
                    f"  - File: {ref['file_path']}, Lines: {ref['start_line']}-{ref['end_line']}"
                )

        if steer_data.get("mentions"):
            lines.append(f"Mentions: {', '.join(steer_data['mentions'])}")

        return "\n".join(lines)
