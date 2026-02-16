from dataclasses import dataclass
from pathlib import Path
from typing import Any, cast
import logging

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
    """Shared dependencies for agent nodes."""

    worker_url: str
    session_id: str
    pm: PromptManager
    llm: ChatOpenAI
    worker_client: WorkerClient
    fs: RemoteFilesystemMiddleware

    @classmethod
    def create(cls, worker_url: str, session_id: str) -> "SharedNodeContext":
        worker_client = WorkerClient(base_url=worker_url, session_id=session_id)
        return cls(
            worker_url=worker_url,
            session_id=session_id,
            pm=PromptManager(),
            llm=ChatOpenAI(model=settings.llm_model, temperature=0),
            worker_client=worker_client,
            fs=RemoteFilesystemMiddleware(worker_client),
        )


class BaseNode:
    """Base class for agent nodes with common utilities."""

    def __init__(self, context: SharedNodeContext):
        self.ctx = context

    def _get_callbacks(self, name: str, session_id: str):
        langfuse_callback = get_langfuse_callback(name=name, session_id=session_id)
        db_callback = DatabaseCallbackHandler(
            episode_id=session_id, langfuse_callback=langfuse_callback
        )
        callbacks = [db_callback]
        if langfuse_callback:
            callbacks.append(langfuse_callback)
        return callbacks

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
