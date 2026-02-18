import asyncio
import logging
from collections.abc import Callable
from dataclasses import dataclass
from pathlib import Path
from typing import TYPE_CHECKING, Any

import dspy
from langchain_core.messages import BaseMessage
from langchain_openai import ChatOpenAI

from controller.agent.config import settings
from controller.agent.prompt_manager import PromptManager
from controller.clients.worker import WorkerClient
from controller.middleware.remote_fs import RemoteFilesystemMiddleware
from controller.observability.database import DatabaseCallbackHandler
from controller.observability.langfuse import get_langfuse_callback

if TYPE_CHECKING:
    from controller.agent.state import AgentState

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
                    f"  - Level: {sel['level']}, ID: {sel['target_id']}, "
                    f"Center: ({center_str})"
                )

        if steer_data.get("code_references"):
            lines.append("Code References:")
            for ref in steer_data["code_references"]:
                lines.append(
                    f"  - File: {ref['file_path']}, "
                    f"Lines: {ref['start_line']}-{ref['end_line']}"
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
        node_type: str,
        max_retries: int = 3,
    ) -> tuple[Any, dict[str, Any], str]:
        """
        Reusable execution loop for DSPy nodes with retries and validation.
        Returns (prediction, artifacts, journal_entry).
        """
        from controller.agent.dspy_utils import WorkerInterpreter
        from worker.utils.file_validation import validate_node_output

        interpreter = WorkerInterpreter(
            worker_client=self.ctx.worker_client, session_id=state.session_id
        )
        tool_fns = self._get_tool_functions(tool_factory)

        # WP07: Inject system instructions from prompts.yaml into signature
        # Map node_type to prompt template names
        node_to_template = {
            "planner": "architect",
            "coder": "engineer",
            "electronics_engineer": "electronics_engineer",
            "reviewer": "critic",
            "cots_search": "cots_search",
        }
        template_name = node_to_template.get(node_type)
        if template_name:
            instructions = self.ctx.pm.render(template_name)
            signature_cls = signature_cls.with_instructions(instructions)

        program = program_cls(
            signature_cls, tools=list(tool_fns.values()), interpreter=interpreter
        )

        # WP07: Try to load compiled prompt if available
        self.ctx.pm.load_compiled_program(node_type, program)

        retry_count = 0
        journal_entry = ""
        prediction = None
        artifacts = {}

        try:
            while retry_count < max_retries:
                try:
                    with dspy.settings.context(lm=self.ctx.dspy_lm):
                        logger.info(
                            f"{node_type}_dspy_invoke_start",
                            session_id=state.session_id,
                        )
                        prediction = program(**inputs)
                        logger.info(
                            f"{node_type}_dspy_invoke_complete",
                            session_id=state.session_id,
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
                        continue

                    return prediction, artifacts, journal_entry

                except Exception as e:
                    logger.error(f"{node_type}_dspy_failed", error=str(e))
                    journal_entry += f"\n[System Error] {e}"
                    retry_count += 1

            journal_entry += "\nMax retries reached."
            return None, artifacts, journal_entry

        finally:
            interpreter.shutdown()
