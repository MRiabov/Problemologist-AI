import asyncio
import re
from collections.abc import Callable
from dataclasses import dataclass
from pathlib import Path
from typing import TYPE_CHECKING, Any

import dspy
import structlog

from controller.agent.config import settings
from controller.agent.prompt_manager import PromptManager
from controller.clients.worker import WorkerClient
from controller.middleware.remote_fs import RemoteFilesystemMiddleware
from controller.observability.database import DatabaseCallbackHandler
from controller.observability.langfuse import init_tracing

if TYPE_CHECKING:
    pass

logger = structlog.get_logger(__name__)


@dataclass
class SharedNodeContext:
    """Consolidates shared dependencies for agent nodes."""

    worker_light_url: str
    session_id: str
    pm: PromptManager
    dspy_lm: dspy.LM
    worker_client: WorkerClient
    fs: RemoteFilesystemMiddleware

    @classmethod
    def create(cls, worker_light_url: str, session_id: str) -> "SharedNodeContext":
        worker_client = WorkerClient(
            base_url=worker_light_url,
            session_id=session_id,
            heavy_url=settings.worker_heavy_url,
        )

        # WP05: Support mock LLMs for integration tests
        if settings.is_integration_test:
            from controller.agent.mock_llm import MockDSPyLM

            logger.info("using_mock_llms_for_integration_test", session_id=session_id)
            dspy_lm = MockDSPyLM(session_id=session_id)
        else:
            # T025: Initialize native tracing
            init_tracing()

            # T012: Initialize DSPy LM for CodeAct support
            api_key = "dummy"
            if settings.openai_api_key:
                api_key = settings.openai_api_key

            dspy_lm = dspy.LM(
                f"openai/{settings.llm_model}",
                api_key=api_key,
                cache=False,
                timeout=600,
            )

        return cls(
            worker_light_url=worker_light_url,
            session_id=session_id,
            pm=PromptManager(),
            dspy_lm=dspy_lm,
            worker_client=worker_client,
            fs=RemoteFilesystemMiddleware(worker_client),
        )

    def get_database_recorder(
        self, session_id: str | None = None
    ) -> DatabaseCallbackHandler:
        """Creates a database recorder for traces."""
        sid = session_id or self.session_id
        return DatabaseCallbackHandler(episode_id=sid)


class BaseNode:
    """Base class for agent nodes providing common utilities."""

    def __init__(self, context: SharedNodeContext):
        self.ctx = context

    def _get_tool_functions(self, tool_factory: Callable) -> dict[str, Callable]:
        """Collects tool functions for DSPy compatibility."""
        tools = tool_factory(self.ctx.fs, self.ctx.session_id)
        tool_fns = {}
        for t in tools:
            # Now tools are already raw functions or wrapped in search_cots_catalog
            name = getattr(t, "name", t.__name__ if hasattr(t, "__name__") else str(t))
            tool_fns[name] = t
        return tool_fns

    def _get_database_recorder(self, session_id: str) -> DatabaseCallbackHandler:
        return self.ctx.get_database_recorder(session_id)

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

        # T015: Parse @file:line-line from message content if not already in code_references
        content = last_msg.content if isinstance(last_msg.content, str) else ""
        # Matches @path/to/file.ext:start-end
        matches = re.finditer(r"@([\w\./\-]+\.\w+):(\d+)-(\d+)", content)
        parsed_refs = []
        for match in matches:
            parsed_refs.append(
                {
                    "file_path": match.group(1),
                    "start_line": int(match.group(2)),
                    "end_line": int(match.group(3)),
                }
            )

        if not steer_data and not parsed_refs:
            return ""

        steer_data = steer_data or {}
        code_refs = steer_data.get("code_references", []) + parsed_refs

        if steer_data.get("selections"):
            lines.append("Geometric Selections:")
            for sel in steer_data["selections"]:
                center_str = ", ".join([f"{v:.2f}" for v in sel["center"]])
                lines.append(
                    f"  - Level: {sel['level']}, ID: {sel['target_id']}, "
                    f"Center: ({center_str})"
                )

        if code_refs:
            lines.append("Code References:")
            for ref in code_refs:
                file_path = ref["file_path"]
                start = ref["start_line"]
                end = ref["end_line"]
                try:
                    # Resolve snippet from worker
                    file_content = await self.ctx.fs.read_file(file_path)
                    file_lines = file_content.splitlines()
                    if start < 1 or end > len(file_lines) or start > end:
                        lines.append(
                            f"  - File: {file_path}, Lines: {start}-{end} "
                            f"[ERROR: Invalid line range. File has {len(file_lines)} lines]"
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
        inputs: dict[str, Any],
        tool_factory: Callable,
        validate_files: list[str],
        node_type: str,
        state: Any = None,
        max_retries: int = 3,
    ) -> tuple[Any, dict[str, Any], str]:
        """
        Reusable execution loop for DSPy nodes with retries and validation.
        Returns (prediction, artifacts, journal_entry).
        """
        from controller.agent.dspy_utils import WorkerInterpreter
        from worker_heavy.utils.file_validation import validate_node_output

        interpreter = WorkerInterpreter(
            worker_client=self.ctx.worker_client, session_id=self.ctx.session_id
        )
        tool_fns = self._get_tool_functions(tool_factory)

        # WP07: Inject system instructions from prompts.yaml into signature
        # Template names are now standardized to match node_type or explicit mappings
        instructions = self.ctx.pm.render(node_type)
        if instructions:
            signature_cls = signature_cls.with_instructions(instructions)

        program = program_cls(
            signature_cls, tools=list(tool_fns.values()), interpreter=interpreter
        )

        # WP07: Try to load compiled prompt if available
        self.ctx.pm.load_compiled_program(node_type, program)

        # WP08: Set node_type on mock LM if available for explicit lookup
        if hasattr(self.ctx.dspy_lm, "node_type"):
            self.ctx.dspy_lm.node_type = node_type

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
                            session_id=self.ctx.session_id,
                        )
                        # Add a timeout to prevent infinite hangs in DSPy CodeAct
                        try:
                            prediction = await asyncio.wait_for(
                                asyncio.to_thread(program, **inputs),
                                timeout=300.0,  # 5 minutes
                            )
                        except TimeoutError:
                            logger.error(
                                f"{node_type}_dspy_timeout",
                                session_id=self.ctx.session_id,
                            )
                            raise RuntimeError(
                                f"DSPy program {node_type} timed out after 300s"
                            )

                        logger.info(
                            f"{node_type}_dspy_invoke_complete",
                            session_id=self.ctx.session_id,
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
                        await asyncio.sleep(1)  # Backoff to prevent log explosion
                        continue

                    return prediction, artifacts, journal_entry

                except Exception as e:
                    logger.error(f"{node_type}_dspy_failed", error=str(e))
                    journal_entry += f"\n[System Error] {e}"
                    retry_count += 1
                    await asyncio.sleep(1)  # Backoff to prevent log explosion

            journal_entry += "\nMax retries reached."
            return None, artifacts, journal_entry

        finally:
            interpreter.shutdown()
