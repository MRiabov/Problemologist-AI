from contextlib import suppress

import dspy
import structlog
from langchain_core.messages import AIMessage

from controller.agent.config import settings
from controller.agent.dspy_utils import init_dspy, wrap_tool_for_dspy
from controller.agent.signatures import CoderSignature
from controller.agent.state import AgentState
from controller.agent.tools import get_engineer_tools
from shared.type_checking import type_check

from .base import BaseNode, SharedNodeContext

logger = structlog.get_logger(__name__)


@type_check
class CoderNode(BaseNode):
    """
    Coder node: Picks a task from TODO, writes code, and executes it.
    Refactored to use DSPy CodeAct.
    """

    def __init__(self, context: SharedNodeContext):
        super().__init__(context)
        # Initialize DSPy
        init_dspy(session_id=self.ctx.session_id)
        # Initialize tools
        self.raw_tools = get_engineer_tools(self.ctx.fs, self.ctx.session_id)
        self.tools = [wrap_tool_for_dspy(t) for t in self.raw_tools]

        # Define DSPy CodeAct module
        self.coder = dspy.CodeAct(CoderSignature, tools=self.tools)

    async def __call__(self, state: AgentState) -> AgentState:
        """Execute the coder node logic."""
        # T010: Find next active item in TODO
        todo = state.todo
        current_step = self._get_next_step(todo)
        if not current_step:
            return state.model_copy(
                update={"journal": state.journal + "\nNo more steps in TODO."}
            )

        max_retries = 3
        retry_count = 0
        journal_entry = f"\nStarting step: {current_step} with DSPy CodeAct Coder."
        last_error = ""

        while retry_count < max_retries:
            try:
                logger.info("coder_dspy_invoke_start", session_id=state.session_id)

                # Invoke DSPy CodeAct
                self.coder(
                    current_step=current_step,
                    plan=state.plan,
                    error=last_error
                )

                logger.info("coder_dspy_invoke_complete", session_id=state.session_id)

                # Validation Gate
                from worker.utils.file_validation import validate_node_output

                # Read current files to validate
                all_files = {}
                for f in [
                    "plan.md",
                    "todo.md",
                    "objectives.yaml",
                    "assembly_definition.yaml",
                ]:
                    with suppress(Exception):
                        all_files[f] = await self.ctx.fs.read_file(f)

                is_valid, validation_errors = validate_node_output("coder", all_files)

                if not is_valid:
                    last_error = "Coder produced invalid output:\n" + "\n".join(
                        [f"- {e}" for e in validation_errors]
                    )
                    journal_entry += (
                        f"\nValidation failed (Attempt {retry_count + 1}): "
                        f"{validation_errors}"
                    )
                    retry_count += 1
                    continue

                # Success
                journal_entry += f"\nSuccessfully executed step: {current_step}"

                # Mark TODO as done
                new_todo = todo.replace(
                    f"- [ ] {current_step}", f"- [x] {current_step}"
                )

                coder_msg = f"Coder successfully executed step: {current_step} using DSPy."
                return state.model_copy(
                    update={
                        "todo": new_todo,
                        "journal": state.journal + journal_entry,
                        "current_step": current_step,
                        "turn_count": state.turn_count + 1,
                        "messages": [AIMessage(content=coder_msg)],
                    }
                )

            except Exception as e:
                last_error = str(e)
                journal_entry += f"\nSystem error during execution: {last_error}"
                retry_count += 1

        journal_entry += f"\nFailed to complete step after {max_retries} retries."
        return state.model_copy(
            update={
                "journal": state.journal + journal_entry,
                "iteration": state.iteration + 1,
                "turn_count": state.turn_count + 1,
            }
        )

    def _get_next_step(self, todo: str) -> str | None:
        """Extract the first '- [ ]' item from the TODO list."""
        elec_keywords = ["circuit", "wire", "electronics", "routing", "psu", "power"]
        for line in todo.split("\n"):
            if line.strip().startswith("- [ ]"):
                task = line.strip().replace("- [ ]", "").strip()
                # If it's an electronics task, skip it
                if any(kw in task.lower() for kw in elec_keywords):
                    continue
                return task
        return None


# Factory function for LangGraph
@type_check
async def coder_node(state: AgentState) -> AgentState:
    # Use session_id from state
    session_id = state.session_id or settings.default_session_id
    ctx = SharedNodeContext.create(
        worker_url=settings.spec_001_api_url, session_id=session_id
    )
    node = CoderNode(context=ctx)
    return await node(state)
