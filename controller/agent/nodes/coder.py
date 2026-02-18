import asyncio
import structlog
from contextlib import suppress

import dspy
from langchain_core.messages import AIMessage, HumanMessage, SystemMessage

from controller.agent.config import settings
from controller.agent.state import AgentState, AgentStatus
from controller.agent.tools import get_engineer_tools
from shared.type_checking import type_check

from .base import BaseNode, SharedNodeContext

logger = structlog.get_logger(__name__)


class CoderSignature(dspy.Signature):
    """
    Coder node: Picks a task from TODO, writes code, executes it, and fixes errors.
    You must use the provided tools to implement the current step in 'script.py'.
    When done, use SUBMIT to provide a summary of your work.
    """

    current_step = dspy.InputField()
    plan = dspy.InputField()
    todo = dspy.InputField()
    journal = dspy.OutputField(desc="A summary of the implementation done for this step")


@type_check
class CoderNode(BaseNode):
    """
    Coder node: Picks a task from TODO, writes code, executes it, and fixes errors.
    Refactored to use DSPy CodeAct with remote worker execution.
    """

    async def __call__(self, state: AgentState) -> AgentState:
        """Execute the coder node logic."""
        # T010: Find next active item in TODO
        todo = state.todo
        current_step = self._get_next_step(todo)
        if not current_step:
            return state.model_copy(
                update={"journal": state.journal + "\nNo more steps in TODO."}
            )

        from controller.agent.dspy_utils import WorkerInterpreter

        # Use WorkerInterpreter for remote execution
        interpreter = WorkerInterpreter(
            worker_client=self.ctx.worker_client, session_id=state.session_id
        )

        # Get tool signatures for DSPy
        tool_fns = self._get_tool_functions(get_engineer_tools)

        program = dspy.CodeAct(
            CoderSignature, tools=list(tool_fns.values()), interpreter=interpreter
        )

        max_retries = 3
        retry_count = 0
        journal_entry = f"\nStarting step: {current_step}"

        while retry_count < max_retries:
            try:
                # Invoke DSPy
                with dspy.settings.context(lm=self.ctx.dspy_lm):
                    logger.info("coder_dspy_invoke_start", session_id=state.session_id)
                    prediction = program(
                        current_step=current_step,
                        plan=state.plan,
                        todo=todo,
                    )
                    logger.info("coder_dspy_invoke_complete", session_id=state.session_id)

                # Validation Gate
                from worker.utils.file_validation import validate_node_output

                # Read current files concurrently to validate
                files_to_read = [
                    "plan.md",
                    "todo.md",
                    "objectives.yaml",
                    "assembly_definition.yaml",
                    "script.py",
                ]
                results = await asyncio.gather(
                    *[self.ctx.fs.read_file(f) for f in files_to_read],
                    return_exceptions=True,
                )
                all_files = {
                    f: res
                    for f, res in zip(files_to_read, results)
                    if not isinstance(res, Exception)
                }

                is_valid, validation_errors = validate_node_output("coder", all_files)

                if not is_valid:
                    logger.warning("coder_validation_failed", errors=validation_errors)
                    retry_count += 1
                    continue

                # Success
                summary = getattr(prediction, "journal", f"Successfully executed step: {current_step}")
                journal_entry += f"\n[Coder] {summary}"

                # Mark TODO as done
                new_todo = todo.replace(
                    f"- [ ] {current_step}", f"- [x] {current_step}"
                )

                return state.model_copy(
                    update={
                        "todo": new_todo,
                        "journal": state.journal + journal_entry,
                        "current_step": current_step,
                        "messages": state.messages + [AIMessage(content=f"Coder summary: {summary}")],
                        "turn_count": state.turn_count + 1,
                    }
                )

            except Exception as e:
                logger.error("coder_dspy_failed", error=str(e))
                journal_entry += f"\n[System Error] {e}"
                retry_count += 1
            finally:
                interpreter.shutdown()

        return state.model_copy(
            update={
                "journal": state.journal + journal_entry + "\nMax retries reached.",
                "iteration": state.iteration + 1,
                "turn_count": state.turn_count + 1,
            }
        )

    def _get_next_step(self, todo: str) -> str | None:
        """Extract the first '- [ ]' item from the TODO list, ignoring electronics tasks."""
        elec_keywords = ["circuit", "wire", "electronics", "routing", "psu", "power"]
        for line in todo.split("\n"):
            if line.strip().startswith("- [ ]"):
                task = line.strip().replace("- [ ]", "").strip()
                # If it's an electronics task, skip it (ElectronicsEngineer will handle it)
                if any(kw in task.lower() for kw in elec_keywords):
                    continue
                return task
        return None


# Factory function for LangGraph
@type_check
async def coder_node(state: AgentState) -> AgentState:
    # Use session_id from state, fallback to default if not set (e.g. tests)
    session_id = state.session_id or settings.default_session_id
    ctx = SharedNodeContext.create(
        worker_url=settings.spec_001_api_url, session_id=session_id
    )
    node = CoderNode(context=ctx)
    return await node(state)
