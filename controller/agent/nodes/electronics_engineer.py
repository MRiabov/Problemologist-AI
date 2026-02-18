import asyncio
import structlog
from contextlib import suppress

import dspy
from langchain_core.messages import AIMessage, HumanMessage, SystemMessage

from controller.agent.config import settings
from controller.agent.state import AgentState, AgentStatus
from controller.agent.tools import get_engineer_tools
from controller.observability.tracing import record_worker_events
from shared.observability.schemas import ElecAgentHandoverEvent
from shared.type_checking import type_check

from .base import BaseNode, SharedNodeContext

logger = structlog.get_logger(__name__)


class ElectronicsSignature(dspy.Signature):
    """
    Electronics Engineer node: Designs circuits and routes wires.
    You must use the provided tools to implement the current step in 'script.py'.
    When done, use SUBMIT to provide a summary of your work.
    """

    current_step = dspy.InputField()
    plan = dspy.InputField()
    assembly_context = dspy.InputField()
    journal = dspy.OutputField(desc="A summary of the electronics work done")


@type_check
class ElectronicsEngineerNode(BaseNode):
    """
    Electronics Engineer node: Designs circuits and routes wires.
    Refactored to use DSPy CodeAct with remote worker execution.
    """

    async def __call__(self, state: AgentState) -> AgentState:
        """Execute the electronics engineer node logic."""
        # 1. Find next electronics task in TODO
        todo = state.todo
        current_step = self._get_next_electronics_step(todo)

        if not current_step:
            try:
                objectives_content = await self.ctx.fs.read_file("objectives.yaml")
                if "electronics_requirements" not in objectives_content:
                    return state
                current_step = "Design circuit and route wires"
            except Exception:
                return state

        # 2. Get current assembly context
        assembly_context = "No assembly context available."
        try:
            with suppress(Exception):
                assembly_context = await self.ctx.fs.read_file(
                    "assembly_definition.yaml"
                )
        except Exception:
            pass

        # 3. Handover event
        await record_worker_events(
            episode_id=state.session_id,
            events=[
                ElecAgentHandoverEvent(
                    from_agent="Mechanical Engineer",
                    to_agent="Electronics Engineer",
                    iteration_count=state.iteration,
                )
            ],
        )

        from controller.agent.dspy_utils import WorkerInterpreter

        # Use WorkerInterpreter for remote execution
        interpreter = WorkerInterpreter(
            worker_client=self.ctx.worker_client, session_id=state.session_id
        )

        # Get tool signatures for DSPy
        tool_fns = self._get_tool_functions(get_engineer_tools)

        program = dspy.CodeAct(
            ElectronicsSignature, tools=list(tool_fns.values()), interpreter=interpreter
        )

        max_retries = 3
        retry_count = 0
        journal_entry = f"\n[Electronics Engineer] Starting task: {current_step}"

        while retry_count < max_retries:
            try:
                # Invoke DSPy
                with dspy.settings.context(lm=self.ctx.dspy_lm):
                    logger.info("electronics_dspy_invoke_start", session_id=state.session_id)
                    prediction = program(
                        current_step=current_step,
                        plan=state.plan,
                        assembly_context=assembly_context,
                    )
                    logger.info("electronics_dspy_invoke_complete", session_id=state.session_id)

                # 4. Validate output
                from worker.utils.file_validation import validate_node_output

                # Read current files concurrently to validate
                files_to_read = ["plan.md", "todo.md", "assembly_definition.yaml", "script.py"]
                results = await asyncio.gather(
                    *[self.ctx.fs.read_file(f) for f in files_to_read],
                    return_exceptions=True,
                )
                all_files = {
                    f: res
                    for f, res in zip(files_to_read, results)
                    if not isinstance(res, Exception)
                }

                is_valid, validation_errors = validate_node_output(
                    "electronics_engineer", all_files
                )

                if not is_valid:
                    logger.warning("electronics_engineer_validation_failed", errors=validation_errors)
                    retry_count += 1
                    continue

                # Success
                summary = getattr(prediction, "journal", f"Successfully completed electronics task: {current_step}")
                journal_entry += f"\n[Electronics] {summary}"
                new_todo = todo
                if current_step in todo:
                    new_todo = todo.replace(
                        f"- [ ] {current_step}", f"- [x] {current_step}"
                    )

                return state.model_copy(
                    update={
                        "todo": new_todo,
                        "journal": state.journal + journal_entry,
                        "turn_count": state.turn_count + 1,
                        "messages": state.messages + [AIMessage(content=f"Electronics summary: {summary}")],
                    }
                )

            except Exception as e:
                logger.error("electronics_dspy_failed", error=str(e))
                journal_entry += f"\n[System Error] {e}"
                retry_count += 1
            finally:
                interpreter.shutdown()

        return state.model_copy(
            update={
                "journal": state.journal + journal_entry + "\nMax retries reached.",
                "turn_count": state.turn_count + 1,
            }
        )

    def _get_next_electronics_step(self, todo: str) -> str | None:
        """Extract the first electronics-related '- [ ]' item."""
        elec_keywords = ["circuit", "wire", "electronics", "routing", "psu", "power"]
        for line in todo.split("\n"):
            if line.strip().startswith("- [ ]"):
                task = line.strip().replace("- [ ]", "").strip()
                if any(kw in task.lower() for kw in elec_keywords):
                    return task
        return None


# Factory function for LangGraph
@type_check
async def electronics_engineer_node(state: AgentState) -> AgentState:
    # Use session_id from state, fallback to default if not set (e.g. tests)
    session_id = state.session_id or settings.default_session_id
    ctx = SharedNodeContext.create(
        worker_url=settings.spec_001_api_url, session_id=session_id
    )
    node = ElectronicsEngineerNode(context=ctx)
    return await node(state)
