import structlog
import dspy
from contextlib import suppress
from langchain_core.messages import AIMessage

from controller.agent.config import settings
from controller.agent.dspy_utils import init_dspy, wrap_tool_for_dspy
from controller.agent.signatures import ElectronicsEngineerSignature
from controller.agent.state import AgentState
from controller.agent.tools import get_engineer_tools
from controller.observability.tracing import record_worker_events
from shared.observability.schemas import ElecAgentHandoverEvent
from shared.type_checking import type_check

from .base import BaseNode, SharedNodeContext

logger = structlog.get_logger(__name__)


@type_check
class ElectronicsEngineerNode(BaseNode):
    """
    Electronics Engineer node: Designs circuits and routes wires.
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
        self.agent = dspy.CodeAct(ElectronicsEngineerSignature, tools=self.tools)

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

        # 3. Generate electronics implementation
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

        max_retries = 3
        retry_count = 0
        journal_entry = f"\n[Elec Engineer] Starting task: {current_step}"

        while retry_count < max_retries:
            try:
                logger.info(
                    "electronics_agent_dspy_invoke_start",
                    session_id=state.session_id
                )

                # Invoke DSPy CodeAct
                self.agent(
                    task=current_step,
                    plan=state.plan,
                    cad_implementation=assembly_context
                )

                logger.info(
                    "electronics_agent_dspy_invoke_complete",
                    session_id=state.session_id
                )

                # 4. Validate output
                from worker.utils.file_validation import validate_node_output

                all_files = {}
                for f in ["plan.md", "todo.md", "assembly_definition.yaml"]:
                    with suppress(Exception):
                        all_files[f] = await self.ctx.fs.read_file(f)

                is_valid, validation_errors = validate_node_output(
                    "electronics_engineer", all_files
                )

                if not is_valid:
                    journal_entry += (
                        f"\nValidation failed (Attempt {retry_count + 1}): "
                        f"{validation_errors}"
                    )
                    retry_count += 1
                    continue

                # Success
                journal_entry += (
                    f"\nSuccessfully completed electronics task: {current_step}"
                )
                new_todo = todo
                if current_step in todo:
                    new_todo = todo.replace(
                        f"- [ ] {current_step}", f"- [x] {current_step}"
                    )

                elec_msg = (
                    f"Electronics Engineer successfully completed task: "
                    f"{current_step} using DSPy CodeAct."
                )
                return state.model_copy(
                    update={
                        "todo": new_todo,
                        "journal": state.journal + journal_entry,
                        "turn_count": state.turn_count + 1,
                        "messages": [AIMessage(content=elec_msg)],
                    }
                )

            except Exception as e:
                journal_entry += (
                    f"\nSystem error during electronics implementation: {e}"
                )
                logger.error("electronics_engineer_system_error", error=str(e))
                retry_count += 1

        journal_entry += f"\nFailed to complete task after {max_retries} retries."
        return state.model_copy(
            update={
                "journal": state.journal + journal_entry,
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
    # Use session_id from state
    session_id = state.session_id or settings.default_session_id
    ctx = SharedNodeContext.create(
        worker_url=settings.spec_001_api_url, session_id=session_id
    )
    node = ElectronicsEngineerNode(context=ctx)
    return await node(state)
