import logging
from contextlib import suppress

import dspy
from langchain_core.messages import AIMessage

from controller.agent.config import settings
from controller.agent.state import AgentState, AgentStatus
from controller.agent.tools import get_engineer_tools
from controller.agent.signatures import PlannerSignature
from controller.agent.dspy_utils import init_dspy, wrap_tool_for_dspy
from controller.observability.tracing import record_worker_events
from shared.observability.schemas import SubmissionValidationEvent
from shared.type_checking import type_check

from .base import BaseNode, SharedNodeContext

logger = logging.getLogger(__name__)


@type_check
class PlannerNode(BaseNode):
    """
    Planner node: Analyzes the task and creates plan.md and todo.md using tools.
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
        self.planner = dspy.CodeAct(PlannerSignature, tools=self.tools)

    async def __call__(self, state: AgentState) -> AgentState:
        """Execute the planner node logic."""
        # T006: Read skills
        skills_context = self._get_skills_context()

        # WP04: Extract steerability context
        steer_context = self._get_steer_context(state.messages)

        max_retries = 3
        retry_count = 0
        journal_entry = "\n[Planner] Starting planning phase with DSPy CodeAct."

        while retry_count < max_retries:
            try:
                logger.info("planner_dspy_invoke_start", session_id=state.session_id)

                # Invoke DSPy CodeAct
                # In CodeAct, the agent will write and run Python code to call tools
                self.planner(
                    task=state.task,
                    skills=skills_context,
                    steer_context=steer_context
                )

                logger.info("planner_dspy_invoke_complete", session_id=state.session_id)

                # Validation Gate
                from worker.utils.file_validation import validate_node_output

                # Read artifacts to validate
                artifacts = {}
                for f in ["plan.md", "todo.md", "assembly_definition.yaml"]:
                    with suppress(Exception):
                        content = await self.ctx.fs.read_file(f)
                        artifacts[f] = content

                is_valid, validation_errors = validate_node_output("planner", artifacts)

                if not is_valid:
                    journal_entry += f"\nValidation failed (Attempt {retry_count + 1}): {validation_errors}"
                    retry_count += 1
                    continue

                # Success
                await record_worker_events(
                    episode_id=state.session_id,
                    events=[
                        SubmissionValidationEvent(
                            artifacts_present=list(artifacts.keys()),
                            verification_passed=True,
                            reasoning_trace_quality=1.0,
                            errors=[],
                        )
                    ],
                )

                plan_summary = artifacts.get('plan.md', '')[:100]
                return state.model_copy(
                    update={
                        "plan": artifacts.get("plan.md", ""),
                        "todo": artifacts.get("todo.md", ""),
                        "status": AgentStatus.EXECUTING,
                        "journal": state.journal + journal_entry,
                        "messages": [AIMessage(content=f"Planner completed task using DSPy CodeAct. Plan: {plan_summary}...")],
                    }
                )

            except Exception as e:
                journal_entry += f"\nSystem error during planning: {e}"
                retry_count += 1

        return state.model_copy(
            update={
                "status": AgentStatus.FAILED,
                "journal": state.journal + journal_entry,
            }
        )


# Factory function for LangGraph
@type_check
async def planner_node(state: AgentState) -> AgentState:
    # Use session_id from state
    session_id = state.session_id or settings.default_session_id
    ctx = SharedNodeContext.create(
        worker_url=settings.spec_001_api_url, session_id=session_id
    )
    node = PlannerNode(context=ctx)
    return await node(state)
