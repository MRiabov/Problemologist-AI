import structlog
from contextlib import suppress

import dspy
from langchain_core.messages import AIMessage, HumanMessage, SystemMessage

from controller.agent.config import settings
from controller.agent.state import AgentState, AgentStatus
from controller.agent.tools import get_engineer_tools
from controller.observability.tracing import record_worker_events
from shared.observability.schemas import SubmissionValidationEvent
from shared.type_checking import type_check

from .base import BaseNode, SharedNodeContext

logger = structlog.get_logger(__name__)


class PlannerSignature(dspy.Signature):
    """
    Planner node: Analyzes the task and creates plan.md and todo.md using tools.
    You must use the provided tools to create 'plan.md' and 'todo.md' directly.
    When done, use SUBMIT to provide a summary of your plan.
    """

    task = dspy.InputField()
    skills = dspy.InputField()
    steer_context = dspy.InputField()
    summary = dspy.OutputField(desc="A summary of the plan created")


@type_check
class PlannerNode(BaseNode):
    """
    Planner node: Analyzes the task and creates plan.md and todo.md using tools.
    Refactored to use DSPy CodeAct with remote worker execution.
    """

    async def __call__(self, state: AgentState) -> AgentState:
        """Execute the planner node logic."""
        # T006: Read skills
        skills_context = self._get_skills_context()
        # WP04: Extract steerability context
        steer_context = self._get_steer_context(state.messages)

        from controller.agent.dspy_utils import WorkerInterpreter

        # Use WorkerInterpreter for remote execution
        interpreter = WorkerInterpreter(
            worker_client=self.ctx.worker_client, session_id=state.session_id
        )

        # Get tool signatures for DSPy
        tool_fns = self._get_tool_functions(get_engineer_tools)

        program = dspy.CodeAct(
            PlannerSignature, tools=list(tool_fns.values()), interpreter=interpreter
        )

        max_retries = 3
        retry_count = 0
        journal_entry = "\n[Planner] Starting planning phase."

        while retry_count < max_retries:
            try:
                with dspy.settings.context(lm=self.ctx.dspy_lm):
                    logger.info("planner_dspy_invoke_start", session_id=state.session_id)
                    prediction = program(
                        task=state.task,
                        skills=skills_context,
                        steer_context=steer_context,
                    )
                    logger.info(
                        "planner_dspy_invoke_complete", session_id=state.session_id
                    )

                # Validation Gate
                from worker.utils.file_validation import validate_node_output

                artifacts = {}
                for f in ["plan.md", "todo.md"]:
                    with suppress(Exception):
                        content = await self.ctx.fs.read_file(f)
                        artifacts[f] = content

                is_valid, validation_errors = validate_node_output("planner", artifacts)

                if not is_valid:
                    logger.warning("planner_validation_failed", errors=validation_errors)
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

                summary = getattr(prediction, "summary", "No summary provided.")
                return state.model_copy(
                    update={
                        "plan": artifacts.get("plan.md", ""),
                        "todo": artifacts.get("todo.md", ""),
                        "status": AgentStatus.EXECUTING,
                        "journal": state.journal + f"\n[Planner] {summary}",
                        "messages": state.messages
                        + [AIMessage(content=f"Plan summary: {summary}")],
                    }
                )

            except Exception as e:
                logger.error("planner_dspy_failed", error=str(e))
                retry_count += 1
            finally:
                interpreter.shutdown()

        return state.model_copy(
            update={
                "status": AgentStatus.FAILED,
                "journal": state.journal + journal_entry + "\nMax retries reached.",
            }
        )


# Factory function for LangGraph
@type_check
async def planner_node(state: AgentState) -> AgentState:
    # Use session_id from state, fallback to default if not set (e.g. tests)
    session_id = state.session_id or settings.default_session_id
    ctx = SharedNodeContext.create(
        worker_url=settings.spec_001_api_url, session_id=session_id
    )
    node = PlannerNode(context=ctx)
    return await node(state)
