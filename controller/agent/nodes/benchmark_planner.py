from contextlib import suppress

import dspy
import structlog
from langchain_core.messages import AIMessage

from controller.agent.config import settings
from controller.agent.state import AgentState, AgentStatus
from controller.agent.tools import get_benchmark_tools
from controller.observability.tracing import record_worker_events
from shared.observability.schemas import SubmissionValidationEvent
from shared.type_checking import type_check

from .base import BaseNode, SharedNodeContext

logger = structlog.get_logger(__name__)


class BenchmarkPlannerSignature(dspy.Signature):
    """
    Benchmark Planner node: Designs a new benchmark challenge.
    You must create 'plan.md', 'todo.md', and 'objectives.yaml'.
    The plan must detail the learning objective and geometry.
    When done, use SUBMIT to provide a summary of your benchmark plan.
    """

    task = dspy.InputField()
    skills = dspy.InputField()
    feedback = dspy.InputField()
    summary = dspy.OutputField(desc="A summary of the benchmark plan created")


@type_check
class BenchmarkPlannerNode(BaseNode):
    async def __call__(self, state: AgentState) -> AgentState:
        skills_context = self._get_skills_context()

        inputs = {
            "task": state.task,
            "skills": skills_context,
            "feedback": (
                state.feedback
                if state.status == AgentStatus.PLAN_REJECTED
                else "New benchmark planning."
            ),
        }
        validate_files = ["plan.md", "todo.md", "objectives.yaml"]

        prediction, artifacts, journal_entry = await self._run_program(
            dspy.ReAct,
            BenchmarkPlannerSignature,
            state,
            inputs,
            get_benchmark_tools,
            validate_files,
            "benchmark_planner",
        )

        if not prediction:
            return state.model_copy(
                update={
                    "status": AgentStatus.FAILED,
                    "journal": state.journal + journal_entry,
                    "turn_count": state.turn_count + 1,
                }
            )

        await record_worker_events(
            episode_id=state.episode_id,
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
                "journal": state.journal + f"\n[Benchmark Planner] {summary}" + journal_entry,
                "messages": state.messages
                + [AIMessage(content=f"Benchmark Plan summary: {summary}")],
                "turn_count": state.turn_count + 1,
            }
        )


@type_check
async def benchmark_planner_node(state: AgentState) -> AgentState:
    session_id = state.session_id or settings.default_session_id
    episode_id = state.episode_id
    ctx = SharedNodeContext.create(
        worker_light_url=settings.spec_001_api_url,
        session_id=session_id,
        episode_id=episode_id,
        agent_role="benchmark_planner",
    )
    node = BenchmarkPlannerNode(context=ctx)
    return await node(state)
