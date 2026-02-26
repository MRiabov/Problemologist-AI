from contextlib import suppress

import dspy
import structlog
from langchain_core.messages import AIMessage

from controller.agent.config import settings
from controller.agent.state import AgentState, AgentStatus
from controller.agent.tools import get_engineer_tools
from controller.observability.tracing import record_worker_events
from shared.enums import ReviewDecision
from shared.models.schemas import ReviewResult
from shared.observability.schemas import ReviewDecisionEvent
from shared.type_checking import type_check

from .base import BaseNode, SharedNodeContext

logger = structlog.get_logger(__name__)


class ExecutionReviewerSignature(dspy.Signature):
    """
    Execution Reviewer node: Evaluates the implementation based on simulation and workbench reports.
    You must use the provided tools to read 'simulation_result.json' and 'workbench_report.md'.
    When done, use SUBMIT to provide your final ReviewResult.
    """

    task = dspy.InputField()
    plan = dspy.InputField()
    todo = dspy.InputField()
    assembly_definition = dspy.InputField()
    objectives = dspy.InputField()
    journal = dspy.InputField()
    review: ReviewResult = dspy.OutputField()


@type_check
class ExecutionReviewerNode(BaseNode):
    """
    Execution Reviewer node: Evaluates the implementation after simulation.
    """

    async def __call__(self, state: AgentState) -> AgentState:
        # T015: Use DSPy to evaluate success
        # Read objectives if possible for context
        objectives = "# No objectives.yaml found."
        with suppress(Exception):
            if await self.ctx.worker_client.exists("objectives.yaml"):
                objectives = await self.ctx.worker_client.read_file("objectives.yaml")

        assembly_definition = "# No assembly_definition.yaml found."
        with suppress(Exception):
            if await self.ctx.worker_client.exists("assembly_definition.yaml"):
                assembly_definition = await self.ctx.worker_client.read_file(
                    "assembly_definition.yaml"
                )

        inputs = {
            "task": state.task,
            "plan": state.plan,
            "todo": state.todo,
            "assembly_definition": assembly_definition,
            "objectives": objectives,
            "journal": state.journal,
        }

        # Validate existence of key reports
        validate_files = ["simulation_result.json", "assembly_definition.yaml"]

        prediction, _artifacts, journal_entry = await self._run_program(
            program_cls=dspy.ReAct,
            signature_cls=ExecutionReviewerSignature,
            state=state,
            inputs=inputs,
            tool_factory=get_engineer_tools,
            validate_files=validate_files,
            node_type="execution_reviewer",
        )

        if not prediction:
            return state.model_copy(
                update={
                    "status": AgentStatus.CODE_REJECTED,
                    "feedback": f"Execution Reviewer failed to complete: {journal_entry}",
                    "journal": state.journal + journal_entry,
                    "turn_count": state.turn_count + 1,
                }
            )

        review = prediction.review
        decision = review.decision
        feedback = review.reason
        if review.required_fixes:
            feedback += "\nRequired Fixes:\n" + "\n".join(
                [f"- {f}" for f in review.required_fixes]
            )

        journal_entry += f"\nCritic Decision: {decision.value}\nFeedback: {feedback}"

        status_map = {
            ReviewDecision.APPROVED: AgentStatus.APPROVED,
            ReviewDecision.REJECTED: AgentStatus.CODE_REJECTED,
            ReviewDecision.REJECT_PLAN: AgentStatus.PLAN_REJECTED,
            ReviewDecision.REJECT_CODE: AgentStatus.CODE_REJECTED,
        }

        # Emit ReviewDecisionEvent for observability
        await record_worker_events(
            episode_id=state.episode_id,
            events=[
                ReviewDecisionEvent(
                    decision=decision,
                    reason=feedback,
                    evidence_stats={
                        "has_sim_report": True,
                        "has_mfg_report": True,
                    },
                )
            ],
        )

        return state.model_copy(
            update={
                "status": status_map.get(decision, AgentStatus.CODE_REJECTED),
                "feedback": feedback,
                "journal": state.journal + journal_entry,
                "messages": [
                    *state.messages,
                    AIMessage(content=f"Review decision: {decision.value}"),
                ],
                "turn_count": state.turn_count + 1,
            }
        )


# Factory function for LangGraph
@type_check
async def execution_reviewer_node(state: AgentState) -> AgentState:
    # Use session_id from state
    session_id = state.session_id or settings.default_session_id

    if state.context:
        ctx = state.context
    else:
        ctx = SharedNodeContext.create(
            worker_light_url=settings.spec_001_api_url, session_id=session_id
        )
        state.context = ctx
    node = ExecutionReviewerNode(context=ctx)
    return await node(state)
