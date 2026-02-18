from contextlib import suppress
from enum import StrEnum

import dspy
import structlog
from langchain_core.messages import AIMessage
from pydantic import BaseModel, Field

from controller.agent.config import settings
from controller.agent.state import AgentState, AgentStatus
from langchain_core.tools import tool

from controller.agent.tools import get_engineer_tools
from controller.observability.tracing import record_worker_events
from shared.observability.schemas import ReviewDecisionEvent
from shared.type_checking import type_check

from .base import BaseNode, SharedNodeContext

logger = structlog.get_logger(__name__)


class CriticDecision(StrEnum):
    APPROVE = "APPROVE"
    REJECT_PLAN = "REJECT_PLAN"
    REJECT_CODE = "REJECT_CODE"
    CONFIRM_PLAN_REFUSAL = "CONFIRM_PLAN_REFUSAL"
    REJECT_PLAN_REFUSAL = "REJECT_PLAN_REFUSAL"


class ReviewResult(BaseModel):
    """Structured output for the reviewer."""

    decision: CriticDecision
    reason: str
    required_fixes: list[str] = Field(default_factory=list)


class ReviewerSignature(dspy.Signature):
    """
    Reviewer node: Evaluates the Coder's output based on simulation and workbench reports.
    You must use the provided tools to read 'simulation_report.json' and 'workbench_report.md'.
    When done, use SUBMIT to provide your final ReviewResult.
    """

    task = dspy.InputField()
    plan = dspy.InputField()
    todo = dspy.InputField()
    assembly_definition = dspy.InputField()
    objectives = dspy.InputField()
    journal = dspy.InputField()
    plan_refusal = dspy.InputField()
    review: ReviewResult = dspy.OutputField()


@type_check
class ReviewerNode(BaseNode):
    """
    Reviewer node: Evaluates the Coder's output based on simulation and workbench reports.
    Refactored to use DSPy CodeAct with remote worker execution.
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

        # Check for plan refusal from the engineer
        plan_refusal = "No plan refusal recorded."
        with suppress(Exception):
            if await self.ctx.worker_client.exists("plan_refusal.md"):
                plan_refusal = await self.ctx.worker_client.read_file("plan_refusal.md")

        inputs = {
            "task": state.task,
            "plan": state.plan,
            "todo": state.todo,
            "assembly_definition": assembly_definition,
            "objectives": objectives,
            "journal": state.journal,
            "plan_refusal": plan_refusal,
        }

        # Validate existence of key reports
        validate_files = ["simulation_result.json", "assembly_definition.yaml"]

        # Ensure review directory exists and N is correct
        review_round = state.turn_count // 5 + 1 # Rough heuristic or we can use a dedicated field
        # Actually turn_count is not ideal. Let's use a simpler one.
        # AgentState doesn't have review_round. BenchmarkState does.
        # I'll use iteration for now.
        review_round = state.iteration + 1
        review_path = f"reviews/review-round-{review_round}/review.md"

        def get_reviewer_tools(fs, session_id):
            tools = get_engineer_tools(fs, session_id)

            @tool
            async def write_review_file(content: str):
                """
                Write the review to the review file.
                Must include YAML frontmatter with decision and comments.
                """
                await fs.write_file(review_path, content, overwrite=True)
                return f"Review written to {review_path}"

            tools.append(write_review_file)
            return tools

        prediction, artifacts, journal_entry = await self._run_program(
            program_cls=dspy.CodeAct,
            signature_cls=ReviewerSignature,
            state=state,
            inputs=inputs,
            tool_factory=get_reviewer_tools,
            validate_files=validate_files + [review_path],
            node_type="reviewer",
        )

        if not prediction:
            return state.model_copy(
                update={
                    "status": AgentStatus.CODE_REJECTED,
                    "feedback": f"Reviewer failed to complete: {journal_entry}",
                    "journal": state.journal + journal_entry,
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
            CriticDecision.APPROVE: AgentStatus.APPROVED,
            CriticDecision.REJECT_PLAN: AgentStatus.PLAN_REJECTED,
            CriticDecision.REJECT_CODE: AgentStatus.CODE_REJECTED,
            CriticDecision.CONFIRM_PLAN_REFUSAL: AgentStatus.PLAN_REJECTED,
            CriticDecision.REJECT_PLAN_REFUSAL: AgentStatus.CODE_REJECTED,
        }

        # Emit ReviewDecisionEvent for observability
        await record_worker_events(
            episode_id=state.session_id,
            events=[
                ReviewDecisionEvent(
                    decision=decision.value.lower(),
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
                "messages": state.messages
                + [AIMessage(content=f"Review decision: {decision.value}")],
            }
        )


# Factory function for LangGraph
@type_check
async def reviewer_node(state: AgentState) -> AgentState:
    # Use session_id from state
    session_id = state.session_id or settings.default_session_id
    ctx = SharedNodeContext.create(
        worker_url=settings.spec_001_api_url, session_id=session_id
    )
    node = ReviewerNode(context=ctx)
    return await node(state)
