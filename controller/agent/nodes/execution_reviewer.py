import json
from contextlib import suppress
from typing import Literal

import dspy
import structlog
from langchain_core.messages import AIMessage
from pydantic import BaseModel

from controller.agent.config import settings
from controller.agent.state import AgentState, AgentStatus
from controller.agent.tools import get_engineer_tools
from controller.observability.tracing import record_worker_events
from shared.enums import AgentName, ReviewDecision
from shared.models.schemas import ReviewResult
from shared.observability.schemas import ReviewDecisionEvent
from shared.type_checking import type_check
from shared.workers.schema import BenchmarkToolResponse

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
    plan_refusal = dspy.InputField(default="")
    objectives = dspy.InputField()
    journal = dspy.InputField()
    review: ReviewResult = dspy.OutputField()


class SimulationReviewOutcome(BaseModel):
    """Structured result of execution review simulation pre-check."""

    ran: bool
    success: bool
    summary: str
    source: Literal["simulation", "precheck"]


@type_check
class ExecutionReviewerNode(BaseNode):
    """
    Execution Reviewer node: Evaluates the implementation after simulation.
    """

    async def __call__(self, state: AgentState) -> AgentState:
        submit_err = await self._ensure_submit_for_review_succeeded()
        if submit_err:
            return state.model_copy(
                update={
                    "status": AgentStatus.CODE_REJECTED,
                    "feedback": submit_err,
                    "journal": state.journal + f"\n[Execution Reviewer] {submit_err}",
                    "turn_count": state.turn_count + 1,
                }
            )

        simulation_outcome = await self._run_simulation_review_step()
        simulation_journal = f"\n[Simulation] {simulation_outcome.summary}"
        if not simulation_outcome.success:
            return state.model_copy(
                update={
                    "status": AgentStatus.CODE_REJECTED,
                    "feedback": simulation_outcome.summary,
                    "journal": state.journal + simulation_journal,
                    "turn_count": state.turn_count + 1,
                }
            )

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

        plan_refusal = ""
        with suppress(Exception):
            if await self.ctx.worker_client.exists("plan_refusal.md"):
                plan_refusal = await self.ctx.worker_client.read_file("plan_refusal.md")

        inputs = {
            "task": state.task,
            "plan": state.plan,
            "todo": state.todo,
            "assembly_definition": assembly_definition,
            "plan_refusal": plan_refusal,
            "objectives": objectives,
            "journal": state.journal + simulation_journal,
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
            node_type=AgentName.EXECUTION_REVIEWER,
        )

        if not prediction:
            return state.model_copy(
                update={
                    "status": AgentStatus.CODE_REJECTED,
                    "feedback": f"Execution Reviewer failed to complete: {journal_entry}",
                    "journal": state.journal + simulation_journal + journal_entry,
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
            ReviewDecision.CONFIRM_PLAN_REFUSAL: AgentStatus.FAILED,
            ReviewDecision.REJECT_PLAN_REFUSAL: AgentStatus.CODE_REJECTED,
        }
        if decision not in status_map:
            return state.model_copy(
                update={
                    "status": AgentStatus.FAILED,
                    "feedback": (
                        "Execution Reviewer returned unsupported structured "
                        f"decision: {decision}"
                    ),
                    "journal": (
                        state.journal
                        + simulation_journal
                        + journal_entry
                        + f"\n[Execution Reviewer] Unsupported decision: {decision}"
                    ),
                    "turn_count": state.turn_count + 1,
                }
            )

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
                "status": status_map[decision],
                "feedback": feedback,
                "journal": state.journal + simulation_journal + journal_entry,
                "messages": [
                    *state.messages,
                    AIMessage(content=f"Review decision: {decision.value}"),
                ],
                "turn_count": state.turn_count + 1,
            }
        )

    async def _run_simulation_review_step(self) -> SimulationReviewOutcome:
        if not await self.ctx.worker_client.exists("script.py"):
            return SimulationReviewOutcome(
                ran=False,
                success=False,
                summary="script.py missing; execution review cannot run simulation.",
                source="precheck",
            )

        try:
            sim_result = await self.ctx.worker_client.simulate(script_path="script.py")
            await self._persist_simulation_artifacts(sim_result)
            message = (sim_result.message or "").strip()
            if sim_result.success:
                return SimulationReviewOutcome(
                    ran=True,
                    success=True,
                    summary=message or "Goal achieved.",
                    source="simulation",
                )
            return SimulationReviewOutcome(
                ran=True,
                success=False,
                summary=message or "Simulation failed.",
                source="simulation",
            )
        except Exception as e:
            logger.warning("execution_reviewer_simulation_failed", error=str(e))
            return SimulationReviewOutcome(
                ran=True,
                success=False,
                summary=f"Simulation failed: {e}",
                source="simulation",
            )

    async def _persist_simulation_artifacts(self, sim_result) -> None:
        """Persist real simulation outputs for downstream asset/tracing sync."""
        try:
            sim_payload = {
                "success": bool(getattr(sim_result, "success", False)),
                "summary": str(getattr(sim_result, "message", "") or ""),
                "confidence": str(getattr(sim_result, "confidence", "") or ""),
            }
            await self.ctx.worker_client.write_file(
                "simulation_result.json",
                json.dumps(sim_payload),
                overwrite=True,
            )
        except Exception as e:
            logger.warning(
                "execution_reviewer_persist_simulation_artifacts_failed", error=str(e)
            )

    async def _ensure_submit_for_review_succeeded(self) -> str | None:
        metadata, trace_err = await self._get_latest_tool_trace_metadata(
            "submit_for_review"
        )
        if metadata is None:
            return (
                "Execution review requires explicit submit_for_review() before reviewer "
                f"handoff: {trace_err}"
            )

        try:
            raw = metadata.observation or ""
            parsed = json.loads(raw)
            result = BenchmarkToolResponse.model_validate(parsed)
            if not result.success:
                return (
                    "Execution review blocked: latest submit_for_review() failed: "
                    f"{result.message}"
                )
        except Exception:
            return (
                "Execution review blocked: submit_for_review() observation could not "
                "be validated as BenchmarkToolResponse."
            )

        return None


# Factory function for LangGraph
@type_check
async def execution_reviewer_node(state: AgentState) -> AgentState:
    session_id = state.session_id
    if not session_id:
        msg = "Missing required session_id for execution_reviewer_node"
        raise ValueError(msg)
    episode_id = state.episode_id
    ctx = SharedNodeContext.create(
        worker_light_url=settings.spec_001_api_url,
        session_id=session_id,
        episode_id=episode_id,
        agent_role=AgentName.EXECUTION_REVIEWER,
    )
    node = ExecutionReviewerNode(context=ctx)
    return await node(state)
