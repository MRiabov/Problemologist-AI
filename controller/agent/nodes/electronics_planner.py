from contextlib import suppress

import dspy
import structlog
from langchain_core.messages import AIMessage

from controller.agent.config import settings
from controller.agent.state import AgentState, AgentStatus
from controller.agent.tools import get_engineer_planner_tools
from controller.observability.tracing import record_worker_events
from shared.enums import AgentName
from shared.observability.schemas import SubmissionValidationEvent
from shared.type_checking import type_check

from .base import BaseNode, SharedNodeContext

logger = structlog.get_logger(__name__)


class ElectronicsPlannerSignature(dspy.Signature):
    """
    Electronics Planner node: Designs the electrical system and power budget.
    You must use the provided tools to update 'plan.md' and 'todo.md' with electronics tasks.
    You also receive benchmark_definition.yaml and benchmark_assembly_definition.yaml handoff context copied into this workspace.
    benchmark_definition.yaml is the source of truth for benchmark/customer caps
    and may be updated to preserve benchmark-owned geometry/randomization while
    materializing planner-authored benchmark estimates. benchmark_assembly_definition.yaml
    remains benchmark-owned context and must not be treated as a second source for
    benchmark caps.
    Before finishing, you must call `submit_plan()` and only finish when it returns ok=true.
    When done, use SUBMIT to provide a summary of your electrical plan.
    """

    task = dspy.InputField()
    objectives = dspy.InputField()
    benchmark_assembly_definition = dspy.InputField()
    skills = dspy.InputField()
    mechanical_plan = dspy.InputField()
    feedback = dspy.InputField()
    summary = dspy.OutputField(desc="A summary of the electrical plan created")


@type_check
class ElectronicsPlannerNode(BaseNode):
    """
    Electronics Planner node: Specialized electrical system planning.
    """

    async def __call__(self, state: AgentState) -> AgentState:
        """Execute the electronics planner node logic."""
        skills_context = self._get_skills_context()

        # Read objectives for context
        objectives = "# No benchmark_definition.yaml found."
        with suppress(Exception):
            objectives = await self._read_optional_workspace_file(
                "benchmark_definition.yaml", objectives
            )
        benchmark_assembly_definition = await self._read_required_workspace_file(
            "benchmark_assembly_definition.yaml"
        )

        inputs = {
            "task": state.task,
            "objectives": objectives,
            "benchmark_assembly_definition": benchmark_assembly_definition,
            "skills": skills_context,
            "mechanical_plan": state.plan,
            "feedback": (
                state.feedback
                if state.status == AgentStatus.PLAN_REJECTED
                else "New electronics planning. No rejection feedback."
            ),
        }
        validate_files = [
            "plan.md",
            "todo.md",
            "benchmark_definition.yaml",
            "assembly_definition.yaml",
            "benchmark_assembly_definition.yaml",
        ]

        prediction, artifacts, journal_entry = await self._run_program(
            program_cls=dspy.ReAct,
            signature_cls=ElectronicsPlannerSignature,
            state=state,
            inputs=inputs,
            tool_factory=lambda fs, sid: get_engineer_planner_tools(
                fs, sid, planner_node_type=AgentName.ELECTRONICS_PLANNER
            ),
            validate_files=validate_files,
            node_type=AgentName.ELECTRONICS_PLANNER,
        )

        if not prediction:
            return state.model_copy(
                update={
                    "status": AgentStatus.FAILED,
                    "journal": state.journal + journal_entry,
                    "turn_count": state.turn_count + 1,
                }
            )

        submission, submit_err = await self._get_latest_submit_plan_result(
            AgentName.ELECTRONICS_PLANNER
        )
        if submission is None or not submission.ok or submission.status != "submitted":
            submit_errors = (
                [submit_err]
                if submit_err
                else (submission.errors if submission else ["submit_plan failed"])
            )
            return state.model_copy(
                update={
                    "status": AgentStatus.FAILED,
                    "journal": (
                        state.journal
                        + "\n[Electronics Planner] submit_plan validation failed: "
                        + "; ".join([e for e in submit_errors if e])
                        + journal_entry
                    ),
                    "turn_count": state.turn_count + 1,
                }
            )

        # Success
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
                "plan": artifacts.get("plan.md", state.plan),
                "todo": artifacts.get("todo.md", state.todo),
                "journal": state.journal
                + f"\n[Electronics Planner] {summary}"
                + journal_entry,
                "messages": [
                    *state.messages,
                    AIMessage(content=f"Electronics Plan summary: {summary}"),
                ],
                "turn_count": state.turn_count + 1,
            }
        )


# Factory function for LangGraph
@type_check
async def electronics_planner_node(state: AgentState) -> AgentState:
    session_id = state.session_id
    if not session_id:
        msg = "Missing required session_id for electronics_planner_node"
        raise ValueError(msg)
    episode_id = state.episode_id
    ctx = SharedNodeContext.create(
        worker_light_url=settings.worker_light_url,
        session_id=session_id,
        episode_id=episode_id,
        agent_role=AgentName.ELECTRONICS_PLANNER,
    )
    node = ElectronicsPlannerNode(context=ctx)
    return await node(state)
