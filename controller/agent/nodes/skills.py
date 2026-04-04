import uuid
from pathlib import Path

import dspy
import structlog
from langchain_core.messages import AIMessage
from langchain_core.runnables import RunnableConfig

from controller.agent.config import settings
from controller.agent.state import AgentState
from controller.agent.tools import filter_tools_for_agent, get_engineer_tools
from controller.observability.tracing import record_worker_events, sync_asset
from shared.enums import AgentName
from shared.observability.schemas import SkillEditEvent
from shared.type_checking import type_check
from worker_light.utils.git import commit_all, init_workspace_repo

from .base import BaseNode, SharedNodeContext

logger = structlog.get_logger(__name__)


class SkillsSignature(dspy.Signature):
    """DSPy signature for the skill learner subagent."""

    task = dspy.InputField()
    journal = dspy.InputField()
    summary = dspy.OutputField(desc="A summary of the skills identified and saved")


@type_check
class SkillsNode(BaseNode):
    """
    Skills node: Analyzes the journal to suggest new skills.
    Refactored to use DSPy ReAct with remote worker execution.
    """

    def __init__(
        self, context: SharedNodeContext, suggested_skills_dir: str = "suggested_skills"
    ):
        super().__init__(context)
        self.suggested_skills_dir = Path(suggested_skills_dir)
        self.suggested_skills_dir.mkdir(parents=True, exist_ok=True)
        init_workspace_repo(self.suggested_skills_dir)

    async def __call__(
        self, state: AgentState, _config: RunnableConfig | None = None
    ) -> AgentState:
        """Execute the sidecar node logic using DSPy ReAct."""

        async def save_suggested_skill(title: str, content: str) -> str:
            """
            Saves a new suggested skill to the repository.
            Args:
                title: A concise title for the skill (e.g., 'motor_initialization').
                content: The markdown content describing the skill.
            """
            clean_title = title.strip().lower().replace(" ", "_").replace(".md", "")
            file_path = self.suggested_skills_dir / f"{clean_title}.md"

            # T030: Safety toggle for deletions
            if file_path.exists():
                import difflib

                old_lines = file_path.read_text().splitlines()
                new_lines = content.splitlines()
                diff = list(difflib.unified_diff(old_lines, new_lines, n=0))
                # Count lines starting with '-' but not '---'
                deletions = sum(
                    1
                    for line in diff
                    if line.startswith("-") and not line.startswith("---")
                )
                if deletions > 15:
                    logger.warning(
                        "skill_update_blocked_too_many_deletions", deletions=deletions
                    )
                    # Emit safety event
                    await record_worker_events(
                        episode_id=self.ctx.session_id,
                        events=[
                            SkillEditEvent(
                                skill_name=clean_title,
                                action="update_blocked",
                                lines_changed=deletions,
                            )
                        ],
                    )
                    return (
                        "Error: Update blocked. You deleted "
                        f"{deletions} lines, but the limit is 15 lines of deletion "
                        "to prevent skill loss."
                    )

            try:
                with file_path.open("w") as f:
                    f.write(content)

                # Sync to Database as Asset
                try:
                    episode_uuid = uuid.UUID(self.ctx.session_id)
                    await sync_asset(
                        episode_uuid, f"suggested_skills/{clean_title}.md", content
                    )
                except Exception as e:
                    logger.warning(f"Failed to sync skill asset: {e}")

                commit_all(
                    self.suggested_skills_dir,
                    f"Checkpoint suggested skill: {clean_title}",
                )

                return (
                    "Skill '"
                    f"{clean_title}"
                    "' saved and checkpointed locally successfully."
                )
            except Exception as e:
                return f"Error saving skill: {e}"

        def get_skills_tools(fs, session_id):
            tools = get_engineer_tools(fs, session_id)
            return filter_tools_for_agent(fs, [*tools, save_suggested_skill])

        inputs = {
            "task": state.task,
            "journal": state.journal,
        }

        validate_files = []

        prediction, _, journal_entry = await self._run_program(
            dspy.ReAct,
            SkillsSignature,
            state,
            inputs,
            get_skills_tools,
            validate_files,
            AgentName.SKILL_AGENT,
        )

        if not prediction:
            return state.model_copy(
                update={
                    "journal": state.journal + f"\n[Skills] Failed: {journal_entry}"
                }
            )

        summary = getattr(prediction, "summary", "No summary provided.")
        full_journal_entry = f"\nSidecar Learner: {summary}" + journal_entry

        return state.model_copy(
            update={
                "journal": state.journal + full_journal_entry,
                "messages": [
                    *state.messages,
                    AIMessage(content=f"Skills update: {summary}"),
                ],
            }
        )


# Factory function for LangGraph
@type_check
async def skills_node(state: AgentState) -> AgentState:
    session_id = state.session_id
    if not session_id:
        msg = "Missing required session_id for skills_node"
        raise ValueError(msg)
    episode_id = state.episode_id
    ctx = SharedNodeContext.create(
        worker_light_url=settings.worker_light_url,
        session_id=session_id,
        episode_id=episode_id,
        agent_role=AgentName.SKILL_AGENT,
    )
    node = SkillsNode(context=ctx)
    return await node(state)
