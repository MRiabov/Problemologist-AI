import os
import uuid
from pathlib import Path

import dspy
import structlog
from langchain_core.messages import AIMessage
from langchain_core.runnables import RunnableConfig

from controller.agent.config import settings
from controller.agent.state import AgentState
from controller.agent.tools import get_engineer_tools
from controller.observability.tracing import record_worker_events, sync_asset
from controller.utils.git import GitManager
from shared.observability.schemas import SkillEditEvent
from shared.type_checking import type_check

from .base import BaseNode, SharedNodeContext

logger = structlog.get_logger(__name__)


class SkillsSignature(dspy.Signature):
    """
    Skills node: Analyzes the journal to suggest new skills.
    You must use the provided tools to analyze the work and save new skills.
    If you identify a valuable skill or pattern, use the `save_suggested_skill` tool to record it.
    When done, use SUBMIT to provide a summary of your work.
    """

    task = dspy.InputField()
    journal = dspy.InputField()
    summary = dspy.OutputField(desc="A summary of the skills identified and saved")


@type_check
class SkillsNode(BaseNode):
    """
    Skills node: Analyzes the journal to suggest new skills.
    Refactored to use DSPy CodeAct with remote worker execution.
    """

    def __init__(
        self, context: SharedNodeContext, suggested_skills_dir: str = "suggested_skills"
    ):
        super().__init__(context)
        self.suggested_skills_dir = Path(suggested_skills_dir)
        self.suggested_skills_dir.mkdir(parents=True, exist_ok=True)
        self.git = GitManager(
            repo_path=self.suggested_skills_dir,
            repo_url=os.getenv("GIT_REPO_URL"),
            pat=os.getenv("GIT_PAT"),
        )
        self.git.ensure_repo()

    async def _sync_git(self, commit_message: str):
        """Sync changes with git via GitManager."""
        await self.git.sync_changes(commit_message, llm=self.ctx.llm, pm=self.ctx.pm)

    async def __call__(
        self, state: AgentState, config: RunnableConfig | None = None
    ) -> AgentState:
        """Execute the sidecar node logic using DSPy CodeAct."""

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
                    return f"Error: Update blocked. You deleted {deletions} lines, but the limit is 15 lines of deletion to prevent skill loss."

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

                # Sync to Git
                await self._sync_git(f"Add skill: {clean_title}")

                return f"Skill '{clean_title}' saved and synced successfully."
            except Exception as e:
                return f"Error saving skill: {e}"

        def get_skills_tools(fs, session_id):
            tools = get_engineer_tools(fs, session_id)
            tools.append(save_suggested_skill)
            return tools

        inputs = {
            "task": state.task,
            "journal": state.journal,
        }

        prediction, _, journal_entry = await self._run_program(
            program_cls=dspy.CodeAct,
            signature_cls=SkillsSignature,
            state=state,
            inputs=inputs,
            tool_factory=get_skills_tools,
            validate_files=[],
            node_type="skill_learner",
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
                "messages": state.messages
                + [AIMessage(content=f"Skills update: {summary}")],
            }
        )


# Factory function for LangGraph
@type_check
async def skills_node(state: AgentState) -> AgentState:
    session_id = state.session_id or settings.default_session_id
    ctx = SharedNodeContext.create(
        worker_light_url=settings.spec_001_api_url, session_id=session_id
    )
    node = SkillsNode(context=ctx)
    return await node(state)
