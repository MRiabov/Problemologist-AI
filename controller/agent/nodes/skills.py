import logging
import os
import uuid
from pathlib import Path

import dspy
from langchain_core.messages import AIMessage
from langchain_core.runnables import RunnableConfig
from langchain_core.tools import tool

from controller.agent.state import AgentState
from controller.agent.tools import get_engineer_tools
from controller.agent.signatures import SkillsSignature
from controller.agent.dspy_utils import init_dspy, wrap_tool_for_dspy
from controller.observability.tracing import sync_asset
from controller.utils.git import GitManager
from shared.type_checking import type_check

from .base import BaseNode, SharedNodeContext

logger = logging.getLogger(__name__)


@type_check
class SkillsNode(BaseNode):
    """
    Skills node: Analyzes the journal to suggest new skills using DSPy CodeAct.
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

        init_dspy(session_id=self.ctx.session_id)

        # Define the specialized tool for suggesting skills
        @tool
        async def save_suggested_skill(title: str, content: str) -> str:
            """
            Saves a new suggested skill to the repository.
            Args:
                title: A concise title for the skill.
                content: The markdown content describing the skill.
            """
            clean_title = title.strip().lower().replace(" ", "_").replace(".md", "")
            file_path = self.suggested_skills_dir / f"{clean_title}.md"

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

        # Get common tools + specialized tool
        common_tools = get_engineer_tools(self.ctx.fs, self.ctx.session_id)
        raw_tools = [*common_tools, save_suggested_skill]
        tools = [wrap_tool_for_dspy(t) for t in raw_tools]

        # Use DSPy CodeAct
        agent = dspy.CodeAct(SkillsSignature, tools=tools)

        try:
            logger.info("skills_dspy_invoke_start", session_id=state.session_id)
            # Invoke DSPy CodeAct
            result = agent(journal=state.journal, task=state.task)
            logger.info("skills_dspy_invoke_complete", session_id=state.session_id)

            journal_entry = (
                f"\nSidecar Learner: DSPy processing complete. "
                f"{result.suggested_skills}"
            )

            skills_msg = (
                f"Sidecar Learner completed processing using DSPy CodeAct. "
                f"Summary: {result.suggested_skills[:100]}..."
            )
            return state.model_copy(
                update={
                    "journal": state.journal + journal_entry,
                    "messages": [AIMessage(content=skills_msg)],
                }
            )
        except Exception as e:
            logger.error(f"Skills agent failed: {e}")
            return state.model_copy(
                update={"journal": state.journal + f"\n[Skills] System error: {e}"}
            )


# Factory function for LangGraph
@type_check
async def skills_node(state: AgentState) -> AgentState:
    from controller.config.settings import settings
    session_id = state.session_id or settings.default_session_id
    ctx = SharedNodeContext.create(
        worker_url=settings.spec_001_api_url, session_id=session_id
    )
    node = SkillsNode(context=ctx)
    return await node(state)
