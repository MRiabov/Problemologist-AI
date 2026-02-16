import logging
import os
import uuid
from pathlib import Path

from git import GitCommandError, Repo
from langchain_core.messages import HumanMessage, SystemMessage
from langchain_core.runnables import RunnableConfig
from langchain_core.tools import tool
from langgraph.prebuilt import create_react_agent

from controller.observability.tracing import sync_asset
from shared.type_checking import type_check

from controller.agent.config import settings
from controller.agent.state import AgentState
from controller.agent.tools import get_engineer_tools

from .base import BaseNode, SharedNodeContext

logger = logging.getLogger(__name__)


@type_check
class SkillsNode(BaseNode):
    """
    Skills node: Analyzes the journal to suggest new skills.
    Refactored to use create_react_agent.
    """

    def __init__(
        self, context: SharedNodeContext, suggested_skills_dir: str = "suggested_skills"
    ):
        super().__init__(context)
        self.suggested_skills_dir = Path(suggested_skills_dir)
        self.suggested_skills_dir.mkdir(parents=True, exist_ok=True)
        self.repo_url = os.getenv("GIT_REPO_URL")
        self.pat = os.getenv("GIT_PAT")
        self.repo = None
        self._ensure_repo()

    def _get_auth_url(self) -> str | None:
        """Inject PAT into URL if available."""
        if not self.repo_url:
            return None
        if self.pat and "https://" in self.repo_url:
            return self.repo_url.replace("https://", f"https://{self.pat}@")
        return self.repo_url

    def _ensure_repo(self):
        """Ensure the skills directory is a git repository."""
        if not self.repo_url:
            logger.warning("GIT_REPO_URL not set, skipping git sync setup.")
            return

        try:
            if (self.suggested_skills_dir / ".git").exists():
                self.repo = Repo(self.suggested_skills_dir)
                logger.info("Loaded existing git repo for skills.")
            else:
                auth_url = self._get_auth_url()
                logger.info(f"Cloning skills repo from {self.repo_url}...")
                self.repo = Repo.clone_from(auth_url, self.suggested_skills_dir)
                logger.info("Skills repo cloned successfully.")

            with self.repo.config_writer() as git_config:
                if not git_config.has_option("user", "email"):
                    git_config.set_value("user", "email", "agent@problemologist.ai")
                    git_config.set_value("user", "name", "Problemologist Agent")

        except Exception as e:
            logger.error(f"Failed to initialize git repo: {e}")
            self.repo = None

    async def _resolve_conflicts(self):
        """Resolve git conflicts using LLM."""
        unmerged = self.repo.git.diff("--name-only", "--diff-filter=U").splitlines()

        for file_path in unmerged:
            full_path = self.suggested_skills_dir / file_path
            if not full_path.exists():
                continue

            content = full_path.read_text()
            prompt = self.ctx.pm.render("git_resolver", content=content)

            logger.info(f"Resolving conflict in {file_path}...")
            response = await self.ctx.llm.ainvoke([HumanMessage(content=prompt)])
            resolved_content = str(response.content)

            if "```" in resolved_content:
                lines = resolved_content.splitlines()
                if lines and lines[0].strip().startswith("```"):
                    lines = lines[1:]
                if lines and lines[-1].strip().startswith("```"):
                    lines = lines[:-1]
                resolved_content = "\n".join(lines)

            full_path.write_text(resolved_content)
            self.repo.git.add(file_path)

    async def _sync_git(self, commit_message: str):
        """Commit and push changes with rebase strategy."""
        if not self.repo:
            return

        try:
            self.repo.git.add(A=True)
            self.repo.index.commit(commit_message)

            try:
                self.repo.git.pull("--rebase")
            except GitCommandError:
                logger.warning("Rebase conflict during pull. Attempting to resolve...")
                try:
                    await self._resolve_conflicts()
                    self.repo.git.rebase("--continue")
                    logger.info("Conflict resolved and rebase continued.")
                except Exception as ex:
                    logger.error(f"Resolution failed: {ex}")
                    self.repo.git.rebase("--abort")
                    logger.error("Rebase aborted. Skills might be out of sync.")
                    return

            self.repo.git.push()
            logger.info("Skills synced to git successfully.")

        except GitCommandError as e:
            logger.error(f"Git sync failed: {e}")

    async def __call__(
        self, state: AgentState, config: RunnableConfig | None = None
    ) -> AgentState:
        """Execute the sidecar node logic using ReAct agent."""

        # Define the specialized tool for suggesting skills
        @tool
        async def save_suggested_skill(title: str, content: str) -> str:
            """
            Saves a new suggested skill to the repository.
            Args:
                title: A concise title for the skill (e.g., 'motor_initialization').
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
        tools = common_tools + [save_suggested_skill]

        agent = create_react_agent(self.ctx.llm, tools)

        prompt = self.ctx.pm.render("sidecar", journal=state.journal, task=state.task)
        # Add explicit direction to use the tool
        prompt += "\n\nIf you identify a valuable skill or pattern, use the `save_suggested_skill` tool to record it."

        messages = [SystemMessage(content=prompt)]

        try:
            # Observability
            callbacks = self._get_callbacks(name="skills", session_id=state.session_id)
            # Merge with existing callbacks if any
            if config and config.get("callbacks"):
                callbacks.extend(config.get("callbacks"))

            result = await agent.ainvoke(
                {"messages": messages}, config={"callbacks": callbacks}
            )
            final_messages = result["messages"]

            # Check if any skills were suggested by looking for tool calls
            suggested_count = 0
            for msg in final_messages:
                if hasattr(msg, "tool_calls") and msg.tool_calls:
                    for tc in msg.tool_calls:
                        if tc["name"] == "save_suggested_skill":
                            suggested_count += 1

            journal_entry = (
                f"\nSidecar Learner: Identified and recorded {suggested_count} new skills."
                if suggested_count > 0
                else "\nSidecar Learner: No new skills identified."
            )

            return state.model_copy(
                update={
                    "journal": state.journal + journal_entry,
                    "messages": state.messages + final_messages,
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
    session_id = state.session_id or settings.default_session_id
    ctx = SharedNodeContext.create(
        worker_url=settings.spec_001_api_url, session_id=session_id
    )
    node = SkillsNode(context=ctx)
    return await node(state)
