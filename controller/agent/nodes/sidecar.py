import logging
import os
from pathlib import Path

from git import GitCommandError, Repo
from langchain_core.messages import HumanMessage
from langchain_openai import ChatOpenAI

from shared.type_checking import type_check

from ..prompt_manager import PromptManager
from ..state import AgentState

logger = logging.getLogger(__name__)


@type_check
class SidecarNode:
    """
    Sidecar Learner node: Analyzes the journal to suggest new skills.
    Includes Git synchronization for skill persistence.
    """

    def __init__(self, suggested_skills_dir: str = "suggested_skills"):
        self.pm = PromptManager()
        self.llm = ChatOpenAI(model="gpt-4o", temperature=0)
        self.suggested_skills_dir = Path(suggested_skills_dir)
        self.suggested_skills_dir.mkdir(parents=True, exist_ok=True)
        self.repo_url = os.getenv("GIT_REPO_URL")
        self.pat = os.getenv("GIT_PAT")
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

            # Configure user if not present (needed for commits)
            with self.repo.config_writer() as git_config:
                if not git_config.has_option("user", "email"):
                    git_config.set_value("user", "email", "agent@problemologist.ai")
                    git_config.set_value("user", "name", "Problemologist Agent")

        except Exception as e:
            logger.error(f"Failed to initialize git repo: {e}")
            self.repo = None

    def _sync_git(self, commit_message: str):
        """Commit and push changes with rebase strategy."""
        if not self.repo:
            return

        try:
            self.repo.git.add(A=True)
            self.repo.index.commit(commit_message)

            # Pull with rebase to handle concurrent updates
            try:
                self.repo.git.pull("--rebase")
            except GitCommandError:
                logger.warning("Rebase conflict during pull. Attempting to resolve...")
                # Simple resolution: ours (agent's new skill) wins for now,
                # or we could just abort.
                # Ideally, we'd use the LLM to merge, but for now let's try to abort rebase and just push?
                # No, if rebase fails, we are in a detached head or intermediate state.
                self.repo.git.rebase("--abort")
                logger.error("Rebase failed and aborted. Skills might be out of sync.")
                # TODO unhandled! should use the llm to resolve it. it can.
                # I think the sidecar has in fact the tools, so it can.
                return

            self.repo.git.push()
            logger.info("Skills synced to git successfully.")

        except GitCommandError as e:
            logger.error(f"Git sync failed: {e}")

    async def __call__(self, state: AgentState) -> AgentState:
        """Execute the sidecar node logic."""
        # T021: Parse Journal for patterns
        prompt = self.pm.render("sidecar", journal=state.journal, task=state.task)

        response = await self.llm.ainvoke([HumanMessage(content=prompt)])
        content = str(response.content)

        # T022: Skill extraction logic
        # Expecting format:
        # SKILL: <Title>
        # CONTENT: <Markdown content>

        suggested_skill = None
        if "SKILL:" in content and "CONTENT:" in content:
            parts = content.split("CONTENT:")
            title = parts[0].replace("SKILL:", "").strip().lower().replace(" ", "_")
            skill_content = parts[1].strip()

            file_path = self.suggested_skills_dir / f"{title}.md"
            with open(file_path, "w") as f:
                f.write(skill_content)
            suggested_skill = title
            logger.info(f"Suggested new skill: {title}")

            # Sync to Git
            self._sync_git(f"Add skill: {title}")

        journal_entry = f"\nSidecar Learner: {'Suggested skill ' + suggested_skill if suggested_skill else 'No new skills identified.'}"

        return state.model_copy(update={"journal": state.journal + journal_entry})


# Factory function for LangGraph
@type_check
async def sidecar_node(state: AgentState) -> AgentState:
    node = SidecarNode()
    return await node(state)
