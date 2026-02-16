import logging
from pathlib import Path
from typing import Any

from git import GitCommandError, Repo
from langchain_core.messages import HumanMessage

logger = logging.getLogger(__name__)


class GitManager:
    """Manages git operations for agent nodes."""

    def __init__(
        self, repo_path: Path, repo_url: str | None = None, pat: str | None = None
    ):
        self.repo_path = repo_path
        self.repo_url = repo_url
        self.pat = pat
        self.repo = None

    def _get_auth_url(self) -> str | None:
        """Inject PAT into URL if available."""
        if not self.repo_url:
            return None
        if self.pat and "https://" in self.repo_url:
            return self.repo_url.replace("https://", f"https://{self.pat}@")
        return self.repo_url

    def ensure_repo(self):
        """Ensure the directory is a git repository and synced."""
        if not self.repo_url:
            logger.warning("No repo URL provided, skipping git setup.")
            return

        try:
            if (self.repo_path / ".git").exists():
                self.repo = Repo(self.repo_path)
                logger.info("Loaded existing git repo.")
            else:
                auth_url = self._get_auth_url()
                logger.info(f"Cloning repo from {self.repo_url}...")
                self.repo = Repo.clone_from(auth_url, self.repo_path)
                logger.info("Repo cloned successfully.")

            with self.repo.config_writer() as git_config:
                if not git_config.has_option("user", "email"):
                    git_config.set_value("user", "email", "agent@problemologist.ai")
                    git_config.set_value("user", "name", "Problemologist Agent")

        except Exception as e:
            logger.error(f"Failed to initialize git repo: {e}")
            self.repo = None

    async def sync_changes(self, commit_message: str, llm: Any = None, pm: Any = None):
        """Commit and push changes with rebase strategy and conflict resolution."""
        if not self.repo:
            return

        try:
            self.repo.git.add(A=True)
            self.repo.index.commit(commit_message)

            try:
                self.repo.git.pull("--rebase")
            except GitCommandError:
                logger.warning("Rebase conflict detected. Attempting resolution...")
                if llm and pm:
                    await self._resolve_conflicts(llm, pm)
                    try:
                        self.repo.git.rebase("--continue")
                        logger.info("Conflict resolved via LLM.")
                    except Exception as ex:
                        logger.error(f"Rebase continue failed: {ex}")
                        self.repo.git.rebase("--abort")
                else:
                    logger.error(
                        "No LLM provided for conflict resolution. Aborting rebase."
                    )
                    self.repo.git.rebase("--abort")
                    return

            self.repo.git.push()
            logger.info("Changes synced successfully.")

        except GitCommandError as e:
            logger.error(f"Git sync failed: {e}")

    async def _resolve_conflicts(self, llm: Any, pm: Any):
        """Resolve git conflicts using LLM."""
        unmerged = self.repo.git.diff("--name-only", "--diff-filter=U").splitlines()

        for file_path in unmerged:
            full_path = self.repo_path / file_path
            if not full_path.exists():
                continue

            content = full_path.read_text()
            prompt = pm.render("git_resolver", content=content)

            logger.info(f"Resolving conflict in {file_path}...")
            response = await llm.ainvoke([HumanMessage(content=prompt)])
            resolved_content = str(response.content)

            # Basic markdown code block stripping
            if "```" in resolved_content:
                lines = resolved_content.splitlines()
                if lines and lines[0].strip().startswith("```"):
                    lines = lines[1:]
                if lines and lines[-1].strip().startswith("```"):
                    lines = lines[:-1]
                resolved_content = "\n".join(lines)

            full_path.write_text(resolved_content)
            self.repo.git.add(file_path)
