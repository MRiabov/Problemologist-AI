import logging
from pathlib import Path

import dspy
from git import GitCommandError, Repo

logger = logging.getLogger(__name__)


class GitResolver(dspy.Signature):
    """Resolve git merge conflicts while maintaining intended logic."""

    conflict_content = dspy.InputField(desc="File content with git conflict markers")
    resolved_content = dspy.OutputField(desc="File content with conflicts resolved")


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

    async def sync_changes(self, commit_message: str, lm: dspy.LM = None):
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
                if lm:
                    await self._resolve_conflicts(lm)
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

    async def _resolve_conflicts(self, lm: dspy.LM):
        """Resolve git conflicts using LLM."""
        unmerged = self.repo.git.diff("--name-only", "--diff-filter=U").splitlines()

        def read_local_file(path: str) -> str:
            return Path(path).read_text()

        def write_local_file(path: str, content: str) -> str:
            Path(path).write_text(content)
            return f"Wrote to {path}"

        for file_path in unmerged:
            full_path = self.repo_path / file_path
            if not full_path.exists():
                continue

            content = full_path.read_text()

            # WP09: Use DSPy for conflict resolution
            logger.info(f"Resolving conflict in {file_path}...")

            with dspy.settings.context(lm=lm):
                resolver = dspy.ReAct(
                    GitResolver, tools=[read_local_file, write_local_file]
                )
                prediction = resolver(conflict_content=content)
                resolved_content = prediction.resolved_content

            # Basic markdown code block stripping (DSPy might still return blocks)
            if "```" in resolved_content:
                lines = resolved_content.splitlines()
                if lines and lines[0].strip().startswith("```"):
                    lines = lines[1:]
                if lines and lines[-1].strip().startswith("```"):
                    lines = lines[:-1]
                resolved_content = "\n".join(lines)

            full_path.write_text(resolved_content)
            self.repo.git.add(file_path)
