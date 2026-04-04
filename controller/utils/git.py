from pathlib import Path

import dspy
import structlog
from git import GitCommandError, Repo

logger = structlog.get_logger(__name__)


class GitResolver(dspy.Signature):
    """Resolve git merge conflicts while maintaining intended logic."""

    conflict_content = dspy.InputField(desc="File content with git conflict markers")
    resolved_content = dspy.OutputField(desc="File content with conflicts resolved")


class GitManager:
    """Legacy git helper for non-promotion flows.

    Skill publication now happens through the dedicated promotion arbiter; this
    helper remains only as a compatibility bridge for older callers.
    """

    def __init__(
        self,
        repo_path: Path,
        repo_url: str | None = None,
        pat: str | None = None,
        session_id: str | None = None,
    ):
        self.repo_path = repo_path
        self.repo_url = repo_url
        self.pat = pat
        self.session_id = session_id
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
            logger.warning("git_no_repo_url_provided", session_id=self.session_id)
            return

        try:
            if (self.repo_path / ".git").exists():
                self.repo = Repo(self.repo_path)
                logger.info("git_repo_loaded", session_id=self.session_id)
            else:
                auth_url = self._get_auth_url()
                logger.info(
                    "git_cloning_repo", url=self.repo_url, session_id=self.session_id
                )
                self.repo = Repo.clone_from(auth_url, self.repo_path)
                logger.info("git_repo_cloned", session_id=self.session_id)

            with self.repo.config_writer() as git_config:
                if not git_config.has_option("user", "email"):
                    git_config.set_value("user", "email", "agent@problemologist.ai")
                    git_config.set_value("user", "name", "Problemologist Agent")

        except Exception as e:
            logger.error("git_init_failed", error=str(e), session_id=self.session_id)
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
                logger.warning("git_rebase_conflict", session_id=self.session_id)
                if lm:
                    await self._resolve_conflicts(lm)
                    try:
                        self.repo.git.rebase("--continue")
                        logger.info("git_conflict_resolved", session_id=self.session_id)
                    except Exception as ex:
                        logger.error(
                            "git_rebase_continue_failed",
                            error=str(ex),
                            session_id=self.session_id,
                        )
                        self.repo.git.rebase("--abort")
                else:
                    logger.error(
                        "git_no_lm_for_conflict_resolution", session_id=self.session_id
                    )
                    self.repo.git.rebase("--abort")
                    return

            self.repo.git.push()
            logger.info("git_changes_synced", session_id=self.session_id)

        except GitCommandError as e:
            logger.error("git_sync_failed", error=str(e), session_id=self.session_id)

    async def _resolve_conflicts(self, lm: dspy.LM):
        """Resolve git conflicts using LLM."""
        unmerged = self.repo.git.diff("--name-only", "--diff-filter=U").splitlines()

        for file_path in unmerged:
            full_path = self.repo_path / file_path
            if not full_path.exists():
                continue

            content = full_path.read_text()

            # WP09: Use DSPy for conflict resolution
            logger.info(
                "git_resolving_conflict",
                file_path=file_path,
                session_id=self.session_id,
            )

            with dspy.settings.context(lm=lm):
                resolver = dspy.ReAct(
                    GitResolver, tools=[]
                )  # FIXME should have all engineering tools. Else how would it edit?
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
