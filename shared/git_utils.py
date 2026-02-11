from pathlib import Path

import structlog
from git import Repo

logger = structlog.get_logger(__name__)


def init_workspace_repo(path: Path) -> Repo:
    """
    Initialize a git repository in the workspace if it doesn't verify exist.
    Configures default user.
    """
    git_dir = path / ".git"
    if not git_dir.exists():
        logger.info("initializing_git_repo", path=str(path))
        repo = Repo.init(path)
        with repo.config_writer() as git_config:
            git_config.set_value("user", "email", "agent@problemologist.ai")
            git_config.set_value("user", "name", "Agent Worker")

        # Create an initial commit to allow future commits to have a parent?
        # Or just let the first commit be the first one.
        # Let's create an empty commit so we have a HEAD.
        # But wait, hard to do empty commit with gitpython easily without index manipulation.
        # Let's just return the repo.
        return repo
    return Repo(path)


def commit_all(path: Path, message: str) -> str | None:
    """
    Add all files and commit. Returns commit hash or None if no changes.
    """
    try:
        repo = Repo(path)
        # Check if there are changes to commit
        if not repo.is_dirty(untracked_files=True):
            logger.info("git_no_changes_to_commit", path=str(path))
            return None

        logger.info("git_committing", path=str(path), message=message)
        repo.git.add(A=True)
        commit = repo.index.commit(message)
        return commit.hexsha
    except Exception as e:
        logger.error("git_commit_failed", error=str(e))
        return None
