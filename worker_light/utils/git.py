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


def has_merge_conflicts(path: Path) -> bool:
    """
    Check if the repository has unresolved merge conflicts.

    Returns:
        True if there are merge conflicts, False otherwise
    """
    try:
        repo = Repo(path)
        # Check for conflict markers in index
        unmerged = repo.index.unmerged_blobs()
        return len(unmerged) > 0

    except Exception as e:
        logger.error("merge_conflict_check_failed", error=str(e))
        return False


def get_conflicted_files(path: Path) -> list[str]:
    """
    Get list of files with merge conflicts.

    Returns:
        List of relative paths to conflicted files
    """
    try:
        repo = Repo(path)
        unmerged = repo.index.unmerged_blobs()
        # unmerged_blobs() returns dict of path -> list of (stage, blob) tuples
        return list(unmerged.keys())
    except Exception as e:
        logger.error("get_conflicted_files_failed", error=str(e))
        return []


def resolve_conflict_ours(path: Path, file_path: str) -> bool:
    """
    Resolve a merge conflict by keeping 'ours' (current branch) version.

    Args:
        path: Repository root path
        file_path: Relative path to conflicted file

    Returns:
        True if successfully resolved, False otherwise
    """
    try:
        repo = Repo(path)
        # Checkout ours version and stage it
        repo.git.checkout("--ours", file_path)
        repo.git.add(file_path)
        logger.info("resolved_conflict_ours", file=file_path)
        return True
    except Exception as e:
        logger.error("resolve_conflict_ours_failed", file=file_path, error=str(e))
        return False


def resolve_conflict_theirs(path: Path, file_path: str) -> bool:
    """
    Resolve a merge conflict by keeping 'theirs' (incoming) version.

    Args:
        path: Repository root path
        file_path: Relative path to conflicted file

    Returns:
        True if successfully resolved, False otherwise
    """
    try:
        repo = Repo(path)
        # Checkout theirs version and stage it
        repo.git.checkout("--theirs", file_path)
        repo.git.add(file_path)
        logger.info("resolved_conflict_theirs", file=file_path)
        return True
    except Exception as e:
        logger.error("resolve_conflict_theirs_failed", file=file_path, error=str(e))
        return False


def abort_merge(path: Path) -> bool:
    """
    Abort an ongoing merge operation.

    Returns:
        True if merge was aborted, False otherwise
    """
    try:
        repo = Repo(path)
        repo.git.merge("--abort")
        logger.info("merge_aborted", path=str(path))
        return True
    except Exception as e:
        logger.error("abort_merge_failed", error=str(e))
        return False


def complete_merge(path: Path, message: str | None = None) -> str | None:
    """
    Complete a merge after all conflicts are resolved.

    Args:
        path: Repository root path
        message: Optional commit message override

    Returns:
        Commit hash if successful, None otherwise
    """
    try:
        repo = Repo(path)

        # Check if there are still conflicts
        if has_merge_conflicts(path):
            logger.error("cannot_complete_merge_conflicts_remain")
            return None

        # If we're in a merge state, conclude it
        merge_head = path / ".git" / "MERGE_HEAD"
        if merge_head.exists():
            if message is None:
                message = "Merge completed by agent"
            # Use git command to commit, which handles MERGE_HEAD cleanup
            repo.git.commit("-m", message)
            commit_hash = repo.head.commit.hexsha
            logger.info("merge_completed", commit=commit_hash)
            return commit_hash

        logger.info("no_merge_in_progress")
        return None

    except Exception as e:
        logger.error("complete_merge_failed", error=str(e))
        return None


def get_repo_status(path: Path) -> dict:
    """
    Get current repository status.

    Returns:
        Dictionary with branch, is_dirty, is_merging, conflicts.
    """
    try:
        repo = Repo(path)
        is_merging = (path / ".git" / "MERGE_HEAD").exists()
        conflicts = get_conflicted_files(path)

        branch = "HEAD"
        if not repo.head.is_detached:
            branch = repo.active_branch.name

        return {
            "branch": branch,
            "is_dirty": repo.is_dirty(untracked_files=True),
            "is_merging": is_merging,
            "conflicts": conflicts,
        }
    except Exception as e:
        logger.error("get_repo_status_failed", error=str(e))
        return {
            "branch": "unknown",
            "is_dirty": False,
            "is_merging": False,
            "conflicts": [],
            "error": str(e),
        }


def _get_auth_url(repo_url: str, pat: str | None) -> str:
    """Inject PAT into URL if available."""
    if pat and "https://" in repo_url:
        return repo_url.replace("https://", f"https://{pat}@")
    return repo_url


def sync_skills(repo_url: str | None, pat: str | None, skills_dir: Path):
    """Clone or pull skills from git repo."""
    if not repo_url:
        logger.info("GIT_REPO_URL not set, skipping skills sync.")
        return

    # Ensure directory exists
    skills_dir.mkdir(parents=True, exist_ok=True)

    auth_url = _get_auth_url(repo_url, pat)

    try:
        if (skills_dir / ".git").exists():
            logger.info("pulling_latest_skills", path=str(skills_dir))
            repo = Repo(skills_dir)
            repo.remotes.origin.pull()
            logger.info("skills_pulled_successfully")
        else:
            logger.info("cloning_skills", url=repo_url, path=str(skills_dir))
            if any(skills_dir.iterdir()):
                repo = Repo.init(skills_dir)
                if "origin" not in repo.remotes:
                    repo.create_remote("origin", auth_url)
                repo.remotes.origin.pull("main")
            else:
                Repo.clone_from(auth_url, skills_dir)
            logger.info("skills_cloned_successfully")
    except Exception as e:
        logger.error("failed_to_sync_skills", error=str(e))
