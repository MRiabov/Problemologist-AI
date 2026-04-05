import contextlib
from pathlib import Path

import structlog
from git import Repo

from shared.git_utils import commit_submission_attempt as _commit_submission_attempt
from shared.skills import load_skills_projection_config

logger = structlog.get_logger(__name__)
ROOT = Path(__file__).resolve().parents[2]
DEFAULT_SKILL_REPO_ROOT = ROOT / ".agents" / "skills"
DEFAULT_SKILL_PROJECTION_CONFIG = ROOT / "config" / "skills_config.yaml"


def init_workspace_repo(path: Path) -> Repo:
    """
    Initialize a git repository in the workspace if it doesn't already exist.
    Configures a default user and creates an initial commit when there is
    content to snapshot.
    """
    git_dir = path / ".git"
    if not git_dir.exists():
        logger.info("initializing_git_repo", path=str(path))
        repo = Repo.init(path)
        with repo.config_writer() as git_config:
            git_config.set_value("user", "email", "agent@problemologist.ai")
            git_config.set_value("user", "name", "Agent Worker")
        commit_all(path, "Initial commit")
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
        logger.warning("git_commit_failed", error=str(e), session_id="system")
        return None


def commit_submission_attempt(path: Path, message: str) -> str | None:
    """
    Snapshot a submission attempt, even when the workspace is clean.
    """
    return _commit_submission_attempt(path, message)


def repo_head_state(path: Path) -> dict[str, str | None]:
    """Return the current branch and HEAD commit for a git repository."""
    try:
        repo = Repo(path)
        branch = "HEAD"
        if not repo.head.is_detached:
            branch = repo.active_branch.name
        head_commit = None
        with contextlib.suppress(Exception):
            head_commit = repo.head.commit.hexsha
        return {
            "branch": branch,
            "head_commit": head_commit,
        }
    except Exception as e:
        logger.warning("repo_head_state_failed", error=str(e), session_id="system")
        return {"branch": None, "head_commit": None}


def repo_is_clean(path: Path) -> bool:
    """Return whether the repository has no staged, unstaged, or untracked changes."""
    try:
        repo = Repo(path)
        return not repo.is_dirty(untracked_files=True) and not has_merge_conflicts(path)
    except Exception as e:
        logger.warning("repo_is_clean_failed", error=str(e), session_id="system")
        return False


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
        logger.warning("merge_conflict_check_failed", error=str(e), session_id="system")
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
        logger.warning("get_conflicted_files_failed", error=str(e), session_id="system")
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
        logger.warning(
            "resolve_conflict_ours_failed",
            file=file_path,
            error=str(e),
            session_id="system",
        )
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
        logger.warning(
            "resolve_conflict_theirs_failed",
            file=file_path,
            error=str(e),
            session_id="system",
        )
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
        logger.warning("abort_merge_failed", error=str(e), session_id="system")
        return False


def copy_tree(
    source_root: Path,
    destination_root: Path,
    *,
    exclude_rel_paths: set[str] | None = None,
) -> list[str]:
    """Copy a tree into another tree while preserving relative paths."""
    source_root = source_root.expanduser().resolve()
    destination_root = destination_root.expanduser().resolve()
    exclude_rel_paths = {
        Path(path).as_posix().lstrip("/") for path in (exclude_rel_paths or set())
    }

    copied: list[str] = []
    for src_path in sorted(p for p in source_root.rglob("*") if p.is_file()):
        rel_path = src_path.relative_to(source_root).as_posix()
        if any(
            rel_path == excluded or rel_path.startswith(f"{excluded}/")
            for excluded in exclude_rel_paths
        ):
            continue
        if any(part == ".git" for part in src_path.parts):
            continue
        if src_path.suffix in {".pyc", ".pyo"}:
            continue
        dst_path = destination_root / rel_path
        dst_path.parent.mkdir(parents=True, exist_ok=True)
        dst_path.write_bytes(src_path.read_bytes())
        copied.append(rel_path)
    return copied


def complete_merge(
    path: Path, message: str | None = None, session_id: str | None = None
) -> str | None:
    """
    Complete a merge after all conflicts are resolved.

    Args:
        path: Repository root path
        message: Optional commit message override
        session_id: Optional session ID for logging

    Returns:
        Commit hash if successful, None otherwise
    """
    try:
        repo = Repo(path)

        # Check if there are still conflicts
        if has_merge_conflicts(path):
            logger.error(
                "cannot_complete_merge_conflicts_remain", session_id=session_id
            )
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
        logger.warning("complete_merge_failed", error=str(e), session_id=session_id)
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
        logger.warning("get_repo_status_failed", error=str(e), session_id="system")
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


def _projected_skill_names(
    source_root: Path, *, config_path: Path | None = None
) -> set[str]:
    policy = load_skills_projection_config(config_path=config_path)
    names: set[str] = set()
    for skill_dir in sorted(source_root.iterdir(), key=lambda path: path.name):
        if not skill_dir.is_dir():
            continue
        if not (skill_dir / "SKILL.md").is_file():
            continue
        if policy.get(skill_dir.name, {}).get("is_for_worker_agents", False):
            names.add(skill_dir.name)
    return names


def _projected_rel_paths(
    source_root: Path, *, config_path: Path | None = None
) -> set[str]:
    projected_skill_names = _projected_skill_names(source_root, config_path=config_path)
    projected_paths: set[str] = set()
    for src_path in sorted(p for p in source_root.rglob("*") if p.is_file()):
        if any(part == ".git" for part in src_path.parts) or src_path.suffix in {
            ".pyc",
            ".pyo",
        }:
            continue
        rel_path = src_path.relative_to(source_root).as_posix()
        parts = Path(rel_path).parts
        if not parts:
            continue
        if len(parts) == 1:
            projected_paths.add(rel_path)
            continue
        if parts[0] in projected_skill_names:
            projected_paths.add(rel_path)
    return projected_paths


def _sync_projected_tree(
    source_root: Path,
    destination_root: Path,
    *,
    config_path: Path | None = None,
) -> list[str]:
    projected_rel_paths = _projected_rel_paths(source_root, config_path=config_path)
    destination_root.mkdir(parents=True, exist_ok=True)

    existing_rel_paths = {
        path.relative_to(destination_root).as_posix()
        for path in destination_root.rglob("*")
        if path.is_file() and ".git" not in path.parts
    }
    for rel_path in sorted(existing_rel_paths - projected_rel_paths):
        (destination_root / rel_path).unlink()

    copied: list[str] = []
    for rel_path in sorted(projected_rel_paths):
        src_path = source_root / rel_path
        dst_path = destination_root / rel_path
        dst_path.parent.mkdir(parents=True, exist_ok=True)
        if not src_path.exists():
            continue
        dst_path.write_bytes(src_path.read_bytes())
        copied.append(rel_path)
    return copied


def sync_skills(
    repo_url: str | None,
    pat: str | None,
    skills_dir: Path,
    session_id: str | None = None,
    source_root: Path | None = None,
):
    """Project worker-visible skills from the canonical repo into the mount."""

    canonical_root = (source_root or DEFAULT_SKILL_REPO_ROOT).expanduser().resolve()
    if canonical_root.exists():
        try:
            copied = _sync_projected_tree(
                canonical_root,
                skills_dir.expanduser().resolve(),
                config_path=DEFAULT_SKILL_PROJECTION_CONFIG,
            )
            logger.info(
                "skills_projected_successfully",
                source=str(canonical_root),
                destination=str(skills_dir),
                copied=len(copied),
            )
            return
        except Exception as e:
            logger.warning(
                "failed_to_project_skills",
                error=str(e),
                session_id=session_id,
                source=str(canonical_root),
            )

    if not repo_url:
        logger.info(
            "canonical_skill_repo_missing_and_no_remote_source",
        )
        return

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
        logger.error("failed_to_sync_skills", error=str(e), session_id=session_id)
