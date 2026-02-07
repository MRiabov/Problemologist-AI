import os
import structlog
from pathlib import Path
from git import Repo

logger = structlog.get_logger(__name__)


def _get_auth_url(repo_url: str, pat: str | None) -> str:
    """Inject PAT into URL if available."""
    if pat and "https://" in repo_url:
        return repo_url.replace("https://", f"https://{pat}@")
    return repo_url


def sync_skills():
    """Clone or pull skills from git repo."""
    repo_url = os.getenv("GIT_REPO_URL")
    pat = os.getenv("GIT_PAT")

    if not repo_url:
        logger.info("GIT_REPO_URL not set, skipping skills sync.")
        return

    # Download skills to a separate directory from the code
    skills_dir = Path("/app/skills")
    skills_dir.mkdir(parents=True, exist_ok=True)

    auth_url = _get_auth_url(repo_url, pat)

    try:
        if (skills_dir / ".git").exists():
            logger.info("Pulling latest skills...")
            repo = Repo(skills_dir)
            repo.remotes.origin.pull()
            logger.info("Skills pulled successfully.")
        else:
            logger.info(f"Cloning skills from {repo_url}...")
            if any(skills_dir.iterdir()):
                repo = Repo.init(skills_dir)
                if "origin" not in repo.remotes:
                    repo.create_remote("origin", auth_url)
                repo.remotes.origin.pull("main")
            else:
                Repo.clone_from(auth_url, skills_dir)
            logger.info("Skills cloned successfully.")
    except Exception as e:
        logger.error(f"Failed to sync skills: {e}")
