import structlog
from git import Repo

from worker.config import settings

logger = structlog.get_logger(__name__)


def _get_auth_url(repo_url: str, pat: str | None) -> str:
    """Inject PAT into URL if available."""
    if pat and "https://" in repo_url:
        return repo_url.replace("https://", f"https://{pat}@")
    return repo_url


def sync_skills():
    """Clone or pull skills from git repo."""
    repo_url = settings.git_repo_url
    pat = settings.git_pat
    skills_dir = settings.skills_dir

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
