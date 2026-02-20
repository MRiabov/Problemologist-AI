import tempfile
from pathlib import Path

from pydantic import Field
from pydantic_settings import BaseSettings, SettingsConfigDict


class WorkerSettings(BaseSettings):
    """Configuration settings for the Problemologist Worker (Light)."""

    is_integration_test: bool = Field(default=False, alias="IS_INTEGRATION_TEST")

    # Skills Configuration
    skills_dir_override: str | None = Field(default=None, alias="SKILLS_DIR")
    # Fallback for local development or custom environments
    skills_dir_default: str | None = Field(default=None, alias="SKILLS_DIR_DEFAULT")

    git_repo_url: str | None = Field(default=None, alias="GIT_REPO_URL")
    git_pat: str | None = Field(default=None, alias="GIT_PAT")

    sessions_dir: str | None = Field(default=None, alias="WORKER_SESSIONS_DIR")

    @property
    def skills_dir(self) -> Path:
        """Get the effective skills directory."""
        if self.skills_dir_override:
            return Path(self.skills_dir_override)

        if self.is_integration_test:
            return Path(tempfile.gettempdir()) / "problemologist_skills"

        if self.skills_dir_default:
            return Path(self.skills_dir_default)

        # Fallback for local development
        app_skills = Path("/app/skills")
        try:
            if app_skills.exists():
                return app_skills
            return Path("./skills").absolute()
        except Exception:
            return Path("./skills").absolute()

    model_config = SettingsConfigDict(
        env_file=".env",
        env_file_encoding="utf-8",
        extra="ignore",
        populate_by_name=True,
    )


settings = WorkerSettings()
