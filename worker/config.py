import tempfile
from pathlib import Path
from pydantic import Field
from pydantic_settings import BaseSettings, SettingsConfigDict


class WorkerSettings(BaseSettings):
    """Configuration settings for the Problemologist Worker."""

    is_integration_test: bool = Field(default=False, alias="IS_INTEGRATION_TEST")

    # Skills Configuration
    # We allow override via SKILLS_DIR env var.
    # If not set, we default based on the environment (integration test vs production/docker).
    skills_dir_override: str | None = Field(default=None, alias="SKILLS_DIR")

    git_repo_url: str | None = Field(default=None, alias="GIT_REPO_URL")
    git_pat: str | None = Field(default=None, alias="GIT_PAT")

    @property
    def skills_dir(self) -> Path:
        """Get the effective skills directory."""
        if self.skills_dir_override:
            return Path(self.skills_dir_override)

        if self.is_integration_test:
            # Use a stable temp dir for integration tests
            return Path(tempfile.gettempdir()) / "problemologist_skills"

        return Path("/app/skills")

    model_config = SettingsConfigDict(
        env_file=".env",
        env_file_encoding="utf-8",
        extra="ignore",
        populate_by_name=True,
    )


settings = WorkerSettings()
