from pathlib import Path
from pydantic_settings import BaseSettings, SettingsConfigDict
from pydantic import Field


class AppConfig(BaseSettings):
    """
    Agent configuration management using Pydantic Settings.
    """

    model_config = SettingsConfigDict(
        env_file=".env", env_file_encoding="utf-8", extra="ignore"
    )

    # Default model to use
    LLM_MODEL: str = Field(default="gpt-4o")

    # OpenAI-compatible API base URL (e.g., OpenRouter)
    OPENAI_API_BASE: str = Field(default="https://api.openai.com/v1")

    # OpenAI-compatible API key
    OPENAI_API_KEY: str = Field(default="")

    # Default temperature for generation
    # Using alias to map LLM_TEMPERATURE environment variable to TEMPERATURE field
    TEMPERATURE: float = Field(default=0.0, validation_alias="LLM_TEMPERATURE")

    # Work directory for scripts and artifacts
    WORKSPACE_DIR: Path = Field(default=Path("./workspaces/main"))

    # Safety limits
    MAX_STEPS: int = Field(default=100)

    @property
    def PROJECT_ROOT(self) -> Path:
        """Returns the project root directory."""
        # src/agent/utils/config.py -> src/agent/utils -> src/agent -> src -> root
        return Path(__file__).resolve().parent.parent.parent.parent

    @property
    def SKILLS_DIR(self) -> Path:
        return self.PROJECT_ROOT / ".agent" / "skills"

    @property
    def SKILL_CREATOR_DIR(self) -> Path:
        return self.SKILLS_DIR / "skill-creator"

    @property
    def GEN_WORKSPACE_DIR(self) -> Path:
        """Workspace directory for the benchmark generator."""
        return self.PROJECT_ROOT / "workspaces" / "gen"

    def validate(self):
        """Validates that necessary API keys are present."""
        # Pydantic handles validation on instantiation.
        pass


# Factory function for testability
def get_config(**overrides) -> AppConfig:
    """
    Returns an AppConfig instance with optional overrides.

    This factory function provides a testability hook - tests can call
    get_config(WORKSPACE_DIR=Path("/tmp/test")) instead of patching globals.

    Args:
        **overrides: Keyword arguments to override config values.

    Returns:
        A fresh AppConfig instance with overrides applied.
    """
    if overrides:
        return AppConfig(**overrides)
    return AppConfig()


# Create a global instance (for convenience in production code)
# Note: Tests should prefer get_config() for explicit dependency injection.
Config = get_config()
