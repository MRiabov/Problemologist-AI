from pathlib import Path
from pydantic_settings import BaseSettings, SettingsConfigDict
from pydantic import Field

class AppConfig(BaseSettings):
    """
    Agent configuration management using Pydantic Settings.
    """
    model_config = SettingsConfigDict(env_file=".env", env_file_encoding="utf-8", extra="ignore")

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

    def validate(self):
        """Validates that necessary API keys are present."""
        # Pydantic handles validation on instantiation.
        pass

# Create a global instance
Config = AppConfig()
