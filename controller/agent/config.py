from pydantic_settings import BaseSettings, SettingsConfigDict

from controller.config.settings import settings as global_settings


class AgentSettings(BaseSettings):
    """Configuration for the Engineer Agent."""

    llm_model: str = global_settings.llm_model
    openai_api_key: str | None = None
    openai_api_base: str | None = None  # For OpenRouter or custom endpoints
    anthropic_api_key: str | None = None

    # URL for the Spec 001 Worker API
    spec_001_api_url: str = "http://worker:8001"

    # DB connection for checkpointing (if using PostgresSaver in future)
    db_connection_string: str | None = None

    # Default session/thread ID for the agent
    default_session_id: str = "default-agent-session"

    model_config = SettingsConfigDict(
        env_file=".env", env_file_encoding="utf-8", extra="ignore"
    )


settings = AgentSettings()
