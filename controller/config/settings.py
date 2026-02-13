from pydantic import Field, StrictStr
from pydantic_settings import BaseSettings, SettingsConfigDict


class Settings(BaseSettings):
    """Configuration settings for the Problemologist Controller."""

    is_integration_test: bool = Field(default=False, alias="IS_INTEGRATION_TEST")

    # LLM Settings
    llm_model: StrictStr = Field(default="z-ai/glm-4.7-flash", alias="LLM_MODEL")
    openai_api_base: str | None = Field(default=None, alias="OPENAI_API_BASE")
    openai_api_key: str | None = Field(default=None, alias="OPENAI_API_KEY")
    llm_temperature: float = Field(default=0.0, alias="LLM_TEMPERATURE")

    # Agent Settings
    max_agent_turns: int = Field(default=60, alias="MAX_AGENT_TURNS")

    # Infrastructure Settings
    temporal_url: str = Field(default="temporal:7233", alias="TEMPORAL_URL")
    worker_url: str = Field(default="http://worker:8001", alias="WORKER_URL")
    postgres_url: str = Field(
        default="postgresql+asyncpg://postgres:postgres@localhost:15432/problemologist",
        alias="POSTGRES_URL",
    )

    # Observability Settings
    langfuse_host: str = Field(default="http://localhost:3000", alias="LANGFUSE_HOST")
    langfuse_public_key: str | None = Field(default=None, alias="LANGFUSE_PUBLIC_KEY")
    langfuse_secret_key: str | None = Field(default=None, alias="LANGFUSE_SECRET_KEY")

    model_config = SettingsConfigDict(
        env_file=".env",
        env_file_encoding="utf-8",
        extra="ignore",
        populate_by_name=True,
    )


settings = Settings()
