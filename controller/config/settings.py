from pydantic import Field, StrictStr
from pydantic_settings import BaseSettings, SettingsConfigDict


class Settings(BaseSettings):
    """Configuration settings for the Problemologist Controller."""

    # LLM Settings
    llm_model: StrictStr = Field(default="gpt-4o", alias="LLM_MODEL")
    openai_api_base: str | None = Field(default=None, alias="OPENAI_API_BASE")
    llm_temperature: float = Field(default=0.0, alias="LLM_TEMPERATURE")

    # Infrastructure Settings
    temporal_url: str = Field(default="temporal:7233", alias="TEMPORAL_URL")
    worker_url: str = Field(default="http://worker:8001", alias="WORKER_URL")
    postgres_url: str = Field(
        default="postgresql+asyncpg://postgres:postgres@localhost:5432/problemologist",
        alias="POSTGRES_URL",
    )

    model_config = SettingsConfigDict(
        env_file=".env",
        env_file_encoding="utf-8",
        extra="ignore",
        populate_by_name=True,
    )


settings = Settings()
