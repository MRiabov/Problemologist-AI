from pydantic_settings import BaseSettings, SettingsConfigDict

from controller.config.settings import settings as global_settings
from shared.agents.config import load_agents_config


def _load_agents_llm_config():
    return load_agents_config().llm


class AgentSettings(BaseSettings):
    """Configuration for the Engineer Agent."""

    llm_model: str = global_settings.llm_model
    llm_cli_command_template: str | None = global_settings.llm_cli_command_template
    llm_cli_timeout_seconds: int = global_settings.llm_cli_timeout_seconds
    openai_api_key: str | None = None
    openai_api_base: str | None = None  # For OpenRouter or custom endpoints
    openrouter_api_key: str | None = global_settings.openrouter_api_key
    anthropic_api_key: str | None = None
    llm_timeout_seconds: int = 300
    native_tool_completion_timeout_seconds: int = 60
    llm_max_tokens: int = _load_agents_llm_config().max_reasoning_tokens
    context_compaction_threshold_tokens: int = (
        _load_agents_llm_config().context_compaction_threshold_tokens
    )
    dspy_program_timeout_seconds: int = 300
    dspy_program_max_retries: int = 2
    require_reasoning_traces: bool = global_settings.require_reasoning_traces

    # URL for the Worker Light API
    worker_light_url: str = global_settings.worker_light_url
    worker_heavy_url: str | None = global_settings.worker_heavy_url
    is_integration_test: bool = global_settings.is_integration_test
    integration_use_real_llm: bool = global_settings.integration_use_real_llm

    # DB connection for checkpointing (if using PostgresSaver in future)
    db_connection_string: str | None = None

    # Default session/thread ID for the agent
    default_session_id: str = "00000000-0000-0000-0000-000000000000"

    model_config = SettingsConfigDict(
        env_file=".env", env_file_encoding="utf-8", extra="ignore"
    )


settings = AgentSettings()
