from pathlib import Path

import structlog
import yaml
from pydantic_settings import BaseSettings, SettingsConfigDict

from controller.config.settings import settings as global_settings
from shared.workers.filesystem.policy import FilesystemConfig

logger = structlog.get_logger(__name__)


def _resolve_llm_max_tokens_from_agents_config(default: int = 16384) -> int:
    potential_paths = [
        Path("config/agents_config.yaml"),
        Path(__file__).parents[2] / "config" / "agents_config.yaml",
        Path("/app/config/agents_config.yaml"),
    ]
    config_path = next((p for p in potential_paths if p.exists()), None)

    if config_path is None:
        return default

    try:
        with config_path.open("r", encoding="utf-8") as f:
            raw_data = yaml.safe_load(f) or {}
        config = FilesystemConfig(**raw_data)
        return config.llm.max_reasoning_tokens
    except Exception as exc:
        logger.warning(
            "failed_to_load_agents_llm_config",
            config_path=str(config_path),
            error=str(exc),
        )
        return default


def _resolve_context_compaction_threshold_tokens_from_agents_config(
    default: int = 225000,
) -> int:
    potential_paths = [
        Path("config/agents_config.yaml"),
        Path(__file__).parents[2] / "config" / "agents_config.yaml",
        Path("/app/config/agents_config.yaml"),
    ]
    config_path = next((p for p in potential_paths if p.exists()), None)

    if config_path is None:
        return default

    try:
        with config_path.open("r", encoding="utf-8") as f:
            raw_data = yaml.safe_load(f) or {}
        config = FilesystemConfig(**raw_data)
        return config.llm.context_compaction_threshold_tokens
    except Exception as exc:
        logger.warning(
            "failed_to_load_agents_context_compaction_tokens_config",
            config_path=str(config_path),
            error=str(exc),
        )
        return default


class AgentSettings(BaseSettings):
    """Configuration for the Engineer Agent."""

    llm_model: str = global_settings.llm_model
    openai_api_key: str | None = None
    openai_api_base: str | None = None  # For OpenRouter or custom endpoints
    openrouter_api_key: str | None = global_settings.openrouter_api_key
    anthropic_api_key: str | None = None
    llm_timeout_seconds: int = 300
    llm_max_tokens: int = _resolve_llm_max_tokens_from_agents_config()
    context_compaction_threshold_tokens: int = (
        _resolve_context_compaction_threshold_tokens_from_agents_config()
    )
    dspy_program_timeout_seconds: int = 300
    react_max_iters: int = 8
    react_planner_max_iters: int = 6
    require_reasoning_traces: bool = global_settings.require_reasoning_traces

    # URL for the Spec 001 Worker API
    spec_001_api_url: str = global_settings.worker_light_url
    worker_heavy_url: str | None = global_settings.worker_heavy_url
    is_integration_test: bool = global_settings.is_integration_test

    # DB connection for checkpointing (if using PostgresSaver in future)
    db_connection_string: str | None = None

    # Default session/thread ID for the agent
    default_session_id: str = "00000000-0000-0000-0000-000000000000"

    model_config = SettingsConfigDict(
        env_file=".env", env_file_encoding="utf-8", extra="ignore"
    )


settings = AgentSettings()
