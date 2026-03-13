from typing import Any, Literal

import dspy
from pydantic import BaseModel
from pydantic_settings import BaseSettings, SettingsConfigDict

from controller.config.settings import settings as global_settings
from shared.agents.config import load_agents_config


def _load_agents_llm_config():
    return load_agents_config().llm


class LiteLLMRequestConfig(BaseModel):
    """Resolved LiteLLM request settings for one model invocation."""

    model: str
    api_key: str | None
    api_base: str | None
    provider: Literal["openrouter", "openai_compatible", "gemini"]


_DIRECT_PROVIDER_PREFIXES = {
    "openai",
    "openrouter",
    "anthropic",
    "azure",
    "gemini",
}

_GOOGLE_AI_STUDIO_V1BETA_BASE = "https://generativelanguage.googleapis.com/v1beta"


class AgentSettings(BaseSettings):
    """Configuration for the Engineer Agent."""

    llm_model: str = global_settings.llm_model
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

    # DB connection for checkpointing (if using PostgresSaver in future)
    db_connection_string: str | None = None

    # Default session/thread ID for the agent
    default_session_id: str = "00000000-0000-0000-0000-000000000000"

    model_config = SettingsConfigDict(
        env_file=".env", env_file_encoding="utf-8", extra="ignore"
    )

    def uses_openrouter(self, model_name: str | None = None) -> bool:
        api_base = self.openai_api_base
        if api_base and "openrouter.ai" in api_base:
            return True

        if model_name:
            if model_name.startswith("openrouter/"):
                return True
            model_prefix = model_name.split("/", 1)[0] if "/" in model_name else None
            if model_prefix in _DIRECT_PROVIDER_PREFIXES:
                return False

        return bool(self.openrouter_api_key)

    def resolve_litellm_request_config(self, model_name: str) -> LiteLLMRequestConfig:
        api_base = self.openai_api_base
        use_openrouter = self.uses_openrouter(model_name)
        prefix = model_name.split("/", 1)[0] if "/" in model_name else None
        is_direct_gemini = model_name.startswith("gemini/")

        if use_openrouter:
            if model_name.startswith("openrouter/"):
                litellm_model = model_name
            else:
                normalized_model = model_name
                # Preserve the existing direct-Gemini env shape while routing
                # through OpenRouter, whose Gemini models use the `google/...`
                # provider namespace.
                if model_name.startswith("gemini/"):
                    normalized_model = f"google/{model_name.split('/', 1)[1]}"
                litellm_model = f"openrouter/{normalized_model}"
        elif prefix in _DIRECT_PROVIDER_PREFIXES:
            litellm_model = model_name
        else:
            litellm_model = f"openai/{model_name}"

        if use_openrouter:
            api_key = self.openrouter_api_key or self.openai_api_key
            provider: Literal["openrouter", "openai_compatible", "gemini"] = (
                "openrouter"
            )
            resolved_api_base = api_base
        elif is_direct_gemini:
            # Route direct Gemini through the documented AI Studio beta endpoint
            # instead of LiteLLM's Gemini 3 alpha default.
            api_key = self.openai_api_key
            provider = "gemini"
            resolved_api_base = api_base or _GOOGLE_AI_STUDIO_V1BETA_BASE
        else:
            api_key = self.openai_api_key
            provider = "openai_compatible"
            resolved_api_base = api_base

        return LiteLLMRequestConfig(
            model=litellm_model,
            api_key=api_key,
            api_base=resolved_api_base,
            provider=provider,
        )


settings = AgentSettings()


def build_dspy_lm(
    model_name: str | None = None,
    *,
    session_id: str | None = None,
    agent_role: str | None = None,
) -> Any:
    if settings.is_integration_test:
        from controller.agent.mock_llm import MockDSPyLM

        return MockDSPyLM(session_id=session_id, node_type=agent_role)

    resolved_model = model_name or settings.llm_model
    request_config = settings.resolve_litellm_request_config(resolved_model)
    api_key = request_config.api_key or "dummy"
    lm_kwargs: dict[str, Any] = {
        "api_key": api_key,
        "cache": False,
        "timeout": settings.llm_timeout_seconds,
        "max_tokens": settings.llm_max_tokens,
    }
    if request_config.api_base:
        lm_kwargs["api_base"] = request_config.api_base
    return dspy.LM(request_config.model, **lm_kwargs)
