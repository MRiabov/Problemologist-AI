from controller.agent.config import AgentSettings


def test_resolve_dspy_lm_request_config_preserves_single_openrouter_prefix():
    settings = AgentSettings(
        llm_model="openrouter/healer-alpha",
        openrouter_api_key="test-openrouter-key",
    )

    request_config = settings.resolve_dspy_lm_request_config(settings.llm_model)

    assert request_config.model == "openrouter/healer-alpha"
    assert request_config.provider == "openrouter"


def test_resolve_dspy_lm_request_config_prefixes_bare_openrouter_slug_once():
    settings = AgentSettings(
        llm_model="healer-alpha",
        openrouter_api_key="test-openrouter-key",
    )

    request_config = settings.resolve_dspy_lm_request_config(settings.llm_model)

    assert request_config.model == "openrouter/healer-alpha"
    assert request_config.provider == "openrouter"
