from __future__ import annotations

import os

_TRUTHY = {"1", "true", "yes", "on"}
_SMOKE_TEST_MODE_ERROR = (
    "deprecated functionality removed: smoke_test_mode=true is only allowed "
    "when IS_INTEGRATION_TEST=true"
)


def is_integration_test_enabled() -> bool:
    return os.getenv("IS_INTEGRATION_TEST", "").strip().lower() == "true"


def ensure_smoke_test_mode_allowed(
    smoke_test_mode: bool | None,
    *,
    integration_enabled: bool | None = None,
) -> bool | None:
    if integration_enabled is None:
        integration_enabled = is_integration_test_enabled()
    if smoke_test_mode is True and not integration_enabled:
        raise ValueError(_SMOKE_TEST_MODE_ERROR)
    return smoke_test_mode


def resolve_default_smoke_test_mode(
    *, integration_enabled: bool | None = None
) -> bool:
    env_smoke = os.getenv("SMOKE_TEST_MODE")
    if env_smoke is not None:
        smoke_test_mode = env_smoke.strip().lower() in _TRUTHY
    else:
        smoke_test_mode = (
            integration_enabled
            if integration_enabled is not None
            else is_integration_test_enabled()
        )
    ensured = ensure_smoke_test_mode_allowed(
        smoke_test_mode, integration_enabled=integration_enabled
    )
    assert ensured is not None
    return ensured
