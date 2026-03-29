from __future__ import annotations

import contextlib
import os

import httpx

_INTEGRATION_TEST_ENDPOINTS = (
    "/api/test/is_integration_test",
    "/test/is_integration_test",
    "/api/health",
    "/health",
)


def controller_reports_integration_test(
    controller_url: str,
    *,
    timeout_s: float = 2.0,
) -> bool | None:
    """Return the controller's integration-test mode if it can be determined.

    The dedicated test endpoint is preferred, with health kept as a compatibility
    fallback for older controller builds.
    """

    base_url = controller_url.rstrip("/")
    with httpx.Client(timeout=timeout_s) as client:
        for path in _INTEGRATION_TEST_ENDPOINTS:
            try:
                response = client.get(f"{base_url}{path}")
            except httpx.HTTPError:
                continue

            if response.status_code != 200:
                continue

            with contextlib.suppress(ValueError):
                payload = response.json()
                if isinstance(payload, dict) and "is_integration_test" in payload:
                    return bool(payload["is_integration_test"])

    return None


def fail_closed_if_integration_test_setup(
    controller_url: str,
    *,
    context: str,
    timeout_s: float = 2.0,
) -> None:
    """Abort when the controller reports integration-test mode."""

    is_integration_test = controller_reports_integration_test(
        controller_url, timeout_s=timeout_s
    )
    if is_integration_test is True:
        raise SystemExit(
            f"deprecated functionality removed: {context} detected integration-test "
            "setup via controller /api/test/is_integration_test"
        )

    if (
        is_integration_test is None
        and os.getenv("IS_INTEGRATION_STEP", "").lower() == "true"
    ):
        raise SystemExit(
            f"deprecated functionality removed: {context} could not verify controller "
            "integration mode while IS_INTEGRATION_STEP=true"
        )
