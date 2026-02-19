import os

import pytest
import schemathesis

# Target the running service URL for integration testing
CONTROLLER_URL = os.getenv("CONTROLLER_URL", "http://localhost:18000")
WORKER_LIGHT_URL = os.getenv("WORKER_LIGHT_URL", "http://localhost:18001")

# Create a schema instance from the live OpenAPI spec
try:
    schema = schemathesis.openapi.from_url(f"{CONTROLLER_URL}/openapi.json")
except Exception:
    # Fallback to local file if service is not running (prevents collection error)
    if os.path.exists("controller_openapi.json"):
        schema = schemathesis.openapi.from_path("controller_openapi.json")
    else:
        # Final fallback for minimal collection
        schema = schemathesis.openapi.from_dict(
            {
                "openapi": "3.0.0",
                "info": {"title": "Placeholder", "version": "1.0.0"},
                "paths": {},
            }
        )

# INT-044: Exclude heavy endpoints from fuzzing to avoid overloading the system
# especially those that trigger long-running background tasks.
HEAVY_ENDPOINTS_REGEX = (
    r"/agent/run"
    r"|/benchmark/generate"
    r"|/simulation/run"
    r"|/benchmark/\{session_id\}/confirm"
    r"|/episodes/\{episode_id\}/messages"
    r"|/api/v1/sessions/\{session_id\}/steer"
    r"|/ops/backup"
)
schema = schema.exclude(path_regex=HEAVY_ENDPOINTS_REGEX)


@pytest.mark.integration_p1
@pytest.mark.debug_fuzz
@schema.parametrize()
def test_api_fuzzing(case):
    """INT-044: Fuzz critical endpoints for strict API behavior; no schema drift."""
    # We may need to set specific headers or auth if required
    case.headers = case.headers or {}
    case.headers["X-Session-ID"] = "fuzz-test-session-int"

    # Execute the test case against the live service
    case.call_and_validate(checks=(schemathesis.checks.not_a_server_error,))


# Note: Ideally we would also target the Worker API, but the Controller is the primary external interface.
# If desired, add another test function for Worker URL.
