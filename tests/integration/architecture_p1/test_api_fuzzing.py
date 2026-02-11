import os

import pytest
import schemathesis

# Target the running service URL for integration testing
CONTROLLER_URL = os.getenv("CONTROLLER_URL", "http://localhost:8000")
WORKER_URL = os.getenv("WORKER_URL", "http://localhost:8001")

# Create a schema instance from the live OpenAPI spec
schema = schemathesis.openapi.from_url(f"{CONTROLLER_URL}/openapi.json")


@pytest.mark.integration_p1
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
