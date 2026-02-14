from unittest.mock import MagicMock, patch

import pytest
import schemathesis

from worker.api.routes import get_router
from worker.app import app
from worker.filesystem.router import FilesystemRouter

# Initialize schemathesis with the app's OpenAPI schema
schema = schemathesis.openapi.from_asgi("/openapi.json", app)


@pytest.fixture(autouse=True)
def mock_dependencies():
    """Mock external dependencies to prevent side effects."""
    # Mock FilesystemRouter
    router_mock = MagicMock(spec=FilesystemRouter)
    router_mock.ls.return_value = []
    router_mock.read.return_value = b""
    router_mock.exists.return_value = True
    router_mock.edit.return_value = True
    router_mock.upload_files.return_value = []
    router_mock.local_backend = MagicMock()

    app.dependency_overrides[get_router] = lambda: router_mock

    # Mock sync_skills and watchdog to avoid side effects during startup
    # Also mock git operations exposed via API
    with (
        patch("worker.app.sync_skills"),
        patch("worker.app.start_watchdog"),
        patch("worker.api.routes.init_workspace_repo"),
        patch("worker.api.routes.commit_all", return_value="deadbeef"),
    ):
        yield

    app.dependency_overrides.clear()


@schema.parametrize()
def test_api_fuzzing(case):
    """Fuzz test the API endpoints using Schemathesis.

    This ensures that the API handles various (including invalid) inputs
    gracefully and follows the OpenAPI specification.
    """
    # Add the required session header that our API expects
    case.headers = case.headers or {}
    case.headers["X-Session-ID"] = "fuzz-test-session"

    # Execute the test case against the ASGI app
    response = case.call()

    # Validate the response against the schema
    # We focus on not_a_server_error to ensure no crashes (5xx)
    case.validate_response(response, checks=(schemathesis.checks.not_a_server_error,))
