from unittest.mock import MagicMock, patch

import pytest
import schemathesis

from worker_light.api.routes import get_router
from worker_light.app import app
from shared.workers.filesystem.router import FilesystemRouter

# Initialize schemathesis with the app's OpenAPI schema
schema = schemathesis.openapi.from_asgi("/openapi.json", app)

# Exclude heavy worker endpoints from fuzzing to keep tests fast and focused on lightweight contracts
HEAVY_ENDPOINTS_REGEX = (
    r"/benchmark/simulate"
    r"|/benchmark/validate"
    r"|/benchmark/analyze"
    r"|/benchmark/preview"
    r"|/benchmark/build"
)
schema = schema.exclude(path_regex=HEAVY_ENDPOINTS_REGEX)


@pytest.fixture(autouse=True)
def mock_dependencies(tmp_path):
    """Mock external dependencies to prevent side effects."""
    # Mock FilesystemRouter
    router_mock = MagicMock(spec=FilesystemRouter)
    router_mock.ls.return_value = []
    router_mock.read.return_value = b""
    router_mock.exists.return_value = True
    router_mock.edit.return_value = MagicMock()
    router_mock.upload_files.return_value = []

    # Provide a real path for local_backend.root to avoid Repo(MagicMock) errors
    local_backend_mock = MagicMock()
    local_backend_mock.root = tmp_path
    # Mock _resolve to return a real path within tmp_path
    local_backend_mock._resolve.side_effect = lambda p: tmp_path / p
    router_mock.local_backend = local_backend_mock

    app.dependency_overrides[get_router] = lambda: router_mock

    # Mock all heavy utilities and those involving git or subprocesses
    with (
        patch("worker_light.app.sync_skills", side_effect=None),
        patch("worker_light.api.routes.init_workspace_repo"),
        patch("worker_light.api.routes.commit_all", return_value="deadbeef"),
        patch("worker_light.api.routes.resolve_conflict_ours", return_value=True),
        patch("worker_light.api.routes.resolve_conflict_theirs", return_value=True),
        patch("worker_light.api.routes.abort_merge", return_value=True),
        patch("worker_light.api.routes.complete_merge", return_value="deadbeef"),
        patch(
            "worker_light.api.routes.get_repo_status",
            return_value={
                "branch": "main",
                "is_dirty": False,
                "is_merging": False,
                "conflicts": [],
            },
        ),
        patch(
            "worker_light.api.routes.run_python_code_async",
            return_value=MagicMock(
                exit_code=0, stdout="OK", stderr="", timed_out=False
            ),
        ),
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
