import schemathesis
from fastapi.testclient import TestClient
from src.worker.app import app
from src.worker.api.routes import get_router
from src.worker.filesystem.router import FilesystemRouter
from unittest.mock import MagicMock
import pytest

# Initialize schemathesis with the app's OpenAPI schema
schema = schemathesis.openapi.from_asgi("/openapi.json", app)

@pytest.fixture(autouse=True)
def mock_fs_router():
    """Override the get_router dependency with a mock."""
    mock = MagicMock(spec=FilesystemRouter)
    # Default behaviors to prevent crashes during fuzzing
    mock.ls.return_value = []
    mock.read.return_value = b""
    mock.exists.return_value = True
    mock.edit.return_value = True
    
    app.dependency_overrides[get_router] = lambda: mock
    yield mock
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