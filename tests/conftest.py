import httpx
import pytest
import os

SERVICES = [
    "http://127.0.0.1:18000",  # Controller
    "http://127.0.0.1:18001",  # Worker Light
    "http://127.0.0.1:18002",  # Worker Heavy
]


@pytest.fixture(autouse=True)
def log_test_marker(request):
    """
    Automatically sends a log marker to all services before and after each test.
    This helps in delimiting logs by test case.
    """
    # Only run for integration tests
    is_integration = any(
        m.name.startswith("integration") for m in request.node.iter_markers()
    )
    if not is_integration and "integration" not in request.node.nodeid:
        yield
        return

    test_id = request.node.nodeid
    marker_start = f"START TEST: {test_id}"

    # Use a sync client for the fixture to avoid complexity with event loops
    with httpx.Client(timeout=1.0) as client:
        for service_url in SERVICES:
            try:
                client.get(f"{service_url}/health", params={"marker": marker_start})
            except Exception:
                # Service might not be up or endpoint doesn't exist, ignore
                pass

    yield

    marker_finish = f"FINISH TEST: {test_id}"
    with httpx.Client(timeout=1.0) as client:
        for service_url in SERVICES:
            try:
                client.get(f"{service_url}/health", params={"marker": marker_finish})
            except Exception:
                pass
