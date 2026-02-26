import httpx
import pytest
import os
import datetime

SERVICES = [
    "http://127.0.0.1:18000",  # Controller
    "http://127.0.0.1:18001",  # Worker Light
    "http://127.0.0.1:18002",  # Worker Heavy
]


@pytest.fixture(autouse=True)
def capture_frontend_logs(request):
    """
    Automatically hooks into the Playwright 'page' fixture (if present) to capture
    console logs and page errors into logs/integration_tests/browser_console.log.
    """
    # Only run for tests that use the 'page' fixture (Playwright)
    if "page" not in request.fixturenames:
        yield
        return

    # request.getfixturevalue("page") will trigger instantiation of the 'page' fixture
    page = request.getfixturevalue("page")
    test_id = request.node.nodeid
    log_dir = "logs/integration_tests"
    log_file = os.path.join(log_dir, "browser_console.log")

    # Ensure log directory exists
    os.makedirs(log_dir, exist_ok=True)

    def handle_console(msg):
        # We want to exclude some extremely noisy logs if necessary,
        # but for now, capture everything.
        timestamp = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
        with open(log_file, "a") as f:
            f.write(f"[{timestamp}] [{test_id}] [{msg.type.upper()}] {msg.text}\n")

    def handle_error(exc):
        timestamp = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
        with open(log_file, "a") as f:
            f.write(f"[{timestamp}] [{test_id}] [BROWSER_EXCEPTION] {exc}\n")

    page.on("console", handle_console)
    page.on("pageerror", handle_error)

    # Initial marker in console log to delimit tests
    with open(log_file, "a") as f:
        f.write(f"\n--- START TEST BROWSER LOG: {test_id} ---\n")

    yield

    with open(log_file, "a") as f:
        f.write(f"--- FINISH TEST BROWSER LOG: {test_id} ---\n")


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
