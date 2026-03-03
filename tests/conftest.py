import datetime
import os
import re

import httpx
import pytest

SERVICES = [
    "http://127.0.0.1:18000",  # Controller
    "http://127.0.0.1:18001",  # Worker Light
    "http://127.0.0.1:18002",  # Worker Heavy
]

# Temporary baseline allowlist for known frontend noise.
# Goal: keep strict mode actionable while we iteratively eliminate existing issues.
DEFAULT_BROWSER_ERROR_ALLOWLIST = [
    r"Failed to parse assembly definition for electronics kb",
    r"Failed to fetch episodes",
    r"Failed to load resource: net::ERR_CONNECTION_REFUSED",
    r"Connection check failed: TypeError: Failed to fetch",
    r"Polling failed TypeError: Failed to fetch",
    r"WebSocket connection to .+ failed: Error in connection establishment: net::ERR_CONNECTION_REFUSED",
    r"WebSocket error Event",
]


def _compile_browser_error_allowlist() -> list[re.Pattern[str]]:
    """
    Build an allowlist of browser error regexes.
    Add custom patterns with BROWSER_ERROR_ALLOWLIST_REGEXES separated by `;;`.
    """
    raw_patterns = [*DEFAULT_BROWSER_ERROR_ALLOWLIST]
    raw_patterns.extend(
        p.strip()
        for p in os.getenv("BROWSER_ERROR_ALLOWLIST_REGEXES", "").split(";;")
        if p.strip()
    )
    return [re.compile(pattern, re.IGNORECASE) for pattern in raw_patterns]


def _is_allowed_browser_error(text: str, allowlist: list[re.Pattern[str]]) -> bool:
    return any(pattern.search(text) for pattern in allowlist)


@pytest.hookimpl(hookwrapper=True)
def pytest_runtest_makereport(item, call):
    """Expose test call outcome on node so fixtures can adjust teardown behavior."""
    outcome = yield
    rep = outcome.get_result()
    setattr(item, f"rep_{rep.when}", rep)


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
    allow_browser_errors = request.node.get_closest_marker("allow_browser_errors")
    strict_mode = os.getenv("STRICT_BROWSER_ERRORS", "1") == "1"
    allowlist = _compile_browser_error_allowlist()
    unexpected_browser_errors: list[str] = []

    # Ensure log directory exists
    os.makedirs(log_dir, exist_ok=True)

    def handle_console(msg):
        # We want to exclude some extremely noisy logs if necessary,
        # but for now, capture everything.
        timestamp = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
        with open(log_file, "a") as f:
            f.write(f"[{timestamp}] [{test_id}] [{msg.type.upper()}] {msg.text}\n")
        if msg.type == "error":
            text = msg.text or ""
            if not _is_allowed_browser_error(text, allowlist):
                unexpected_browser_errors.append(f"console.error: {text}")

    def handle_error(exc):
        timestamp = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
        error_text = str(exc)
        with open(log_file, "a") as f:
            f.write(f"[{timestamp}] [{test_id}] [BROWSER_EXCEPTION] {error_text}\n")
        if not _is_allowed_browser_error(error_text, allowlist):
            unexpected_browser_errors.append(f"pageerror: {error_text}")

    page.on("console", handle_console)
    page.on("pageerror", handle_error)

    # Initial marker in console log to delimit tests
    with open(log_file, "a") as f:
        f.write(f"\n--- START TEST BROWSER LOG: {test_id} ---\n")

    yield

    with open(log_file, "a") as f:
        f.write(f"--- FINISH TEST BROWSER LOG: {test_id} ---\n")

    rep_call = getattr(request.node, "rep_call", None)
    test_call_failed = bool(rep_call and rep_call.failed)
    if (
        strict_mode
        and not allow_browser_errors
        and not test_call_failed
        and unexpected_browser_errors
    ):
        sample = "\n".join(f"- {line}" for line in unexpected_browser_errors[:10])
        overflow = len(unexpected_browser_errors) - 10
        suffix = f"\n- ... and {overflow} more" if overflow > 0 else ""
        pytest.fail(
            "Unexpected browser errors detected.\n"
            f"{sample}{suffix}\n"
            f"See detailed browser logs in {log_file}.\n"
            "To allow expected noise, use marker @pytest.mark.allow_browser_errors "
            "or set BROWSER_ERROR_ALLOWLIST_REGEXES."
        )


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
