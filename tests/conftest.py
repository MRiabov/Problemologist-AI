import contextlib
import datetime
import os
import re
import time
from pathlib import Path

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
    r"TypeError: Failed to fetch",
    r"Failed to hydrate episode after status update TypeError: Failed to fetch",
    r"Failed to load resource: net::ERR_CONNECTION_REFUSED",
    r"Connection check failed: TypeError: Failed to fetch",
    r"Polling failed TypeError: Failed to fetch",
    r"WebSocket connection to .+ failed: Error in connection establishment: net::ERR_CONNECTION_REFUSED",
    r"WebSocket error Event",
]

BACKEND_ERROR_LOG_FILES = {
    "controller": Path("logs/integration_tests/controller_errors.log"),
    "worker_light": Path("logs/integration_tests/worker_light_errors.log"),
    "worker_heavy": Path("logs/integration_tests/worker_heavy_errors.log"),
    "temporal_worker": Path("logs/integration_tests/temporal_worker_errors.log"),
}


def _is_integration_test(request: pytest.FixtureRequest) -> bool:
    return (
        any(m.name.startswith("integration") for m in request.node.iter_markers())
        or "integration" in request.node.nodeid
    )


def _should_enforce_integration_readiness(pytestconfig: pytest.Config) -> bool:
    if os.getenv("IS_INTEGRATION_TEST", "").lower() == "true":
        return True

    markexpr = str(getattr(pytestconfig.option, "markexpr", "") or "")
    if "integration" in markexpr:
        return True

    args = [str(arg) for arg in pytestconfig.invocation_params.args]
    return any("tests/integration" in arg or "tests/e2e" in arg for arg in args)


def _wait_for_service_health_stable(
    service_url: str,
    *,
    client: httpx.Client,
    timeout_s: float = 20.0,
    interval_s: float = 0.25,
    consecutive_successes: int = 3,
) -> None:
    deadline = time.monotonic() + timeout_s
    stable = 0

    while time.monotonic() < deadline:
        try:
            response = client.get(f"{service_url}/health")
            if response.status_code == 200:
                stable += 1
                if stable >= consecutive_successes:
                    return
            else:
                stable = 0
        except Exception:
            stable = 0
        time.sleep(interval_s)

    pytest.exit(
        f"{service_url}/health did not become stable "
        f"({consecutive_successes} consecutive successes required)",
        returncode=1,
    )


@pytest.fixture(scope="session", autouse=True)
def ensure_services_are_ready(pytestconfig: pytest.Config):
    """
    Integration-only startup gate.
    Requires stable /health responses before tests execute to avoid cold-start races.
    """
    if not _should_enforce_integration_readiness(pytestconfig):
        return

    with httpx.Client(timeout=2.0) as client:
        for service_url in SERVICES:
            _wait_for_service_health_stable(service_url, client=client)


def _strip_ansi(text: str) -> str:
    return re.sub(r"\x1B\[[0-?]*[ -/]*[@-~]", "", text)


def _compile_backend_error_allowlist() -> list[re.Pattern[str]]:
    raw_patterns = [
        p.strip()
        for p in os.getenv("BACKEND_ERROR_ALLOWLIST_REGEXES", "").split(";;")
        if p.strip()
    ]
    return [re.compile(pattern, re.IGNORECASE) for pattern in raw_patterns]


def _match_backend_issue(line: str, allowlist: list[re.Pattern[str]]) -> str | None:
    normalized = _strip_ansi(line).strip()
    if not normalized:
        return None
    if any(pattern.search(normalized) for pattern in allowlist):
        return None
    return normalized


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


def _has_marker(request: pytest.FixtureRequest, marker_name: str) -> bool:
    """Robust marker presence check for fixture gating."""
    return any(marker.name == marker_name for marker in request.node.iter_markers())


def _compile_marker_regex_allowlist(
    request: pytest.FixtureRequest, marker_name: str
) -> tuple[bool, list[re.Pattern[str]]]:
    """
    Parse marker-provided regex allowlist.
    - No-arg marker means "allow all" (legacy behavior).
    - Marker args/kwargs define scoped regex allowlist.
    """
    patterns: list[str] = []
    has_allow_all = False

    for marker in request.node.iter_markers(name=marker_name):
        if not marker.args and not marker.kwargs:
            has_allow_all = True
            continue

        for arg in marker.args:
            if isinstance(arg, str) and arg.strip():
                patterns.append(arg.strip())

        for key in ("regexes", "patterns", "pattern"):
            value = marker.kwargs.get(key)
            if isinstance(value, str) and value.strip():
                patterns.append(value.strip())
            elif isinstance(value, (list, tuple, set)):
                patterns.extend(
                    str(item).strip() for item in value if str(item).strip()
                )

    compiled: list[re.Pattern[str]] = []
    for pattern in patterns:
        try:
            compiled.append(re.compile(pattern, re.IGNORECASE))
        except re.error as exc:
            raise pytest.UsageError(
                f"Invalid regex in @{marker_name}: {pattern!r} ({exc})"
            ) from exc

    return has_allow_all, compiled


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
    allow_browser_errors = _has_marker(request, "allow_browser_errors")
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
            "If this test explicitly expects this particular error by it's definition in "
            "`specs/integration_tests.md`, you can suppress the exception by "
            '@pytest.mark.allow_browser_errors("long_error_substring_or_regex")'
            "or set BROWSER_ERROR_ALLOWLIST_REGEXES for global patterns."
        )


@pytest.fixture(autouse=True)
def capture_backend_errors(request):
    """
    For backend integration tests, fail when new lines appear in dedicated
    service error logs (ERROR+), unless explicitly allowlisted.
    """
    if not _is_integration_test(request):
        yield
        return

    if request.node.get_closest_marker("integration_frontend"):
        yield
        return

    strict_mode = os.getenv("STRICT_BACKEND_ERRORS", "1") == "1"
    allow_all_backend_errors, marker_allowlist = _compile_marker_regex_allowlist(
        request, "allow_backend_errors"
    )
    allowlist = [*_compile_backend_error_allowlist(), *marker_allowlist]
    start_offsets: dict[str, int] = {}

    for service, path in BACKEND_ERROR_LOG_FILES.items():
        try:
            start_offsets[service] = path.stat().st_size
        except FileNotFoundError:
            start_offsets[service] = 0

    yield

    rep_call = getattr(request.node, "rep_call", None)
    test_call_failed = bool(rep_call and rep_call.failed)
    if not strict_mode or allow_all_backend_errors or test_call_failed:
        return

    issues: list[str] = []
    for service, path in BACKEND_ERROR_LOG_FILES.items():
        if not path.exists():
            continue
        start_offset = start_offsets.get(service, 0)
        with path.open("r", encoding="utf-8", errors="ignore") as handle:
            handle.seek(start_offset)
            for raw_line in handle:
                matched = _match_backend_issue(raw_line, allowlist)
                if matched:
                    issues.append(f"{service}: {matched}")

    if issues:
        sample = "\n".join(f"- {line}" for line in issues[:12])
        overflow = len(issues) - 12
        suffix = f"\n- ... and {overflow} more" if overflow > 0 else ""
        pytest.fail(
            "Unexpected backend errors/exceptions detected in dedicated service error logs.\n"
            f"{sample}{suffix}\n"
            "See logs/integration_tests/{controller,worker_light,worker_heavy,temporal_worker}_errors.log.\n"
            "If this test explicitly expects this particular error by it's definition in "
            "`specs/integration_tests.md`, you can suppress the exception by "
            '@pytest.mark.allow_backend_errors("long_error_substring_or_regex") '
            "or set BACKEND_ERROR_ALLOWLIST_REGEXES for global patterns."
        )


@pytest.fixture(autouse=True)
def log_test_marker(request):
    """
    Automatically sends a log marker to all services before and after each test.
    This helps in delimiting logs by test case.
    """
    # Only run for integration tests
    if not _is_integration_test(request):
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
            with contextlib.suppress(Exception):
                client.get(f"{service_url}/health", params={"marker": marker_finish})
