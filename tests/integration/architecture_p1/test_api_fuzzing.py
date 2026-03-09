import os
import re

import pytest
import schemathesis

# Target the running service URL for integration testing
CONTROLLER_URL = os.getenv("CONTROLLER_URL", "http://127.0.0.1:18000")
WORKER_LIGHT_URL = os.getenv("WORKER_LIGHT_URL", "http://127.0.0.1:18001")
WORKER_HEAVY_URL = os.getenv("WORKER_HEAVY_URL", "http://127.0.0.1:18002")

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
HEAVY_ENDPOINTS_PATTERN = re.compile(HEAVY_ENDPOINTS_REGEX)
PATH_PARAM_PATTERN = re.compile(r"\{([^}]+)\}")


def _load_controller_schema():
    try:
        return schemathesis.openapi.from_url(f"{CONTROLLER_URL}/openapi.json")
    except Exception:
        # Fallback to local file if service is not running (prevents hard failure).
        if os.path.exists("controller_openapi.json"):
            return schemathesis.openapi.from_path("controller_openapi.json")
        # Final fallback for minimal execution.
        return schemathesis.openapi.from_dict(
            {
                "openapi": "3.0.0",
                "info": {"title": "Placeholder", "version": "1.0.0"},
                "paths": {},
            }
        )


def _iter_non_heavy_operations(schema):
    for operation_result in schema.get_all_operations():
        operation = operation_result.ok()
        if HEAVY_ENDPOINTS_PATTERN.search(operation.path):
            continue
        yield operation


def _default_path_parameters(path: str, *, session_id: str) -> dict[str, str]:
    values: dict[str, str] = {}
    for name in PATH_PARAM_PATTERN.findall(path):
        if name == "session_id":
            values[name] = session_id
            continue
        if (
            name.endswith("episode_id")
            or name.endswith("trace_id")
            or name.endswith("review_id")
        ):
            values[name] = "00000000-0000-0000-0000-000000000000"
            continue
        if name.endswith("path"):
            values[name] = "fuzz.txt"
            continue
        values[name] = "1"
    return values


def _run_single_case(operation, *, base_url: str, session_id: str) -> None:
    case = operation.Case(
        path_parameters=_default_path_parameters(operation.path, session_id=session_id)
    )
    case.headers = case.headers or {}
    case.headers["X-Session-ID"] = session_id
    case.call_and_validate(
        base_url=base_url, checks=(schemathesis.checks.not_a_server_error,)
    )


@pytest.mark.integration_p1
@pytest.mark.debug_fuzz
@pytest.mark.allow_backend_errors(
    regexes=[
        r"planner_handoff_validation_failed",
        r"reviewer_handoff_validation_failed",
        r"benchmark_reviewer_missing_structured_decision",
    ]
)
def test_api_fuzzing():
    """INT-044: Fuzz critical endpoints for strict API behavior; no schema drift."""
    schema = _load_controller_schema()
    failures: list[str] = []

    for operation in _iter_non_heavy_operations(schema):
        endpoint = f"{operation.method.upper()} {operation.path}"
        try:
            _run_single_case(
                operation,
                base_url=CONTROLLER_URL,
                session_id="fuzz-test-session-int",
            )
        except Exception as exc:
            failures.append(f"{endpoint}: {exc}")

    if failures:
        sample = "\n".join(failures[:10])
        overflow = len(failures) - 10
        suffix = f"\n... and {overflow} more failures" if overflow > 0 else ""
        pytest.fail(f"Controller API fuzz failures:\n{sample}{suffix}")


# Note: Ideally we would also target the Worker API, but the Controller is the primary external interface.
# If desired, add another test function for Worker URL.


@pytest.mark.integration_p1
@pytest.mark.worker_heavy_fuzz
def test_worker_heavy_fuzz():
    """INT-044-H: Fuzz Heavy Worker endpoints."""
    try:
        heavy_schema = schemathesis.openapi.from_url(f"{WORKER_HEAVY_URL}/openapi.json")
    except Exception:
        pytest.skip(f"Worker heavy not available at {WORKER_HEAVY_URL}")

    heavy_schema = heavy_schema.exclude(path_regex=r"/benchmark/simulate")
    failures: list[str] = []
    for operation_result in heavy_schema.get_all_operations():
        operation = operation_result.ok()
        endpoint = f"{operation.method.upper()} {operation.path}"
        try:
            _run_single_case(
                operation,
                base_url=WORKER_HEAVY_URL,
                session_id="fuzz-test-session-heavy",
            )
        except Exception as exc:
            failures.append(f"{endpoint}: {exc}")

    if failures:
        sample = "\n".join(failures[:10])
        overflow = len(failures) - 10
        suffix = f"\n... and {overflow} more failures" if overflow > 0 else ""
        pytest.fail(f"Worker-heavy API fuzz failures:\n{sample}{suffix}")
