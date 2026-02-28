# Audit Report: Commit 6484acc vs Integration Test Specification

This report evaluates the adherence of the recent merge (commit `6484acc155adb26b72655d5b1cfbc76f803299ad`) to the requirements defined in [integration-tests.md](file:///home/maksym/Work/proj/Problemologist/Problemologist-AI/specs/integration-tests.md).

## Summary Verdict
**Partial Adherence with Critical Violations.**
While the commit introduces the recommended structure for P0/P1 architecture tests and correctly implements several HTTP-based tests, it also includes a fallback mechanism and several test files that directly violate the "Non-negotiable Integration Execution Contract".

---

## ✅ Adherences

- **Mapped IDs**: New tests correctly use `INT-xxx` IDs in their names and docstrings (e.g., [test_int_001_compose_boot_health_contract](file:///home/maksym/Work/proj/Problemologist/Problemologist-AI/tests/integration/architecture_p0/test_architecture_p0.py#28-47), [test_int_030_interrupt_propagation](file:///home/maksym/Work/proj/Problemologist/Problemologist-AI/tests/integration/architecture_p0/test_int_026_030.py#225-266)).
- **HTTP/UI Boundary**: [tests/e2e/test_int_170_171.py](file:///home/maksym/Work/proj/Problemologist/Problemologist-AI/tests/e2e/test_int_170_171.py) (Playwright) and [tests/integration/architecture_p0/test_architecture_p0.py](file:///home/maksym/Work/proj/Problemologist/Problemologist-AI/tests/integration/architecture_p0/test_architecture_p0.py) (httpx) correctly interact with the system via external boundaries.
- **Strict Typing**: Tests in [test_architecture_p0.py](file:///home/maksym/Work/proj/Problemologist/Problemologist-AI/tests/integration/architecture_p0/test_architecture_p0.py) and [test_int_026_030.py](file:///home/maksym/Work/proj/Problemologist/Problemologist-AI/tests/integration/architecture_p0/test_int_026_030.py) correctly use `PartMetadata` from `shared.models.schemas` as required by line 39 of the spec.
- **Event assertions**: [test_int_026_mandatory_event_families](file:///home/maksym/Work/proj/Problemologist/Problemologist-AI/tests/integration/architecture_p0/test_int_026_030.py#18-98) correctly asserts against the event stream in the simulation result.

---

## ❌ Non-Adherences (Critical Violations)

### 1. Forbidden Patching & In-Process Fallback
> [!WARNING]
> **Violation of Rule 31 & 34**
> [conftest.py](file:///home/maksym/Work/proj/Problemologist/Problemologist-AI/tests/integration/architecture_p0/conftest.py) implements a fallback that imports application internals (FastAPI [app](file:///home/maksym/Work/proj/Problemologist/Problemologist-AI/tests/worker_heavy/simulation/test_loop.py#149-162), etc.) and uses `patch`/`AsyncMock` to fake services if the real stack is not reachable.
>
> **Spec Reference**:
> - "No `patch`, `monkeypatch`, or fake clients... in integration tests." (Line 34)
> - "Reject as unit-test anti-pattern: Importing FastAPI app/TestClient only." (Line 225)

### 2. Unit Tests Disguised as Integration Tests
> [!IMPORTANT]
> Several files included in the diff are effectively unit tests that mock internal logic rather than exercising the container stack:
> - [test_genesis_builder.py](file:///home/maksym/Work/proj/Problemologist/Problemologist-AI/tests/integration/test_genesis_builder.py): Uses `@patch` on `export_stl` and `tetrahedralize`.
> - [test_physics_fluids_wp04.py](file:///home/maksym/Work/proj/Problemologist/Problemologist-AI/tests/integration/test_physics_fluids_wp04.py): mocks the entire Genesis (`gs`) module and physics backend.
> - [test_electronics_engineer.py](file:///home/maksym/Work/proj/Problemologist/Problemologist-AI/tests/integration/test_electronics_engineer.py): Directly imports and mocks LangGraph nodes.
>
> **Spec Reference**:
> - "A test that imports app internals... to invoke business logic directly is **not** integration coverage." (Line 38)
> - "CONTROL THE TEST EXECUTION **ONLY** BY HTTP REQUESTS... DO NOT DO TESTS THAT ARE EFFECTIVELY UNIT TESTS" (Line 14-15)

---

## Recommendations for Remediation

1.  **Remove In-Process Fallbacks**: The [conftest.py](file:///home/maksym/Work/proj/Problemologist/Problemologist-AI/tests/integration/architecture_p0/conftest.py) should only yield a client pointing to `CONTROLLER_URL`. If the service is unreachable, the test should fail with a connection error, forcing the user to bring up the compose stack.
2.  **Refactor Mocked Tests**: Files like [test_genesis_builder.py](file:///home/maksym/Work/proj/Problemologist/Problemologist-AI/tests/integration/test_genesis_builder.py) should be moved to a unit test directory or refactored to trigger the respective logic via the `worker-heavy` API, asserting against the produced `.msh` or `.stl` files in S3/MinIO.
3.  **Strictly Enforce No-Patch Rule**: Any use of `unittest.mock.patch` in the `tests/integration/` directory should be considered a lint error.
