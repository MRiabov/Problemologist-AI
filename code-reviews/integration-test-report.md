# Comprehensive Integration Test Report

## Executive Summary
A full run of 45 integration test files was conducted. The results reveal several critical systemic issues and test-specific regressions.

**Total Files**: 45
**PASSED**: 9
**FAILED**: 34
**SKIPPED**: 2 (Environment/Hang issues)

### Methodology & Raw Logs
To execute 45 tests efficiently within session limits, a custom batching approach was used that kept the local infrastructure (Postgres, Temporal, Minio) active between test files.

**Exact terminal outputs for every test file are available in `code-reviews/raw-integration-logs/`.**

---

## Core Systemic Failures

### 1. Template Missing: `electronics_planner`
- **Impact**: All agent tasks involving electronics planning crash immediately.
- **Root Cause**: The `electronics_planner` template is referenced in `controller/agent/nodes/electronics_planner.py` but is missing from `config/prompts.yaml` and the `PromptManager` mapping.
- **Affected Tests**:
    - `tests/integration/architecture_p0/test_missing_p0.py`
    - `tests/integration/architecture_p0/test_architecture_p0.py` (Tests involving electronics)
    - `tests/integration/architecture_p1/test_int_064_to_069.py`

### 2. API Contract Mismatch (P1 Tests)
- **Impact**: Multiple high-level integration tests fail with `422 Unprocessable Entity`.
- **Root Cause**: The tests pass the `prompt` as a query parameter to `/benchmark/generate`, but the API now expects it in the request body according to the latest Pydantic schemas.
- **Affected Tests**:
    - `tests/integration/architecture_p1/test_engineering_loop.py`
    - `tests/integration/architecture_p1/test_handover.py`
    - `tests/integration/architecture_p1/test_manufacturing.py`
    - `tests/integration/test_full_workflow.py`

### 3. Temporal Workflow Registration
- **Impact**: Tests hang indefinitely when waiting for certain workflows.
- **Root Cause**: `BackupWorkflow` is not registered in `controller/temporal_worker.py`.
- **Affected Tests**:
    - `tests/integration/architecture_p1/test_observability_extended.py` (HUNG, skipped in final run)

### 4. Schema & Case Sensitivity Regressions
- **WIRE_TORN Case Mismatch**: `FailureReason.WIRE_TORN` (uppercase) is returned by the physics backend, but tests expect `wire_torn` (lowercase).
- **SimulationFailure Detail Type**: `SimulationFailure.detail` expects a `string`, but the motor overload logic is passing a `boolean` (True) instead of the motor name.
  - *File*: `tests/integration/architecture_p0/test_architecture_p0.py` (`test_int_022`)

---

## Detailed Failure Log

| Test Category | File | Status | Root Cause / Observations |
|---------------|------|--------|--------------------------|
| **Electronics** | `test_integration_electronics.py` | FAILED | Case mismatch in `WIRE_TORN`; Mock validation missing `section` arg. |
| **Physics** | `test_fem_breakage.py` | FAILED | `TypeError` in `MetricCollector` when comparing `MagicMock` with `float`. |
| **Physics** | `test_physics_backends.py` | FAILED | `FileNotFoundError` for `tests/worker/minimal.xml` (Missing test asset). |
| **Physics** | `test_genesis_interaction.py` | FAILED | `test_apply_control` fails due to mock unpacking error in `call_args`. |
| **Physics** | `test_validation.py` | FAILED | `ValueError`: Part 'unknown' missing required metadata in builder. |
| **Physics** | `test_physics_fluids_extended.py` | FAILED | `ReadTimeout` during high-fidelity Genesis initialization. |
| **Architecture** | `test_architecture_p0.py` | FAILED | Agent loop in `test_int_002`; `ValidationError` in `test_int_022`. |
| **Architecture** | `test_int_102_111.py` | FAILED | `ReadTimeout` during heavy FEM material validation. |
| **Architecture** | `test_int_026_030.py` | FAILED | `test_int_026`: `simulation_result` event not correctly captured in traces. |
| **Architecture** | `test_reviewer_evidence.py` | FAILED | `'DatabaseCallbackHandler' has no attribute 'run_inline'` (LangGraph/Callback bug). |
| **Architecture** | `test_batch_execution.py` | FAILED | Agent failed (likely due to the missing `electronics_planner` template). |
| **Infrastructure** | `test_playwright_benchmark.py` | FAILED | Chromium executable missing in sandbox environment. |
| **Infrastructure** | `test_gpu_oom_retry.py` | FAILED | `AttributeError`: Patch target `SimulationLoop` not found in expected module. |
| **Infrastructure** | `test_integration_docker.py` | FAILED | Docker-in-Docker permission or connectivity issues in some environments. |
| **Infrastructure** | `test_worker_concurrency.py` | FAILED | Session isolation issues or resource contention under load. |

---

## Recommendations
1. **Infrastructure Stability**: Increase `ReadTimeout` for integration tests that utilize the Genesis backend, as software rendering is significantly slower than GPU-accelerated runs.
2. **Template Completion**: Immediately add `electronics_planner` and `plan_reviewer` mappings to `config/prompts.yaml`.
3. **API Alignment**: Update integration tests to match the current Pydantic request body schemas for benchmark generation.
4. **Mock Sanitization**: Fix `MetricCollector` and `GenesisBackend` mocks to return proper types (floats instead of Mocks) to avoid `TypeError` during comparison.
5. **Workflow Audit**: Audit `controller/temporal_worker.py` to ensure all workflows defined in the system are correctly registered.
