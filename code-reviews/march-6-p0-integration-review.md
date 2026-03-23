# P0 Integration Test Spec Drift Report - March 6, 2026

## Overview

This report documents the gap between the current integration test implementation and the mandates defined in `specs/integration-tests.md`. Several P0 (blocker) requirements are either missing, partially implemented, or have drifted into "unit-test-like" patterns that violate the integration contract.

## Critical Drift & Gaps

### 1. Architecture P0 (Backend)

| ID          | Title                          | Status      | Drift/Gap Details                                                                                                                                                                                                                    |
| ----------- | ------------------------------ | ----------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| **INT-005** | Engineer planner artifact gate | **Drift**   | `test_planner_gates.py` uses `benchmark/submit` for artifact checks. The spec requires verifying that `submit_plan` (tool-call) is the mandatory gate and that episode traces contain `TOOL_START` for `node_type=engineer_planner`. |
| **INT-024** | Benchmark validation toolchain | **Missing** | Current tests in `test_architecture_p0.py` only check valid scripts. Missing verification that `/validate` catches intersecting/invalid objective setups across randomization ranges.                                                |
| **INT-126** | Wire tear behavior             | **Missing** | No integrated test for `FAILED_WIRE_TORN` when tension exceeds tensile rating.                                                                                                                                                       |
| **INT-128** | Electronics schema gate        | **Missing** | `benchmark_definition.yaml` validation for `electronics_requirements` is not exercised in integration.                                                                                                                               |

### 2. Frontend P0 (UI Delivery)

| ID          | Title                      | Status        | Drift/Gap Details                                                                                                                    |
| ----------- | -------------------------- | ------------- | ------------------------------------------------------------------------------------------------------------------------------------ |
| **INT-162** | Interrupt UX propagation   | **Partial**   | `test_frontend_p0.py` checks button visibility but does not verify the full lifecycle: API call -> backend interrupt -> stream halt. |
| **INT-163** | Steerability context cards | **Missing**   | No test for multi-select context card creation and removal.                                                                          |
| **INT-164** | Code viewer + mentions     | **Misplaced** | Found in `tests/integration/frontend/` instead of `p0/`. Needs verification of valid/invalid range rejection.                        |
| **INT-165** | CAD topology selection     | **Misplaced** | Found in `tests/integration/frontend/` instead of `p0/`.                                                                             |
| **INT-167** | Controller-proxied assets  | **Missing**   | No verification that CAD fetches use `/api/episodes/{id}/assets/` via controller proxy.                                              |
| **INT-170** | Post-run feedback UX       | **Missing**   | Thumbs up/down modal persistence and retrieve-back not tested.                                                                       |

## Recommendations

1. Refactor `INT-005` and `INT-113/114` to strictly follow the trace-based submission gate logic.
2. Implement missing Electronics P0s (`INT-126` to `INT-128`) to prevent regressions in the circuit validation logic.
