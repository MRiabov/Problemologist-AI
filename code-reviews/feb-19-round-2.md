# Integration Test Audit - Feb 19, 2026

## Overview

This audit compares the integration test coverage in `tests/integration/` against the requirements defined in `specs/integration-tests.md`. 

While the project has established a strong directory structure (`smoke/`, `architecture_p0/`, `architecture_p1/`, `evals_p2/`), there is a significant gap in coverage for the recently added **WP2 (Fluids/FEM)** and **WP3 (Electronics)** features.

## Missing Integration Tests

### P0: Architecture Parity Baseline

| ID | Title | Status | Notes |
|---|---|---|---|
| INT-014 | COTS Propagation | Missing | Needs verification that COTS data persists into `plan.md` and `assembly_definition.yaml`. |
| INT-024 | Benchmark Validation | Missing | Needs to test the `/validate` endpoint with conflicting geometry/objectives. |
| INT-025 | Events Collection E2E | Missing | Marked as "todo" in `test_observability_extended.py`. Needs real event stream ingestion check. |
| INT-027 | Seed/Variant Tracking | Missing | Verify DB persistence of `variant_id` and `seed` from the API response. |
| INT-120 | Circuit Validation Gate | Missing | Verification that `/validate_circuit` blocks `/simulate`. |
| INT-121 | Short Circuit Detection | Missing | Assert `FAILED_SHORT_CIRCUIT` status via API. |
| INT-122 | PSU Overcurrent | Missing | Assert `FAILED_OVERCURRENT_SUPPLY` status via API. |
| INT-123 | Wire Overcurrent | Missing | Assert `FAILED_OVERCURRENT_WIRE` status via API. |
| INT-124 | Open Circuit | Missing | Assert `FAILED_OPEN_CIRCUIT` status via API. |
| INT-125 | Motor Power Gating | Missing | Verify torque = 0 when `is_powered` is 0. |
| INT-126 | Wire Tear | Missing | Verify `FAILED_WIRE_TORN` when tension exceeds tensile rating. |
| INT-127 | Electronics Compat | Missing | Verify pre-WP3 benchmarks default to `is_powered = 1.0`. |
| INT-128 | Electronics Schema | Missing | Verify `objectives.yaml` validation for electronics fields. |

### P1: Full Architecture Workflow

| ID | Title | Status | Notes |
|---|---|---|---|
| INT-131 | Fluid Benchmark Loop | Missing | Full Planner -> Engineer -> Reviewer loop for fluid containment tasks. |
| INT-132 | Elec-Mech Loop | Missing | Full integration of mechanism design, wiring, and review. |
| INT-133 | Wire Conflict Handoff | Missing | Trigger Elec -> Mech handover via wire/mesh intersection. |
| INT-134 | Stress Heatmap | Missing | Verify S3 persistence and discoverability of stress heatmap images. |
| INT-135 | Wire Clearance Check | Missing | Verify rejection of routes intersecting solid parts. |
| INT-136 | Power Budgeting | Missing | Verify correct draw calculation and PSU warnings. |
| INT-137 | COTS Electrical | Missing | Query for PSUs, relays, and connectors in `parts.db`. |
| INT-140 | Wire/Elec Costing | Missing | Verify inclusion of per-meter wire costs in `validate_and_price`. |
| INT-141 | Circuit Transient | Missing | Verify motor state timeline from transient solver. |

### P2: Evaluation Architecture

| ID | Title | Status | Notes |
|---|---|---|---|
| INT-151 | Breakage Prevention | Missing | Multi-seed statistical check for `PART_BREAKAGE`. |
| INT-152 | Safety Factor Range | Missing | Verify average safety factor is within 1.5-5.0. |
| INT-153 | Fluid Eval | Missing | Containment success rate target (50%). |
| INT-154 | Elec Success Rate | Missing | Valid circuit generation success rate target (80%). |
| INT-155 | Wire Jitter Survival | Missing | Survival rate check for wire routing under jitter. |
| INT-156 | Gating Correctness | Missing | Verify 95% accuracy in power gating behavior. |

## Implementation Gaps & Observations

1.  **Stubs/Skips:** 
    *   `INT-106` (Flow Rate) is currently a stub.
    *   `INT-110` (GPU OOM) is implemented but skips if no GPU is present; needs a deterministic mock trigger for CI environments.
2.  **Observability:** `INT-025` is critical for ensuring the data flywheel (events -> database) is functioning. Currently, events are verified only as part of the tool response (`BenchmarkToolResponse`), not at the final persistence boundary.
3.  **Boundary Enforcement:** Several tests (e.g., `test_manufacturing.py`) skip negative path testing (e.g., intentionally using bad materials) which is required by the "Non-negotiable Integration Execution Contract".

## Recommendations

1.  **Prioritize WP3 P0:** Implement `INT-120` to `INT-128` as these represent the core safety gates for the electronics feature.
2.  **Fix INT-025:** Implement a dedicated test that queries the events table or `events.jsonl` persistence after an episode to ensure the observability pipeline is closed-loop.
3.  **Refine P2 Evaluations:** Implement `INT-151` through `INT-156` using the existing `evals_p2` structure to ensure quality regressions are caught early.
