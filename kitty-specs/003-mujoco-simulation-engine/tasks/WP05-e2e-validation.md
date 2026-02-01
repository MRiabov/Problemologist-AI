---
work_package_id: WP05
title: End-to-End Validation & Reporting
lane: planned
dependencies: ["WP04"]
subtasks:
- T017
- T018
- T019
---

# WP05: End-to-End Validation & Reporting

## Objective

Verify the full system loop with a realistic scenario and add artifact output (replay files).

## Context

We need to prove that a "Real" agent (e.g., a simple hardcoded pusher) can actually "win" the simulation.

## Subtasks

### T017: Create Reference Test Case

**Goal**: A known-good setup.
**Implementation**:

- `tests/fixtures/pusher_bot.py`.
- A generic `build123d` block geometry.
- A script that just applies constant force `motor = 1.0`.
- An environment with a goal zone right in front of it.

### T018: E2E Integration Test

**Goal**: Automated full system test.
**Implementation**:

- `tests/test_e2e.py`.
- Post the Reference Bundle to `FastAPI` (TestClient).
- Assert status 200.
- Assert outcome == "SUCCESS". (Tune the test case so it wins).

### T019: Replay Artifacts

**Goal**: Return visual evidence.
**Implementation**:

- In `SimulationLoop`, enable `mj_saveLastXML` or log states to `.mjcf`.
- Or use `mujoco.mj_saveData`.
- Return the trace file (or path to it) in API response.

## Definition of Done

- [ ] E2E test passes green.
- [ ] We can confirm a "Winning" agent actually gets a "SUCCESS" outcome.
- [ ] Documentation updated with "How to run".
