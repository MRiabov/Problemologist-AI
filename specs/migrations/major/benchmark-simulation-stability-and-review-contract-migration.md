---
title: Benchmark Simulation Stability and Review Contract Migration
status: migration
agents_affected:
  - benchmark_planner
  - benchmark_plan_reviewer
  - benchmark_coder
  - benchmark_reviewer
added_at: '2026-04-09T00:00:00Z'
---

# Benchmark Simulation Stability and Review Contract Migration

<!-- Major migration. Benchmark-side simulation must stop acting like a solve gate. -->

## Purpose

This migration removes the accidental solve requirement from the benchmark-side
simulation and review flow.

The benchmark graph exists to create and hand off a problem instance for the
engineer graph to solve. It is not supposed to force the benchmark creator to
prove that the benchmark itself reaches the task goal. The benchmark-side
simulation contract should therefore validate that the benchmark scene is
physically stable, that the declared benchmark motion is reconstructable when
motion exists, and that the authored benchmark package is consistent with the
saved workspace snapshot. It should not require goal completion, terminal-event
proof, or engineer-style payload trajectory semantics.

The engineer-side motion forecast and payload trajectory contracts remain
unchanged. `goal_zone_entry`, `goal_zone_contact`, `terminal_event`, and
related proof fields stay engineer-owned and do not become benchmark-side
semantics.

Where benchmark and engineer contracts share stable fields, the implementation
should factor them into reusable schema fragments or strict base models. The
benchmark and engineer artifacts should remain separate concrete models, but
they should compose the same shared field definitions and validators instead of
duplicating whole schemas.

## Problem Statement

The current benchmark flow still inherits engineer-style success language in
multiple places.

1. `simulate_benchmark()` currently runs through the same shared simulation
   path that interprets goal-zone completion as the decisive success signal
   whenever a goal zone exists.
2. The benchmark submission gate rejects the benchmark unless the simulation
   summary looks like the benchmark "reached the objective".
3. The benchmark review handover repeats that check and persists a
   `goal_reached` flag in the benchmark review manifest.
4. The benchmark controller path treats a benchmark simulation result as
   successful only when the summary text says goal completion occurred.
5. Benchmark workflow tests currently assert that the benchmark review manifest
   reports `goal_reached is True`.
6. This leaks engineer semantics into the benchmark graph and makes the
   benchmark creator look like it must solve the environment before it can hand
   it off.
7. The benchmark motion contract is already explicit in the architecture, but
   it is about benchmark-owned fixture behavior and motion evidence, not about
   a benchmark-side terminal goal event.

## Current-State Inventory

| Area | Current behavior | Why it must change |
| -- | -- | -- |
| `worker_heavy/simulation/loop.py` | When a scene has goal sites, the simulation loop marks success only if the tracked body reaches the goal. | Benchmark-side simulation should be able to stop on stability and benchmark-contract compliance without requiring a solved goal state. |
| `worker_heavy/utils/validation.py` | The summary text becomes `Goal achieved.` when the loop reports success, which reinforces the solve-oriented meaning. | Benchmark-side success text should distinguish stability from engineer goal completion. |
| `worker_heavy/utils/handover.py` | Benchmark submission rejects unless `_goal_reached(simulation_result.summary)` is true, and the persisted benchmark review manifest stores `goal_reached`. | Benchmark handoff should not require the benchmark creator to prove a solve. |
| `controller/agent/review_handover.py` | Benchmark review entry rejects if the simulation summary or review manifest does not confirm goal completion. | Review routing must validate benchmark stability and evidence, not engineer-style goal completion. |
| `controller/agent/benchmark/nodes.py` | The benchmark graph treats a persisted simulation result as successful only when its summary says goal achieved / green zone / goal zone. | Benchmark generation must accept a stable benchmark run even when the benchmark itself is not solving the task. |
| `shared/workers/schema.py` | `ReviewManifest` includes a required `goal_reached` field shared across stages. | The benchmark stage should not be forced to serialize engineer-only solve semantics. |
| `tests/integration/architecture_p1/test_benchmark_workflow.py` | Asserts `manifest.goal_reached is True` for the benchmark flow. | The regression suite currently enforces the wrong benchmark contract. |
| `tests/integration/architecture_p1/test_handover.py` | Asserts `manifest.goal_reached is True` for benchmark-to-engineer handoff. | The benchmark handoff test should assert stability and evidence, not solve completion. |
| `specs/architecture/simulation-and-rendering.md` | Documents benchmark-owned motion as validation setup, but the runtime still behaves as if benchmark-side goal completion is required. | The architecture and runtime need to agree on the benchmark contract. |
| `specs/architecture/agents/tools.md` | Describes `simulate_benchmark()` as a benchmark-side helper, but the shared runtime behavior still leaks goal-reaching semantics into benchmark approval. | The tool contract needs a benchmark-neutral success definition. |
| `specs/architecture/agents/roles-detailed/benchmark-coder.md` and `benchmark-reviewer.md` | Role guidance still says to validate and simulate before handoff without making the benchmark-vs-solution distinction explicit enough. | Benchmark roles need explicit stability/evidence wording. |

## Proposed Target State

1. `validate_benchmark()` remains the static gate for benchmark admissibility.
   It checks geometry, objective consistency, motion metadata, labels, and
   workspace fidelity.
2. `simulate_benchmark()` becomes a benchmark-stability and evidence pass.
   It confirms that the benchmark scene runs without physics failure and, when
   benchmark-owned motion exists, that the declared motion is reproduced in the
   evidence artifacts.
3. Benchmark-side simulation success no longer depends on goal-zone completion
   or any terminal-event proof copied from the engineer motion-forecast
   contract.
4. `goal_reached` stops being a benchmark review contract field and stops
   being a benchmark approval prerequisite. If the runtime needs a
   machine-readable benchmark motion status, it must be benchmark-neutral and
   must not encode solve completion.
5. Benchmark review is based on the approved handoff, validation success,
   simulation stability, render/video evidence, and reviewer judgment about
   benchmark solvability. The reviewer does not require the benchmark to solve
   itself.
6. Engineer-side motion-forecast and payload-trajectory semantics remain
   untouched. Terminal goal proof stays engineer-owned.
7. No new benchmark-side terminal-event concept is introduced by this migration.
   Benchmark motion stays descriptive and evidence-based, not goal-solve based.
8. Shared fields and validators should be factored into reusable Pydantic base
   models or schema fragments, then composed into separate benchmark and
   engineer concrete contracts. The two stages should not duplicate the full
   model definitions, but they also should not collapse into one permissive
   model with optional solve fields.

## Required Work

### 1. Split benchmark-side simulation semantics

- Introduce an explicit benchmark-side simulation mode or equivalent contract
  flag in the shared submission helper and the worker-heavy simulation entry
  point.
- Make that benchmark mode evaluate stability, fail-fast physics errors, and
  benchmark motion evidence without requiring goal completion.
- Keep engineer-side simulation behavior unchanged.
- Preserve the existing physics failure taxonomy.

### 2. Remove benchmark solve gates from handover and review

- Remove benchmark-side `_goal_reached(...)` checks from
  `worker_heavy/utils/handover.py`.
- Stop requiring goal completion in `controller/agent/review_handover.py` for
  benchmark review entry.
- Update `controller/agent/benchmark/nodes.py` so a stable benchmark simulation
  can be considered successful even when the benchmark scene does not reach the
  task goal.
- Keep benchmark motion evidence checks and visual-inspection requirements in
  place for moving benchmark fixtures.

### 3. Retire benchmark-side `goal_reached` from the review manifest contract

- Split the shared review manifest contract into stage-aware validation so the
  benchmark review manifest no longer requires `goal_reached`. Keep the common
  manifest fields in shared fragments or base models so benchmark and
  engineer-specific contracts do not diverge by copy-paste.
- If a machine-readable benchmark motion status is retained, rename it to a
  benchmark-neutral evidence field such as `motion_evidence_verified` or an
  equivalent stage-specific flag.
- Do not preserve `goal_reached` as a benchmark approval prerequisite.
- Keep the engineer execution review contract free to retain goal-completion
  semantics if needed.

### 4. Update benchmark-facing documentation

- Update `specs/architecture/simulation-and-rendering.md` so benchmark-side
  simulation is described as a stability/evidence run, not a solve run.
- Update `specs/architecture/agents/tools.md` so `simulate_benchmark()` is
  benchmark-neutral and does not imply goal completion.
- Update `specs/architecture/agents/handover-contracts.md` so benchmark review
  routes on benchmark stability and evidence rather than benchmark-side goal
  completion.
- Update `specs/architecture/agents/roles-detailed/benchmark-coder.md` and
  `specs/architecture/agents/roles-detailed/benchmark-reviewer.md` so their
  acceptance language distinguishes benchmark stability from engineer success.
- Update any benchmark workflow docs that still describe the benchmark review
  manifest as requiring `goal_reached`.

### 5. Refresh benchmark tests and seeded fixtures

- Update `tests/integration/architecture_p1/test_benchmark_workflow.py` and
  `tests/integration/architecture_p1/test_handover.py` so benchmark handoff
  assertions no longer require `manifest.goal_reached is True`.
- Add a regression that accepts a benchmark review when the simulation is
  stable and the benchmark motion evidence is present, even if the summary does
  not mention goal completion.
- Keep negative coverage for invalid geometry, missing benchmark motion data,
  and stale review manifests.
- Refresh mock responses and seeded benchmark episodes that currently encode
  "goal achieved" as the benchmark success marker.

## Non-Goals

- Do not change the engineer-side motion-forecast or payload-trajectory
  contract.
- Do not introduce a benchmark `terminal_event` or a benchmark payload
  trajectory artifact.
- Do not require the benchmark creator to solve the environment before the
  benchmark can be approved.
- Do not add a new benchmark `verify` requirement to the handoff flow.
- Do not relax benchmark geometry validation, motion metadata validation, or
  render-evidence requirements.
- Do not remove the goal zone from benchmark definitions. The goal zone remains
  part of the benchmark problem definition for engineers to solve against; it
  simply stops being a benchmark-side success criterion.

## Sequencing

The safe implementation order is:

1. Remove the benchmark goal-completion gate from the handover/review
   consumers.
2. Add benchmark-stability semantics to the shared simulation helper or
   worker-heavy simulation entry point.
3. Split or stage-scope the review manifest contract so benchmark approval no
   longer depends on `goal_reached`.
4. Update benchmark-facing docs and role guidance.
5. Refresh benchmark workflow tests, seed fixtures, and mock responses.

## Acceptance Criteria

1. A benchmark episode can complete its handoff when validation passes and the
   simulation is stable, even if the simulation summary does not claim goal
   completion.
2. Benchmark review no longer requires `goal_reached` or any benchmark-side
   terminal-event proof.
3. Benchmark reviewers still inspect motion evidence when moving benchmark
   fixtures exist, and still reject invalid geometry, missing motion metadata,
   or contradictory evidence.
4. Engineer-side simulation continues to require goal completion and terminal
   proof exactly as before.
5. The benchmark-facing docs, role guidance, and integration tests all describe
   the same benchmark-neutral simulation contract.
6. `goal_reached` does not appear in benchmark review approval logic, and any
   retained machine-readable benchmark motion status is benchmark-neutral rather
   than solve-oriented.

## Migration Checklist

### Contract plumbing

- [ ] Split `shared/workers/schema.py::ReviewManifest` into shared fields plus
  stage-specific benchmark and engineer contracts so benchmark review no
  longer requires `goal_reached`.
- [ ] Keep the engineer-side review contract able to serialize
  `goal_reached` and any other goal-completion fields without reintroducing
  them into the benchmark review contract.
- [ ] Add an explicit benchmark-versus-engineer simulation intent or stage flag
  to the shared submission path in `shared/utils/agent/__init__.py` and the
  worker client in `controller/clients/worker.py`.
- [ ] Thread that intent through the controller RPC and the heavy-worker
  simulation entrypoint so benchmark simulation can choose a stability/evidence
  mode without copying engineer solve semantics.
- [ ] Add a benchmark-payload observation window to the benchmark simulation
  path, with a policy default of 1.5 seconds, so payload out-of-bounds before
  that window is a hard failure and payload out-of-bounds after that window is
  recorded as benchmark evidence rather than a benchmark-simulation failure.
- [ ] Keep that grace-window exception scoped to the benchmark payload only;
  benchmark-owned fixtures and simulation bounds remain fail-closed at all
  times.
- [x] Update `worker_heavy/simulation/loop.py` so benchmark mode records
  stability and benchmark motion evidence without converting goal-zone reach
  into benchmark approval success.
- [x] Update `worker_heavy/utils/validation.py` so benchmark-side success text
  is benchmark-neutral while engineer-side success text remains
  goal-oriented.
- [x] Remove benchmark review gating on `_goal_reached(...)` from
  `worker_heavy/utils/handover.py`, `controller/agent/review_handover.py`, and
  `controller/agent/benchmark/nodes.py`.
- [x] Keep benchmark render persistence, attachment summaries, and review
  manifest artifacts intact while omitting benchmark-side `goal_reached`.
- [x] If a machine-readable benchmark motion status is still needed, rename it
  to a benchmark-neutral evidence field and define it on the shared base
  contract instead of reusing solve-completion wording.

### Docs and prompts

- [ ] Update `specs/architecture/simulation-and-rendering.md` so benchmark
  simulation is described as a stability/evidence pass, not a solve pass.
- [ ] Update `specs/architecture/agents/tools.md` so `simulate_benchmark()`
  is benchmark-neutral and does not imply goal completion.
- [x] Update `specs/architecture/agents/handover-contracts.md` so benchmark
  review routes on benchmark stability and evidence rather than benchmark-side
  goal completion.
- [ ] Update `specs/architecture/agents/roles-detailed/benchmark-coder.md`
  and `specs/architecture/agents/roles-detailed/benchmark-reviewer.md` so the
  benchmark acceptance language distinguishes stability from engineer success.
- [ ] Update `docs/api-contracts.md` and any inline contract comments that
  still describe benchmark review as `goal_reached`-based.
- [ ] Remove or rewrite prompt text that teaches benchmark agents to solve the
  benchmark during simulation or review.

### Tests, fixtures, and evals

- [x] Update `tests/integration/architecture_p1/test_benchmark_workflow.py`
  so benchmark handoff passes without `manifest.goal_reached is True`.
- [x] Update `tests/integration/architecture_p1/test_handover.py` so the
  benchmark handoff path asserts stability and evidence instead of benchmark
  goal completion.
- [x] Add a regression that approves a benchmark when simulation is stable and
  evidence is present even if the summary does not mention goal completion.
- [x] Add regressions that still reject invalid geometry, missing motion
  metadata, stale simulations, and contradictory evidence.
- [ ] Add regression coverage for early benchmark-payload out-of-bounds
  failing benchmark simulation and late benchmark-payload out-of-bounds not
  failing benchmark simulation after the configured grace window.
- [x] Refresh benchmark-side mock responses, seeded episodes, and review
  manifests that still encode `goal_reached` as the benchmark success marker.
- [ ] Update `specs/integration-test-list.md` and any eval rows that still map
  benchmark approval to solve completion.
- [ ] Verify the engineer-side integration tests still expect goal completion
  and terminal proof unchanged.

### Cutover verification

- [x] Run the narrow benchmark handoff and review integration slice first.
- [ ] Widen to reviewer-evidence and node-entry validation slices only if the
  narrow slice exposes shared-contract fallout.
- [ ] Confirm no new benchmark `verify` path, route, or handoff prerequisite
  was introduced.
- [ ] Freeze the migration doc after implementation lands and the checklist is
  complete.

## File-Level Change Set

- `worker_heavy/simulation/loop.py`
- `worker_heavy/utils/validation.py`
- `worker_heavy/utils/handover.py`
- `controller/agent/review_handover.py`
- `controller/agent/benchmark/nodes.py`
- `shared/workers/schema.py`
- `shared/utils/agent/__init__.py`
- `specs/architecture/simulation-and-rendering.md`
- `specs/architecture/agents/tools.md`
- `specs/architecture/agents/handover-contracts.md`
- `specs/architecture/agents/roles-detailed/benchmark-coder.md`
- `specs/architecture/agents/roles-detailed/benchmark-reviewer.md`
- `docs/api-contracts.md`
- `tests/integration/architecture_p1/test_benchmark_workflow.py`
- `tests/integration/architecture_p1/test_handover.py`
- `tests/integration/architecture_p1/test_reviewer_evidence.py`
- `specs/integration-test-list.md`

## Test Impact

- `tests/integration/architecture_p1/test_benchmark_workflow.py`
- `tests/integration/architecture_p1/test_handover.py`
- `tests/integration/architecture_p1/test_reviewer_evidence.py` if it asserts
  benchmark-manifest goal completion anywhere in its benchmark fixtures
- `tests/integration/architecture_p0/test_node_entry_validation.py` if any
  seeded benchmark entry still expects goal-completion wording
- `specs/integration-test-list.md` for any benchmark flow rows that currently
  encode `goal_reached` as a benchmark acceptance signal
