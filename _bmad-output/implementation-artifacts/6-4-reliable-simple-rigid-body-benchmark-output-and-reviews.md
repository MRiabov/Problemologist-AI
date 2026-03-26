# Story 6.4: Reliable End-to-End Benchmark Generation

Status: ready-for-dev

## Story

As a benchmark maintainer, I want Codex to generate simple rigid-body benchmarks end to end from realistic requests so that a human engineer can use the benchmark generation pipeline for their own design problems without manual correction.

## Acceptance Criteria

1. Given 10 realistic benchmark generation requests, when Codex runs the benchmark generation pipeline from prompt to final plan-review decision, then at least 8 complete successfully without manual correction.
2. Given a successful benchmark generation run, when I inspect the persisted artifacts and traces, then the output is explainable, attributable to the latest revision, and contains no missing or contradictory pipeline artifacts.
3. Given any benchmark generation run in this story, when the pipeline receives an invalid benchmark request or encounters invalid artifact, review, or handoff data, then the request or run is rejected deterministically with an explicit error instead of silently adapting, inferring success, or continuing with corrupted state.
4. Given the resulting pipeline is handed to a human engineer, when they generate benchmarks for their own design requests, then they can use the same benchmark-generation flow without needing a separate debugging or cleanup step.

## Tasks / Subtasks

- [ ] Stand up the real-world benchmark generation eval batch from realistic benchmark requests issued by human engineers. (AC: 1, 4)
  - [ ] Shape the batch around the kinds of benchmark requests a human engineer would actually give the system: simple rigid-body setups with goals, forbidden zones, build zones, allowed interaction constraints, and runtime randomization.
  - [ ] Keep the batch split between expected-pass requests and quality-only technically valid but unsuitable benchmark cases so the eval measures real request handling instead of fixture memorization.
  - [ ] Use `INT-114`, `INT-203`, and `INT-204` as regression anchors for submission, solvability rejection, and latest-revision inspection, but do not treat them as the whole eval set.
  - [ ] Add or refresh a quality-only technically valid but unsuitable benchmark case if the current corpus does not already exercise technically valid but unsuitable output.
  - [ ] Keep the batch deterministic so the 10-attempt usability score is reproducible across reruns.
- [ ] Wire the end-to-end success metric into the eval runner and benchmark review checks. (AC: 1, 2, 3)
  - [ ] Compute a clear success metric for the pipeline, with at least one explicit measure for `usable_without_correction` and one explicit measure for review explainability.
  - [ ] Keep request-intake rejection separate from pipeline artifact rejection so the score reflects both clean acceptance and fail-closed error handling.
  - [ ] Keep the metric logic anchored in the existing eval and review plumbing (`evals/logic/runner.py`, `evals/logic/review_checks.py`, and the benchmark reviewer artifacts) instead of inventing a parallel reporting path.
  - [ ] Preserve latest-revision attribution, render inspection evidence, and explicit refusal reasons in the batch output so failures can be diagnosed from artifacts alone.
- [ ] Iterate the benchmark generator and reviewer behavior until the measured batch passes. (AC: 1, 2, 3, 4)
  - [ ] Tighten `config/prompts.yaml` and the benchmark node/review logic if the batch shows unclear reasoning, stale revision attribution, or review outputs that still require manual correction.
  - [ ] Keep changes fail-closed and regression-driven: if a revision regresses the success rate, treat that as a failing eval until the underlying prompt or validator issue is fixed.
  - [ ] Confirm the final iteration preserves the existing solvability rejection and latest-revision inspection behavior instead of trading trustworthiness for looser acceptance.
- [ ] Extend integration coverage for the pipeline and the 8-of-10 success threshold. (AC: 1, 2, 3, 4)
  - [ ] Refresh `tests/integration/mock_responses/INT-114.yaml`, `tests/integration/mock_responses/INT-203.yaml`, and `tests/integration/mock_responses/INT-204.yaml` if needed so the batch exercises the intended pass, reject, and inspect paths.
  - [ ] Update `tests/integration/architecture_p0/test_planner_gates.py`, `tests/integration/architecture_p1/test_benchmark_workflow.py`, and `tests/integration/architecture_p1/test_reviewer_evidence.py` to assert the batch metric, the explicit review rationale, and the latest-revision evidence contract.
  - [ ] Add or refresh any eval-harness coverage needed to run the batch deterministically and to fail closed when the score drops below 8/10.

## Dev Notes

- Epic 6 is still the simple rigid-body, gravity-enabled benchmark family. The goal of this story is to make the pipeline reliable enough that a human engineer can hand it benchmark requests and get usable benchmark packages back most of the time, not to broaden the physics scope.
- The right abstraction here is an eval loop. Existing benchmark planner and reviewer fixtures provide regression anchors, but the batch itself should resemble realistic benchmark requests from human engineers and drive iteration from that distribution.
- Canonical reviewer comments are stage-specific and schema-owned. Keep the persisted `summary`, `checklist`, and `required_fixes` machine-readable, and keep checklist values to the existing `pass | fail | not_applicable` contract.
- When render images exist for the latest revision, the plan reviewer must still inspect them through `inspect_media(...)` before approval. Text-only file listing does not satisfy that review contract.
- The latest revision review manifest and review decision/comments YAML remain the source of truth for plan-review attribution. Do not let stale artifacts or earlier revisions masquerade as the current decision.
- Quality rejection must stay explicit and deterministic. If the reviewer cannot explain the problem or acceptance reason clearly enough for a human to follow, the correct outcome is rejection with a concrete reason.
- `INT-114`, `INT-203`, and `INT-204` are regression anchors, but the real deliverable is the eval harness and batch metric that measures end-to-end benchmark-generation reliability across realistic Epic 6 user-style requests.
- If the batch usability metric needs a deterministic seed set, keep it keyed to the existing benchmark-planner corpus so the 10-attempt sample is reproducible and comparable across runs.

### Non-Goals

- Do not add actuators, FEM, fluids, or other new physics to Epic 6.
- Do not redesign the engineer workflow or replay/export pipeline in this story.
- Do not replace the existing benchmark review schema with a new review format.
- Do not turn this into a generic dataset-export story; the focus is the benchmark trust eval loop and the system changes needed to pass it.

### Project Structure Notes

- `config/prompts.yaml` owns the benchmark planner and benchmark plan reviewer prompt contracts.
- `controller/agent/benchmark/nodes.py` owns benchmark node behavior, review persistence, and revision attribution.
- `evals/logic/runner.py` and `evals/logic/review_checks.py` are the natural places to express the batch metric and the fail-closed trust score gate.
- `controller/api/tasks.py` may need to carry latest-revision metadata or review outcome normalization if the current trace/persistence path is not enough.
- `tests/integration/architecture_p0/test_planner_gates.py` already covers the latest-revision render inspection gate and should remain a regression anchor.
- `tests/integration/architecture_p1/test_benchmark_workflow.py` and `tests/integration/architecture_p1/test_reviewer_evidence.py` cover benchmark output, reviewer evidence, and persisted checklist behavior.
- `tests/integration/mock_responses/INT-114.yaml`, `tests/integration/mock_responses/INT-203.yaml`, and `tests/integration/mock_responses/INT-204.yaml` are the seeded benchmark-generator fixtures to keep aligned with the story’s reliability and explainability goals.

### Testing Standards Summary

- Use integration tests only; do not add unit-only coverage for this story.
- Assert against HTTP responses, persisted review artifacts, reviewer manifests, traces, and fixture-backed benchmark outputs.
- Verify the usability target through a deterministic seeded batch so the 8-of-10 expectation is reproducible.
- Verify review explainability by checking the persisted review decision/comments YAML, not by matching loose text heuristics in logs.
- Keep failures fail-closed: unclear explanation, missing evidence, or stale revision linkage must not pass as success.

### References

- \[Source: `_bmad-output/planning-artifacts/epics.md`, Epic 6: Gravity: Benchmarks, Story 6.4\]
- \[Source: `_bmad-output/implementation-artifacts/6-1-generate-simple-rigid-body-benchmarks.md`, Story 6.1 baseline simple rigid-body scope and fail-closed submission path\]
- \[Source: `_bmad-output/implementation-artifacts/6-2-produce-a-representative-simple-rigid-body-validation-set.md`, Story 6.2 curated seed coverage and review-fixture alignment\]
- \[Source: `_bmad-output/implementation-artifacts/6-3-refine-simple-rigid-body-benchmark-quality.md`, Story 6.3 benchmark quality-loop and rejected-example preservation\]
- \[Source: `evals/logic/runner.py`, eval execution and batch reporting logic\]
- \[Source: `evals/logic/review_checks.py`, review-check helpers and batch pass/fail evaluation\]
- \[Source: `specs/architecture/agents/overview.md`, benchmark generator workflow split\]
- \[Source: `specs/architecture/agents/roles.md`, benchmark plan reviewer responsibilities and stage-canonical checklist expectations\]
- \[Source: `specs/architecture/agents/handover-contracts.md`, strict planner/reviewer handoff contracts and review YAML pair requirements\]
- \[Source: `specs/architecture/agents/artifacts-and-filesystem.md`, read-only planner handoff rules, review artifact naming, and latest-revision evidence expectations\]
- \[Source: `specs/architecture/agents/tools.md`, `submit_plan()`, `submit_review()`, and `inspect_media(...)` contracts\]
- \[Source: `specs/architecture/evals-and-gates.md`, benchmark reviewer checklist metrics and fail-closed evaluation behavior\]
- \[Source: `specs/architecture/observability.md`, review decision events, evidence tracking, and latest-revision attribution\]
- \[Source: `specs/architecture/simulation-and-dod.md`, simple rigid-body / gravity-only simulation assumptions\]
- \[Source: `specs/business-usecase.md`, user-facing benchmark generation and solution workflow context\]
- \[Source: `specs/dataset-generation.md`, benchmark generation and evaluation dataset expectations\]
- \[Source: `specs/integration-tests.md`, INT-031, INT-034, INT-114, INT-203, and INT-204\]
- \[Source: `config/prompts.yaml`, benchmark planner and benchmark plan reviewer prompt contracts\]
- \[Source: `tests/integration/architecture_p0/test_planner_gates.py`, latest-revision render inspection gate coverage\]
- \[Source: `tests/integration/architecture_p1/test_benchmark_workflow.py`, benchmark output and review workflow coverage\]
- \[Source: `tests/integration/architecture_p1/test_reviewer_evidence.py`, persisted review evidence and checklist coverage\]
- \[Source: `tests/integration/mock_responses/INT-114.yaml`, planner submission fixture\]
- \[Source: `tests/integration/mock_responses/INT-203.yaml`, unsolvable benchmark review rejection fixture\]
- \[Source: `tests/integration/mock_responses/INT-204.yaml`, latest-revision render inspection approval fixture\]

## Dev Agent Record

### Agent Model Used

TBD

### Debug Log References

### Completion Notes List

### File List
