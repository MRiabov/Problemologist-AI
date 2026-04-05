---
name: integration-tests-workflow
description: Run, debug, and implement integration tests for RunAudit at the public boundary of the feature under test. Use when updating system integration tests or the YAML-as-checklist experiment suite.
---

# Integration Tests Workflow

Use this workflow when adding, updating, or debugging integration tests in RunAudit.

The important distinction is the boundary under test:

- Core product flows should exercise the real app/worker/storage boundary.
- YAML-as-checklist experiment tests should exercise the experiment runner, frozen artifacts, and compatibility adapter boundary.

## Non-negotiable rules

01. Test the public boundary of the feature under test.
02. Do not import internal modules to execute business logic directly.
03. Do not patch or mock project modules; only isolate unavoidable third-party instability.
04. Assert against observable outputs: HTTP responses, persisted artifacts, manifests, ledgers, DB rows, object storage, logs, and emitted events.
05. Keep deterministic fixtures for model-output variability when the test needs them.
06. Maintain stable test ids:
    - `INT-xxx` for core product integration tests
    - `INT-EXP-xxx` for `specs/experiments/yaml-as-checklist/integration-tests.md`
07. Include at least one expected-fail assertion wherever the architecture specifies fail-closed behavior.
08. Parse JSON, YAML, or similar artifacts into typed models before making assertions.
09. Use state-based polling for readiness and completion rather than hardcoded sleeps.
10. Before trusting logs or artifacts, confirm they belong to the current run.

## Suite shape

- Core product integration tests cover backend, worker, storage, and observability boundaries.
- YAML-as-checklist experiment integration tests cover corpus loading, chunking, mutation freezing, compatibility mapping, scoring, and run reporting.
- Deterministic fixture files are allowed for model outputs only.
- Fixtures must not bypass fail-closed gates or fabricate benchmark truth.

## Fixture guidance

The YAML-as-checklist experiment uses a scenario-based `mock_responses.yaml` fixture file for deterministic model behavior.

Fixture rules:

- keep fixture scenarios minimal and stable
- use fixtures only for model outputs that need determinism
- do not use fixtures to bypass manifest freezes, adapter mapping, or scoring
- keep negative cases real enough to reach the intended evaluated stage

## Common anti-patterns

- Unit-style tests that import internals and skip the real boundary
- Broad mocks that hide boundary failures
- Asserting only on terminal status without artifact evidence
- Treating confidence as a score signal
- Hardcoded sleeps or stale log/artifact assertions

## Validation checklist

Before finishing an integration test change, verify all of the following:

1. The test exercises the public boundary of the feature under test.
2. The test asserts on persisted artifacts, logs, or structured outputs.
3. The test preserves or improves mapped coverage.
4. The test includes a fail-path assertion when applicable.
5. The test id matches the active catalog (`INT-xxx` or `INT-EXP-xxx`).
6. Any deterministic fixture file is scoped to the specific model-output behavior it controls.
