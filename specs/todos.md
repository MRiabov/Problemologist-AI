# Integration Mock Refactor TODOs

## Purpose

This document tracks a deferred refactor of integration LLM mocks from the current
"per-node fixture" shape to a "conversation transcript" shape.

The refactor is intentionally postponed because the current integration surface is
large (130+ tests), and changing the mock protocol now would create high debugging cost.

## Date: 2026-03-03

## Scope (What This Refactor Must Do)

1. Replace scenario structure in `tests/integration/mock_responses/` with a transcript model.
2. Support ordered multi-agent turns in one scenario (`planner -> coder -> reviewer`, etc.).
3. Represent tool interactions explicitly as step pairs:
   - model output selecting a tool
   - tool observation returned to the model
4. Allow deterministic end-of-conversation assertions in the mock layer (optional per scenario).
5. Keep backward compatibility during migration so existing INT tests can run unchanged first.

## Non-Goals (What This Refactor Must Not Do)

1. Do not rewrite all integration tests in one pass.
2. Do not alter HTTP-level integration assertions to internal/unit-style assertions.
3. Do not couple transcript fixtures to unstable DSPy internals that change per adapter retry.

## Prerequisites Before Starting

1. Integration suite is stable enough that new failures are attributable to refactor work.
2. Add trajectory diagnostics first:
   - expected step index
   - actual step index
   - mismatched field (`tool_name`, `tool_args`, `observation`, final output)
3. Freeze a small pilot set of INT scenarios to migrate first (do not start with all).

## Implementation Plan (When Resumed)

1. Define transcript schema in `tests/integration/mock_responses/` (new section) and document it.
2. Implement transcript reader in `controller/agent/mock_llm.py` behind a feature flag or schema detection.
3. Add compatibility adapter from old per-node schema to transcript runtime behavior.
4. Migrate pilot scenarios and validate via `./scripts/run_integration_tests.sh` targeted tests.
5. Expand migration gradually and remove compatibility layer only after full parity.

## Exit Criteria

1. Pilot INT tests pass with transcript-mode mocks.
2. Failure messages are step-local and actionable (no opaque mismatch errors).
3. Legacy scenarios still run during migration window.
4. Full migration can be done incrementally without multi-day triage spikes.

## TODO: Agent Tool Transport Refactor (HTTP -> WebSockets)

### Goal

Refactor agent tool-call transport from HTTP requests to WebSockets to reduce per-call
overhead and improve end-to-end tool latency (noted estimate: ~20-40% from OpenAI findings).

### Required Work

1. Define a WebSocket protocol for tool requests/responses with strict correlation IDs.
2. Preserve existing auth, permission policy checks, and audit/trace semantics.
3. Implement timeout, retry, and reconnect behavior with deterministic failure handling.
4. Preserve ordering guarantees for tool results consumed by agent runtime.
5. Keep HTTP transport available behind a feature flag during migration.
6. Add integration coverage to compare correctness parity and latency impact.

### Exit Criteria

1. WebSocket transport passes existing integration contracts for tool execution.
2. Observability parity is maintained (tool start/end/error traces and IDs).
3. Controlled rollout path exists with fallback to HTTP.

## TODO: no silent fallbacks pattern

As per conversation:

```md
specs/integration-tests.md; specs/desired_architecture.md

  Are there any "meta" requirements we want to tighten in integration tests as related to desired_architecture? Basically, I've
  found many bugs in integration tests by enforcing no errors logged in the console (as per integration test spec). I'm now
  wondering - anything else I'm missing?
  
[... skipped ... an unsuccessful review; prompted again with higher reasoning model, one of the answers was:]
  
 1. No silent fallback / degraded-success

  - Rule: any fallback/degradation path must be explicit (event + machine-readable flag/reason), never hidden behind 200 OK.
  - Why high value: catches the nastiest production bugs.
  - Cost: low-medium (one shared assertion helper + log/event pattern checks).

  [... another result filtered ...]
  <!--prompted the model on how to actually implement this:-->
Actual implementation: 
• For #1 (“no silent fallback / degraded-success”), the theory is simple:

  1. Define a tiny degradation contract

  - Every endpoint/tool run can return either normal or degraded.
  - If degraded, response must include degraded: true and degradation_reason (enum).
  - No degraded path is allowed to look like a normal success.

  2. Require a matching observability signal

  - Any degraded response must emit one structured event, same IDs (user_session_id, episode_id), same reason.
  - This makes degradation queryable and auditable.

  3. Write integration tests only for forced-degradation scenarios

  - Intentionally trigger known fallback paths (example: GPU OOM retry, external service unavailable, parser fallback).
  - Assert:
      - request still returns expected status (maybe 200, maybe 503, whatever contract says),
      - degraded flag/reason is present when fallback happened,
      - matching degradation event exists,
      - terminal state is not mislabeled as clean success.

  4. Add one global anti-silent assertion
  
  - If a known fallback condition is triggered in test setup, test fails unless degraded flag/event appears.
  - This is the “tripwire” that catches silent regressions.

  That’s it. Minimal surface area, high bug-catching value, and no heavy deterministic/orchestration machinery.
```
