# Quickstart: Automatic Node Entry Validation

## Prerequisites

1. Start integration stack (`controller`, `worker-light`, `worker-heavy`, `postgres`, `temporal`, `minio`).
2. Ensure `IS_INTEGRATION_TEST=true` for fail-fast verification scenario.
3. Use HTTP-only flows for all verifications.

## Scenario A: Non-integration reroute behavior

1. Start controller in non-integration mode.
2. Create an episode that reaches a node with invalid entry prerequisites (for example, missing planner artifacts before coder entry).
3. Poll `GET /api/episodes/{episode_id}` until next transition is visible.
4. Verify:
   - Episode does not immediately fail solely due to first entry rejection.
   - Traces contain entry-validation failure payload (`node`, `disposition=reroute_previous`, `reason_code`).
   - Target node execution is skipped for rejected turn.

## Scenario B: Integration fail-fast behavior

1. Start controller with `IS_INTEGRATION_TEST=true`.
2. Trigger the same invalid-entry condition as Scenario A.
3. Poll `GET /api/episodes/{episode_id}`.
4. Verify:
   - Episode transitions to `FAILED` in one graph turn after validation failure.
   - No loopback retry chain occurs.
   - Validation telemetry includes `disposition=fail_fast` and explicit reason details.

## Scenario C: Reviewer gate on entry

1. Trigger reviewer entry with stale/missing `.manifests/review_manifest.json`.
2. Poll episode status and traces.
3. Verify:
   - Reviewer node execution is blocked on entry.
   - Failure reason references handover invariant violation.
   - Integration mode still fails fast, non-integration mode reroutes per mapping.

## Suggested commands

```bash
# Run targeted integration file (planned for this feature)
./scripts/run_integration_tests.sh tests/integration/architecture_p0/test_node_entry_validation.py

# Run full P0 suite
./scripts/run_integration_tests.sh -m integration_p0
```

## Expected observability signals

- Episode status transitions visible through `/api/episodes/{episode_id}`.
- Persisted traces/events with entry validation failure metadata.
- Validation logs include deterministic reason codes and reroute target (if applicable).
