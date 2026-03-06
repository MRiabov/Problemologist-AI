# Data Model: Automatic Node Entry Validation

## Entity: NodeEntryValidationResult

Represents the decision returned by pre-entry validation for a target node.

### Fields

- `ok` (`bool`): Whether the target node may execute.
- `target_node` (`AgentName`): Node being validated for entry.
- `disposition` (`EntryFailureDisposition`): `allow`, `reroute_previous`, or `fail_fast`.
- `errors` (`list[NodeEntryValidationError]`): Structured failure details.
- `reroute_target` (`AgentName | None`): Deterministic previous node for runtime recovery.
- `reason_code` (`str`): Stable machine-readable summary code (for logs/tests).

## Entity: NodeEntryValidationError

Represents one failed invariant during node entry checks.

### Fields

- `code` (`str`): Stable error key (for example `missing_plan`, `stale_manifest`, `missing_artifact`).
- `message` (`str`): Human-readable detail for logs and feedback.
- `source` (`EntryValidationSource`): `state`, `artifact`, `handover`, or `policy`.
- `artifact_path` (`str | None`): Related file path when artifact-driven.

## Entity: NodeEntryContract

Defines required preconditions for a specific first-class graph node.

### Fields

- `node` (`AgentName`): Node this contract applies to.
- `required_state_fields` (`list[str]`): Mandatory in-memory state fields.
- `required_artifacts` (`list[str]`): Required files/artifacts before entry.
- `custom_check` (`str`): Named custom validator hook (for reviewer handover checks).
- `integration_failure_policy` (`EntryFailureDisposition`): Expected disposition in integration mode.

## Entity: NodeEntryFailureRecord

Persisted observability payload for a failed node-entry validation.

### Fields

- `episode_id` (`UUID | str`)
- `node` (`AgentName`)
- `disposition` (`EntryFailureDisposition`)
- `reason_code` (`str`)
- `errors` (`list[NodeEntryValidationError]`)
- `reroute_target` (`AgentName | None`)
- `integration_mode` (`bool`)
- `timestamp` (`datetime`)

## Enums

### EntryFailureDisposition

- `allow`
- `reroute_previous`
- `fail_fast`

### EntryValidationSource

- `state`
- `artifact`
- `handover`
- `policy`

## Relationships

- One `NodeEntryContract` exists per first-class orchestration node.
- Evaluating a `NodeEntryContract` produces one `NodeEntryValidationResult`.
- Failed `NodeEntryValidationResult` emits one `NodeEntryFailureRecord` and one or more `NodeEntryValidationError` rows.
- `NodeEntryFailureRecord` links to episode state transitions (`RUNNING -> FAILED` in integration mode).

## Validation Rules

- `ok=true` implies `disposition=allow`, `errors=[]`, and `reroute_target=None`.
- `disposition=reroute_previous` requires non-null `reroute_target`.
- `disposition=fail_fast` requires episode/session status transition to failed state in same execution turn.
- `errors` must be non-empty when `ok=false`.
- `target_node` and `reroute_target` must be members of `AgentName` enum.

## State Transitions

### Non-integration mode

1. Evaluate entry contract.
2. If valid: enter target node.
3. If invalid: persist `NodeEntryFailureRecord`, route to deterministic previous node.

### Integration mode

1. Evaluate entry contract.
2. If valid: enter target node.
3. If invalid: persist `NodeEntryFailureRecord`, transition to `FAILED`, stop retry loop.
