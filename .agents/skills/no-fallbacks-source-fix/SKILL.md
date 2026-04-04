---
name: no-fallbacks-source-fix
description: Enforce fail-closed behavior by removing permissive fallbacks and fixing root causes at source, especially for planner/handover/eval validity gates and deterministic derived-field checks.
---

# No Fallbacks, Fix At Source

Use this skill when:

- outputs look "successful" but are semantically invalid
- code uses synthetic/default artifacts to pass gates
- status transitions rely on loose string heuristics or permissive fallback paths
- dataset/eval rows must only include solid, spec-valid outputs

Schema-valid but deterministically wrong outputs are still failures.

## Non-negotiable policy

1. Do not generate synthetic artifacts to "recover" failed model execution.
2. Do not infer success from free-text hints when structured fields exist.
3. Invalid or incomplete outputs must fail closed, not progress state.
4. Fix the root cause where it is produced, not downstream with compatibility shims.
5. Keep strict boundary contracts from `specs/desired_architecture.md` and `specs/integration-tests.md`.
6. Any value that can be deterministically derived from schema, enums, numeric constraints, arithmetic, or other source fields must be recomputed or checked at node entry and exit; mismatches fail closed.

## Required implementation pattern

1. Validate required artifacts, schema, and deterministic derived fields before status transitions.
2. Add semantic checks where schema-only checks are insufficient (prompt-intent alignment).
3. When removing deprecated fallback behavior, raise an explicit `ValueError` with a message such as `deprecated functionality removed: <what was removed>` or similarly direct wording.
4. Record explicit validation errors in logs/metadata for debugging.
5. Iterate with targeted debugging and source-level fixes until the issue is actually resolved; do not stop at the first partial mitigation.
6. Route invalid outputs to `FAILED` (or equivalent), never to "planned/completed".
7. Remove legacy/permissive routing branches that bypass structured decisions.
8. When a deterministic check fails, record both the source values and recomputed values in logs/metadata; do not weaken the gate to keep a run moving.

## Red flags to remove

- "Fallback bundle" writers that fabricate `plan.md` / `todo.md` / YAML outputs.
- String contains checks like `"APPROVED" in feedback` when typed decisions exist.
- Defaulting unknown agent types to another agent.
- Silent session-id/default context fallbacks that can leak/isolate incorrectly.
- Accepting approximate numeric strings or inferred enum values when exact structured values exist.

## Validation before finishing

1. Re-run integration flows through `./scripts/run_integration_tests.sh` (targeted then broader marker).
2. Confirm invalid outputs now fail and no longer transition to success-like states.
3. Confirm valid flows still pass.
4. If the first fix does not hold up under debugging or integration validation, continue iterating until the root cause is fixed.
5. If invalid data was already generated, clean it from eval datasets/DB artifacts.
6. Confirm deterministic derived-field checks fail invalid inputs and pass valid inputs without loosening the contract.

**NOTE**: If I interrupted you during execution with pointing to this skill, it means that I simply spotted a relevant bug in your code. After you are done fixing it, proceed with fixing whatever you were fixing earlier.
