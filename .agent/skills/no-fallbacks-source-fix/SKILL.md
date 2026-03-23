---
name: no-fallbacks-source-fix
description: Enforce fail-closed behavior by removing permissive fallbacks and fixing root causes at source, especially for planner/handover/eval validity gates.
---

# No Fallbacks, Fix At Source

Use this skill when:

- outputs look "successful" but are semantically invalid
- code uses synthetic/default artifacts to pass gates
- status transitions rely on loose string heuristics or permissive fallback paths
- dataset/eval rows must only include solid, spec-valid outputs

## Non-negotiable policy

1. Do not generate synthetic artifacts to "recover" failed model execution.
2. Do not infer success from free-text hints when structured fields exist.
3. Invalid or incomplete outputs must fail closed, not progress state.
4. Fix the root cause where it is produced, not downstream with compatibility shims.
5. Keep strict boundary contracts from `specs/desired_architecture.md` and `specs/integration-tests.md`.

## Required implementation pattern

1. Validate required artifacts and schema before status transitions.
2. Add semantic checks where schema-only checks are insufficient (prompt-intent alignment).
3. Record explicit validation errors in logs/metadata for debugging.
4. Route invalid outputs to `FAILED` (or equivalent), never to "planned/completed".
5. Remove legacy/permissive routing branches that bypass structured decisions.

## Red flags to remove

- "Fallback bundle" writers that fabricate `plan.md` / `todo.md` / YAML outputs.
- String contains checks like `"APPROVED" in feedback` when typed decisions exist.
- Defaulting unknown agent types to another agent.
- Silent session-id/default context fallbacks that can leak/isolate incorrectly.

## Validation before finishing

1. Re-run integration flows through `./scripts/run_integration_tests.sh` (targeted then broader marker).
2. Confirm invalid outputs now fail and no longer transition to success-like states.
3. Confirm valid flows still pass.
4. If invalid data was already generated, clean it from eval datasets/DB artifacts.

**NOTE**: If I interrupted you during execution with pointing to this skill, it means that I simply spotted a relevant bug in your code. After you are done fixing it, proceed with fixing whatever you were fixing earlier.
