---
name: source-fix-remediation-workflow
description: Step-by-step workflow to identify fallback-driven regressions and replace them with strict source-level validation and fail-closed transitions.
---

# Source-Fix Remediation Workflow

Use this skill to execute the fix (not just policy) when fallback behavior causes invalid outputs to pass.

## Workflow

1. Reproduce with system runner
- Use `./scripts/run_integration_tests.sh` for integration behavior.
- For evals, use `dataset/evals/run_evals.py` with explicit agent/task scope.

2. Locate the transition gate
- Find where status changes (`PLANNED`, `COMPLETED`, etc.).
- Identify all conditions allowing transition.

3. Locate fallback producers
- Search for synthetic artifact writers and permissive defaults (`fallback`, `default`, heuristic string checks).
- Map: producer -> validator -> transition.

4. Tighten source contracts
- Require structured outputs from the producing node.
- Validate required artifacts and typed schema at gate time.
- Add semantic checks for prompt/intent where needed.

5. Remove permissive branches
- Delete synthetic artifact generation used to pass gates.
- Replace free-text inference with structured decision requirements.
- Convert unknown/ambiguous paths to explicit failures.

6. Preserve observability
- Add explicit error logs and validation messages with actionable detail.
- Persist reasons in status metadata for postmortem/debug.

7. Validate end-to-end
- Run targeted integration tests first, then marker suite.
- Re-run affected eval slice.
- Confirm invalid paths now fail; valid paths still pass.

8. Clean bad generated data
- Remove previously persisted invalid eval rows/artifacts derived from fallback paths.
- Keep only rows generated from valid upstream outputs.

## Editing checklist

- No fabricated `plan.md` / `todo.md` / YAML for recovery.
- No "contains APPROVED" style routing.
- No unknown-agent remap fallbacks.
- No silent default session context for runtime flows.
- Status transitions only happen after explicit validation success.
