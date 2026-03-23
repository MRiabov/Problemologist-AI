# Code Review: Evaluation Failures Investigation - March 3, 2026 (Round 2)

## Scope

Cross-check of `code-reviews/march-3-review-round-1.md` against current code and `logs/manual_run/*` evidence.

## Verdict

Partially matches.

- **Confirmed from Round 1:**
  - DSPy node failures/timeouts are real and frequent.
  - Validation strictness is rejecting many runs (`plan.md`, `todo.md`, `assembly_definition.yaml`).
  - Workspace initialization overwrite risk exists in `initialize_agent_files` path-check logic.
- **Not fully captured in Round 1:**
  - A concrete **policy/init conflict**: `engineer_coder` init writes `plan.md` while policy denies `plan.md` writes.
  - A concrete **tooling bug**: worker `grep` fails due to `ProtocolFileInfo` treated as dict.
  - Eval routing choices in `run_evals.py` (review/electronics/skill mapped to `engineer_coder`) amplify failures.

## Evidence-backed findings (ordered by impact)

### 0. Read-visibility leak in `list_files`/`grep` (critical)

- `list_files` enforces read policy only on the requested directory path, not on each returned entry.
- If role allows reading `"/"` or `"."`, the result can include filenames/dirs outside intended readable scope.
- Worker `ls("/")` merges local root and mounted dirs without per-entry policy filtering.
- `grep` without explicit `path` checks `"."` once, then searches broadly.

Evidence:

- `controller/middleware/remote_fs.py` (`list_files` checks only `path`; `grep` checks `.` when path missing)
- `shared/workers/filesystem/router.py` (`ls` returns merged root contents and mount points without policy filtering)
- `config/agents_config.yaml` (many roles allow read on `"/"` and `"."`)
- `specs/desired_architecture.md` (requires every read/write op be policy-checked with default deny semantics)

### 1. Policy/init conflict for `engineer_coder` (critical)

- `initialize_agent_files` writes engineer templates including `plan.md` for `engineer_coder`.
- Policy denies `plan.md` writes for coder roles.
- Logs show repeated permission denials during init.

Evidence:

- `controller/agent/initialization.py` (engineer mappings include `plan.md`)
- `config/agents_config.yaml` (coder deny includes `plan.md`)
- `logs/manual_run/controller.log` (`filesystem_permission_denied agent=engineer_coder action=write path=plan.md`)

### 2. DSPy provider errors + 180s timeout (critical)

- Frequent `litellm.BadRequestError` and `*_dspy_timeout` cause retries and terminal failure.
- Timeout is hardcoded via settings path into `asyncio.wait_for(...)`.

Evidence:

- `controller/agent/nodes/base.py` (timeout/retry loop)
- `logs/manual_run/controller.log` (`planner_dspy_failed`, `plan_reviewer_dspy_timeout`, `electronics_planner_dspy_failed`)

### 3. Validation gate failures are real and repeated (high)

- `plan_md_missing_sections`, template placeholders in TODO, incomplete `assembly_definition.yaml` (missing required fields like `constraints`, `totals`).

Evidence:

- `shared/workers/markdown_validator.py`
- `worker_heavy/utils/file_validation.py`
- `logs/manual_run/controller.log` validation error entries

### 4. Worker grep bug causes tool-path instability (high)

- `grep_raw` in local backend indexes `ProtocolFileInfo` like dict (`f_info["is_dir"]`, `f_info["path"]`) although it is model/object-like.
- Log shows `'FileInfo' object is not subscriptable`.

Evidence:

- `shared/workers/filesystem/backend.py`
- `logs/manual_run/worker_light.log`

### 5. Round-1 finding #4 is valid but partially misattributed (medium)

- Overwrite bug is real because existence check compares `"benchmark_definition.yaml"` against listed `"/benchmark_definition.yaml"`.
- But in eval path for `electronics_engineer`, current `run_evals.py` routes request as `agent_name="engineer_coder"`; this does not directly include template objective write in that branch.

Evidence:

- `controller/agent/initialization.py` (existence check and mappings)
- `shared/workers/filesystem/backend.py` (listed path format)
- `evals/run_evals.py` (request agent mapping)

## Match vs Round 1 summary

- **Matches:** Findings #2, #3, #4 are directionally correct.
- **Partial mismatch:** Finding #1 is now less central than policy/init mismatch in this run; role mapping alone is not the primary blocker observed in logs.
- **Missing in Round 1:** Grep tool bug and explicit policy/init contradiction.

## Recommended priority (no logic edits applied in this review)

1. Resolve policy/init contradiction for coder initialization writes.
2. Fix worker grep type bug.
3. Stabilize DSPy failure mode (provider errors/timeouts) and then re-run evals.
4. Tighten eval routing per role intent after above blockers are removed.
5. Add an integration test for pre-seeded-file preservation during initialization.
