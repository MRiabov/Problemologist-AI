---
name: agent-pipeline-debugging-workflow
description: Debug and fix agent-pipeline failures by running minimal-scope evals through dataset/evals/run_evals.py (single agent, single task), inspecting logs/manual_run plus eval runner logs, and implementing root-cause fixes aligned with specs/desired_architecture.md, specs/business-usecase.md, specs/integration-tests.md, and specs/dataset-generation.md.
---

# Agent Pipeline Debugging Workflow

Use this workflow to make failing agent evals pass with the smallest reproducible run.

## Non-negotiable rules

1. Never debug with multiple agents (`--agent all` is forbidden for triage).
2. Always reproduce with one agent and one task first (`--task-id`, `--limit 1`, `--concurrency 1`).
3. Run evals through `uv run dataset/evals/run_evals.py`.
4. Use standard `run_evals.py` startup (it already runs env setup); avoid separate `./scripts/env_up.sh` for normal eval triage.
5. Fix root causes in prompts/code/environment; do not add permissive fallbacks that hide failures.
6. Validate fixes against intended architecture/business behavior, not just current implementation behavior.

## Minimal run setup

1. Select one agent and one seed task (normally supplied by user/failure context):

```bash
AGENT=engineer_planner
TASK_ID=$(jq -r '.[0].id' "dataset/data/seed/role_based/${AGENT}.json")
echo "$AGENT $TASK_ID"
```

2. Run exactly that case:

```bash
uv run dataset/evals/run_evals.py \
  --agent "$AGENT" \
  --task-id "$TASK_ID" \
  --limit 1 \
  --concurrency 1 \
  --log-level INFO
```

3. For richer trace output during triage, add:

```bash
--verbose --log-level DEBUG
```

## Canonical debug commands

- Preferred log-reading method:
  Use a subagent to inspect logs and return concise evidence (`cause`, `proof lines`, `next fix`), instead of raw full-log reads in the main agent context.

- Primary agent log location:
  Agent pipeline logs/traces are in `logs/manual_run/` (`controller.log`, `worker_light.log`, `worker_heavy.log`, `temporal_worker.log`). Agent-level orchestration traces are in `logs/manual_run/controller.log`.

- List available task IDs for one agent:

```bash
jq -r '.[].id' dataset/data/seed/role_based/engineer_planner.json
```

- Fast sidecar sanity check (single use case):

```bash
uv run dataset/evals/run_evals.py \
  --agent skill_agent \
  --task-id sk-001-sidecar-skill \
  --limit 1 \
  --concurrency 1
```

- Extract key failure lines from eval runner logs:

```bash
rg "eval_trigger_failed|controller_request_failed|eval_failed|eval_timeout|eval_failed_missing_traces|overall_summary" logs/evals/run_evals.log
```

- Inspect service logs for the same timestamp window:

```bash
tail -n 200 logs/manual_run/controller.log
tail -n 200 logs/manual_run/worker_light.log
tail -n 200 logs/manual_run/worker_heavy.log
tail -n 200 logs/manual_run/temporal_worker.log
```

## Iterative debug directive

When asked to debug pipeline failures, keep iterating this loop until pass (or a real blocker is found):

1. Reproduce on one agent + one task.
2. Classify the failure from logs.
3. Map observed behavior to expected behavior in specs.
4. Implement the smallest source-level fix.
5. Re-run the same single case.
6. After pass, widen one notch (`--limit 2` for the same agent).

## Triage classification

- Environment/bootstrap failure:
  `controller_unreachable`, `worker_unreachable`, dependency/import/startup errors.
- API/orchestration contract failure:
  `eval_trigger_failed`, wrong status transitions, bad request/response shape.
- Trace/flow integrity failure:
  `eval_failed_missing_traces`, missing required agent trace despite completion-like state.
- Timeout/stall failure:
  `eval_timeout`, repeated `RUNNING` without meaningful progress.
- Prompt/behavior failure:
  pipeline executes but artifacts are invalid/incomplete relative to requirements.
- Architecture gap:
  current code path is over-complex, contradictory, or missing logic needed by intended behavior.

## Architecture and business alignment checks

Use these files as the contract hierarchy for decisions:

1. `specs/desired_architecture.md` (source-of-truth behavior)
2. `specs/business-usecase.md` (business-level success criteria)
3. `specs/integration-tests.md` (hard behavioral gates and INT mapping)
4. `specs/dataset-generation.md` (seed lineage, batch/eval intent)

If integration expectations and architecture text conflict, document the inconsistency and fix toward the intended product behavior, not accidental implementation quirks.

## Prompt vs code vs environment decision rule

- Fix environment when services or dependencies fail before agent logic runs.
- Fix code/orchestration when state transitions, contracts, or trace semantics are wrong.
- Fix prompts when orchestration is healthy but outputs are consistently low-quality or malformed.
- If success requires fallback-heavy behavior, the source bug is still unresolved.

## Validation checklist before finishing

- Target run passes with one agent, one task, `--limit 1`, `--concurrency 1`.
- Relevant logs in `logs/manual_run/` and `logs/evals/run_evals.log` are consistent with a clean pass.
- Fix aligns with `desired_architecture` and business use case.
- No permissive fallback introduced to mask failures.
- At least one adjacent case for the same agent passes (`--limit 2` or another explicit `--task-id`).

## Reference files

- `dataset/evals/run_evals.py`
- `scripts/env_up.sh` (called by `run_evals.py` unless `--skip-env-up` is explicitly used)
- `logs/manual_run/`
- `logs/evals/run_evals.log`
- `specs/desired_architecture.md`
- `specs/business-usecase.md`
- `specs/integration-tests.md`
- `specs/dataset-generation.md`
