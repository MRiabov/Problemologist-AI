# Runner and Artifact Reference

## Eval entrypoints

- Use `uv run dataset/evals/run_evals.py` for the eval loop.
- Use `uv run scripts/validate_eval_seed.py` when the row might be invalid, stale, or missing artifacts.
- Use `uv run dataset/evals/materialize_seed_workspace.py` when you need the exact prompt/workspace for one row.
- Use `scripts/update_eval_seed_renders.py` only when you are intentionally refreshing deterministic render bundles.
- Use `scripts/env_down.sh` first if the bootstrap is wedged; the eval runner will bring the environment back up.
- Use `--queue` when another run already holds the eval lock.
- Use `--skip-env-up` only when the stack is already up and you are doing validation-only triage.
- Use `--run-judge --run-reviewers-with-judge` when debugging prompt behavior and reviewer checklists matter.
- Use `--full-sim` only when you need Genesis fidelity for backend-specific behavior.
- Use `--random` or multi-ID `--task-id` only after the first seed is understood.

## Public integration boundary

- Use `./scripts/run_integration_tests.sh` for the public integration boundary.
- Use `--queue` when another integration run already holds the lock.
- Use `INTEGRATION_RUNNER_VERBOSE=1` when you need orchestration detail.
- Leave `INTEGRATION_EARLY_STOP_ON_BACKEND_ERRORS=1` on by default.
- Set `INTEGRATION_EARLY_STOP_ON_BACKEND_ERRORS=0` only when you need full-duration logs after a known backend error.
- Use `--no-smoke` only when the slice really needs high-fidelity simulation.
- For simulation-facing slices, run MuJoCo first with `--fast-sim`, then Genesis with `--full-sim`.
- Keep xdist enabled unless a test has an explicit, justified reason not to.
- Keep backend-error allowlists narrow; prefer exact regexes or a test-local `@pytest.mark.allow_backend_errors(...)` rule.

## Artifact roots

- `logs/evals/runs/run_<timestamp>/`
- `logs/evals/current/`
- `logs/evals/current/sessions/`
- `logs/evals/latest.log`
- `logs/evals/hard_check_pass_rates.yaml`
- `logs/integration_tests/runs/run_<timestamp>/`
- `logs/integration_tests/current/`
- `logs/integration_tests/current/json/`
- `logs/integration_tests/json/` compatibility symlinks
- `logs/integration_tests/full_test_output.log`
- `logs/manual_run/`
- `test_output/junit.xml`
- `test_output/junit_slices/`

## Per-run evidence

- `events.jsonl`
- `readable_agent_logs.log`
- `llm-calls.jsonl`
- `preload.log`
- `backend-api.log`
- `analytics-worker*.log`
- `controller.log`
- `worker_light.log`
- `worker_heavy.log`
- `temporal_worker.log`
- `controller_errors.log` and `controller_errors.json`
- `worker_light_errors.log` and `worker_light_errors.json`
- `worker_heavy_errors.log` and `worker_heavy_errors.json`
- `temporal_worker_errors.log` and `temporal_worker_errors.json`
- `logs/integration_tests/current/json/*_errors.json`
- `backend_error_allowlisted_prefixes.json`
- `ledger/ledger.jsonl`
- `reports/report.json`
- `reports/confusion-matrix.json`
- `predictions/checklist-native.jsonl`
- `predictions/baseline.jsonl`
- `renders/current-episode/` for scratch previews
- `renders/<bundle>/render_manifest.json` and any video sidecars for published render evidence

## Triage checks

- Confirm the exact run id, agent, task id, backend, marker/file selection, and timestamp window before reading logs.
- Treat shared infra evidence as contested if another run is active.
- Treat failed seed validation as a seed/fixture defect, not as evidence that the eval runner is healthy.
- Use the narrowest failing slice first.
- After the first green slice, rerun one adjacent slice or a second explicit case.
