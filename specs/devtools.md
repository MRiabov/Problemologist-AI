# Developer instrumentation

## Scope summary

- This file documents the developer-facing bootstrap, orchestration, validation, and artifact-regeneration surface that supports local development and eval/debug workflows.
- It is adjacent to, not a replacement for, `specs/architecture/**`.
- Use `specs/integration-test-rules.md` for integration-test boundary rules, `specs/dataset-generation.md` and `specs/architecture/evals-architecture.md` for eval semantics, and `specs/architecture/agents/agent-harness.md` for the workspace contract.
- The contract here is operational: how developers reproduce the stack, inspect seeded workspaces, run integration slices, and regenerate derived artifacts.

## Ownership model

The developer instrumentation layer is split into a small set of canonical entrypoints and a larger set of internal helpers.

| Layer | Canonical files | Responsibility | Stability |
| -- | -- | -- | -- |
| Local bootstrap | `scripts/env_up.sh`, `scripts/env_down.sh` | Bring the selected local stack profile up and down, including infra, app processes, and profile-scoped cleanup | Public and stable |
| Integration orchestration | `scripts/run_integration_tests.sh`, `scripts/internal/integration_runner.py` | Run the canonical integration suite through the real stack and the real HTTP/system boundaries | Public wrapper, internal implementation |
| Eval orchestration | `dataset/evals/run_evals.py`, `evals/logic/runner.py`, `dataset/evals/materialize_seed_workspace.py` | Run evals, materialize seeded workspaces, and expose the Codex-debug path | Public wrapper plus internal implementation |
| Eval coordination | `scripts/internal/eval_run_lock.py`, `scripts/internal/eval_seed_renders.py` | Serialize eval runs and regenerate deterministic seed render bundles | Internal helper modules |
| Seed and fixture validation | `scripts/validate_eval_seed.py`, `scripts/validate_integration_mock_response_preflight.py`, `scripts/normalize_integration_mock_responses.py` | Validate seeded eval rows, validate integration mock-response scenarios, and repair deterministic fixture drift | Public maintenance utilities |
| Derived artifact regeneration | `scripts/generate_openapi.py`, `scripts/persist_test_results.py` | Regenerate API schemas and persist test-history outputs | Public utilities |
| Compatibility and environment helpers | `scripts/ensure_docker_vfs.sh`, `scripts/ensure_ngspice.sh`, `scripts/cleanup_local_s3.py` | Make the local stack runnable in constrained environments and clear object-store state before runs | Public support scripts |
| Experimental probes | `scripts/experiments/**` | Measure or compare runtime behavior without defining the stable contract | Non-contractual |

## Local bootstrap

`scripts/env_up.sh` and `scripts/env_down.sh` are the paired local-environment controls.

### `scripts/env_up.sh`

- `scripts/env_up.sh` is the canonical local bootstrap command.
- The default profile is `integration`; `--profile eval` switches to the eval stack profile.
- The script loads `.env` when present, exports the selected profile, and reuses the shared stack-profile helper from `evals.logic.stack_profiles`.
- It stops the selected profile first, then starts infra and application services.
- It starts the containerized infra stack from `docker-compose.test.yaml`, runs migrations, launches the local controller and workers, and starts the renderer in Docker.
- It also starts the frontend dev server only when the port is available and the stack profile asks for it.
- It performs local compatibility setup before bootstrapping services, including Docker VFS readiness and `ngspice` availability checks.

### `scripts/env_down.sh`

- `scripts/env_down.sh` is the canonical teardown command for the selected profile.
- It reads the same profile selection as `env_up.sh` and stops only the processes and containers associated with that profile.
- It removes the saved PID files for host processes and stops the renderer container explicitly.
- It tears down the infra stack with `docker compose down -v --remove-orphans`.
- It clears root log symlinks when the stack is configured to create them.

### Local bootstrap support scripts

- `scripts/ensure_docker_vfs.sh` exists to keep Docker usable in environments where the default storage driver is not reliable.
- `scripts/ensure_ngspice.sh` keeps electronics validation runnable before the stack starts.
- `scripts/cleanup_local_s3.py` clears object-store state before a run so prior artifacts do not leak into the next session.

## Integration orchestration

The integration runner is a real-stack boundary contract, not a synthetic test harness.

### Public entrypoint

- `scripts/run_integration_tests.sh` is the only supported public entrypoint for integration verification.
- The shell wrapper sets the integration defaults and then hands off to `scripts/internal/integration_runner.py`.
- It keeps browser-fixture disabling, backend-error early stop, ordered marker slicing, and async cleanup as runner-level behavior rather than test-level improvisation.
- It should remain the human-facing command that other docs reference.

### Runner implementation

- `scripts/internal/integration_runner.py` owns lock acquisition, run-state persistence, service startup, health checks, log archiving, backend-error early stop, pytest execution, and cleanup.
- It treats the integration stack as a single serialized shared resource guarded by `/tmp/problemologist-integration.lock`.
- It records the requested run in `/tmp/problemologist-integration.run.json` so a blocked caller can decide whether to queue or reuse the active logs.
- It writes run logs under `logs/integration_tests/runs/run_*` and updates `logs/integration_tests/current/` to point at the active run.
- It keeps xdist enabled by default and only opts out when the caller explicitly requests a different pytest distribution mode.
- It can split the default suite into deterministic marker buckets so P0/P1 coverage is reached before the broader slices.
- It performs backend-error early stop when enabled and only falls through to full-duration runs when the errors are allowlisted or the gate is disabled.
- It drives the frontend build only when the selected pytest scope actually needs it.

### Integration run state

The integration runner owns the following persistent surfaces:

- `/tmp/problemologist-integration.lock`
- `/tmp/problemologist-integration.run.json`
- `logs/integration_tests/current/`
- `logs/integration_tests/current/json/`
- `logs/integration_tests/runs/run_*`
- the compatibility symlinks under `logs/integration_tests/` and `logs/integration_tests/json/`
- `test_output/junit.xml`
- `test_output/junit_slices/`

### Integration rules carried by the scripts

- Integration traffic stays on the real HTTP/system boundary.
- Integration runs do not become unit tests with mocks dressed up as end-to-end coverage.
- The runner should fail closed on startup, lock contention, or backend errors rather than silently degrade to a weaker execution path.
- Simulation-heavy slices follow the backend-matrix rules from `specs/integration-test-rules.md`; the script layer is responsible for honoring the selected backend, not for inventing its own matrix.

## Eval orchestration

The eval tooling mirrors the integration tooling, but it owns a separate lock, a separate log tree, and a separate workspace-inspection path.

### `dataset/evals/run_evals.py`

- `dataset/evals/run_evals.py` is a compatibility shim.
- Its only job is to add the repo root to `sys.path` and call `evals.logic.runner.run_cli()`.
- The file should stay thin so historical invocations remain valid without turning the shim into orchestration logic.

### `evals/logic/runner.py`

- `evals/logic/runner.py` is the real eval runner.
- It owns backend selection, task filtering, complexity-level filtering, concurrency, rate limiting, and run logging.
- It serializes eval runs through `/tmp/problemologist-eval.lock` and `/tmp/problemologist-eval.run.json`.
- It writes logs under `logs/evals/runs/run_*` and maintains `logs/evals/current/` plus `logs/evals/latest.log`.
- It can run against the controller-backed path or the local Codex path.
- Controller-backed eval runs bootstrap the `eval` profile through `scripts/env_up.sh` unless the caller explicitly skips that step.
- Codex-backed eval runs remain local to the materialized workspace and do not require the controller/worker stack to be booted for the agent loop itself.

### `dataset/evals/materialize_seed_workspace.py`

- `dataset/evals/materialize_seed_workspace.py` is an inspection helper for a single seeded eval row.
- It materializes the row into a temp workspace, writes the Codex prompt to `prompt.md`, and prints the copied file list for inspection.
- It is not the eval runner and should not be treated as the owner of the eval loop.
- It can optionally bootstrap the `eval` profile, launch Codex, or open the Codex UI.
- It uses the shared eval lock when it needs to coordinate with an active eval run.
- It exposes explicit sandbox selection through `--yolo` and `--no-yolo`; the script should never invent a hidden bypass mode.

### Eval coordination helpers

- `scripts/internal/eval_run_lock.py` is the shared lock/state helper for eval runs.
- `scripts/internal/eval_seed_renders.py` supports deterministic regeneration of seed render bundles.
- These modules are implementation details and should stay under `scripts/internal/`.

### Eval run state

The eval runner owns the following persistent surfaces:

- `/tmp/problemologist-eval.lock`
- `/tmp/problemologist-eval.run.json`
- `logs/evals/current/`
- `logs/evals/runs/run_*`
- `logs/evals/latest.log`
- `logs/evals/current/sessions/`

## Seed and fixture validation

The validation helpers are developer tooling, not product behavior.

### `scripts/validate_eval_seed.py`

- This script validates seeded eval entry contracts without running the full eval loop.
- It seeds the local workspace through the real helper path and validates the row against the current contract set.
- It can refresh deterministic seed manifests and render bundles when asked.
- It can optionally run the eval runner in judge mode after validation, using the codex backend by default.
- If `--run-judge` is requested for more than 10 selected seed rows, the script requires `-y` before it will launch the expensive judge pass.
- It must fail closed when required eval-row artifacts are missing, malformed, or no longer match the expected workspace contract.

### `scripts/validate_integration_mock_response_preflight.py`

- This script validates `tests/integration/mock_responses/*.yaml` and the referenced payload files against real node-entry and handoff contracts.
- It replays scenario entries cumulatively rather than trusting a single fixture file in isolation.
- It is stricter than the fixture normalizer and should be treated as a contract gate, not a convenience parser.
- It is intentionally happy-path oriented for the current `INT-###` corpus and should not guess at future negative-test semantics.

### `scripts/normalize_integration_mock_responses.py`

- This utility rewrites deterministic derived fields in integration mock-response fixtures.
- It is allowed to repair drift, but it should not alter scenario intent or narrative text.
- It is a consistency tool for checked-in fixtures, not a benchmark for agent behavior.

## Derived outputs

These scripts own a few generated or persisted artifacts that should be treated as outputs, not hand-edited source.

### OpenAPI generation

- `scripts/generate_openapi.py` regenerates `controller_openapi.json` and `worker_openapi.json`.
- The script exists so API and UI contract changes can be refreshed from the live FastAPI app definitions.
- When controller or worker routes change, this regeneration step belongs in the same change set.

### Test history persistence

- `scripts/persist_test_results.py` reads `test_output/junit.xml` and appends the parsed results to test history.
- It maintains the rolling history in `test_output/integration_test_history.json`.
- It archives older runs in `test_output/integration_test_archive.json`.
- It is post-run bookkeeping, not a substitute for the integration runner.

## Legacy and experimental helpers

- `scripts/run_agent.sh` is a legacy container wrapper and is not the canonical agent runtime path.
- `scripts/experiments/**` contains measurement and probe scripts for renderer behavior, simulation reuse, and related investigations.
- These files are useful when they are referenced by a contract or experiment, but they do not define stable production behavior by themselves.

## Edit rules

- Add new developer-facing entrypoints here when they become stable enough that other docs should route to them.
- Keep compatibility shims thin; move orchestration into the underlying Python module instead of growing the shell wrapper indefinitely.
- Keep internal helper modules under `scripts/internal/` unless they become public, documented entrypoints.
- Prefer fail-closed behavior over silent fallback for missing dependencies, missing artifacts, or missing locks.
- If a script starts owning a new persistent output path, document that path here in the same change.

## Non-goals

- This file does not replace the architecture docs.
- This file does not restate the product acceptance rules already covered by `specs/integration-test-rules.md` and `specs/architecture/**`.
- This file does not turn every ad hoc maintenance script into a stable contract; only the tooling that others are expected to use repeatedly belongs here.
