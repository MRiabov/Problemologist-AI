---
name: integration-tests-workflow
description: Run, debug, and implement integration tests using the project runner, HTTP-only test boundaries, exact deterministic assertions, and INT-xxx conventions. Use this skill whenever user references a test failure, or you want to test your code better.
---

# Integration Tests Workflow

Use this skill when the user asks to:

- debug or triage integration test failures
- add or update integration tests
- run integration suites or specific `INT-xxx` tests

When a response contains deterministic schema, enum, numeric, or derived fields, assert the exact structured values rather than only the shape or status.

## Non-negotiable rules

1. Run integration tests through `./scripts/run_integration_tests.sh` (not plain `pytest`).
2. Keep tests at HTTP/system boundaries only.
3. Do not mock/patch internals in integration tests (`controller.*`, `worker.*`, `shared.*`).
4. Assert via observable outputs: HTTP responses, logs, DB rows, S3 artifacts, `events.jsonl`, and exact structured values for deterministic schema, enum, numeric, or derived fields.
5. Maintain `INT-xxx` mapping and conventions from `specs/integration-tests.md`; any integration-test addition, removal, rename, or scope change must be reflected in that spec.
6. For `build123d` scripts used in tests, ensure every part has `PartMetadata`.
7. Use session IDs in the form `INT-{number}-{uuid8}`.
8. Before trusting `test_output/junit.xml` or `logs/integration_tests/*`, confirm they came from the current runner invocation; stale artifacts from aborted runs are common.
9. If you hit devops/startup instability (ports, stale containers, unhealthy services), run `scripts/env_down.sh` first, then rerun `./scripts/run_integration_tests.sh`; the runner will bring the environment back up automatically.

## System architecture for integration runs

- Infra comes from `docker-compose.test.yaml`: Postgres (`:15432`), MinIO (`:19000`), Temporal (`:17233`).
- Application services run as local Python processes: Controller (`:18000`), Worker Light (`:18001`), Worker Heavy (`:18002`), Temporal worker.
- Frontend tests use a static build served on `:15173` (not dev server).

## Required runner behavior (assume and preserve)

`./scripts/run_integration_tests.sh` is the source of truth for orchestration. It:

- sets integration env vars and loads `.env`
- ensures supporting dependencies/services are available
- starts infra containers
- runs migrations
- starts backend processes
- builds/serves frontend when needed
- runs pytest with integration markers
- persists test results and performs cleanup
- acquires a host-level exclusive lock at `/tmp/problemologist-integration.lock` before any destructive setup so concurrent runs fail closed instead of clobbering each other
- supports `--queue` to wait for the shared lock when you intentionally want serialized runs
- writes run-state metadata to `/tmp/problemologist-integration.run.json` so blocked callers can see the active test selection and decide whether to reuse `logs/integration_tests/current/`

Do not bypass this runner when executing integration tests.

## Canonical commands

- Full integration suite: `./scripts/run_integration_tests.sh`
- Marker suite: `./scripts/run_integration_tests.sh -m integration_p0`
- Single file: `./scripts/run_integration_tests.sh tests/integration/.../test_file.py`
- Single test: `./scripts/run_integration_tests.sh tests/integration/.../test_file.py::test_int_020_...`
- Marker expression: `./scripts/run_integration_tests.sh -m "integration_p0 and not integration_frontend"`
- MuJoCo-default run: `./scripts/run_integration_tests.sh ...`
- Genesis full-fidelity simulation: `./scripts/run_integration_tests.sh --full-sim ...`
- High-fidelity mode: `./scripts/run_integration_tests.sh --no-smoke`
- Disable automatic backend-error early stop for a run: `INTEGRATION_EARLY_STOP_ON_BACKEND_ERRORS=0 ./scripts/run_integration_tests.sh ...`

## MuJoCo toggle rule

Use the default MuJoCo run when debugging logic that is not about Genesis-only behavior and not about simulation fidelity itself.

- The default run uses MuJoCo (`SIMULATION_DEFAULT_BACKEND=MUJOCO`) and is often much faster for plumbing/debug loops.
- `--full-sim` switches the run to Genesis.
- Good candidates: controller/worker orchestration, filesystem/session isolation, API schema drift, manifests/handover gates, prompt/tool-call plumbing, general benchmark/engineer workflow checks that are backend-portable.
- Do not use the MuJoCo default for FEM, fluids, part breakage, Genesis-native stress reporting, dynamic wire/softbody behavior, or any backend-specific artifact behavior.
- Before opting into Genesis, check that the target test is not in the Genesis-only path:
  - inspect `tests/integration/backend_utils.py`
  - look for `skip_unless_genesis(...)`
  - check the backend matrix notes in `specs/integration-tests.md`
- If unsure whether the logic is backend-portable, stay on the default backend.

## Iterative debug directive

- If the user asks to "debug iteratively", run tests and continue debugging/fixing in a loop until the issue is fully resolved (or you hit a hard blocker that requires user input).
- For iterative triage, prefer running with higher failure capture to reduce reruns, e.g. `./scripts/run_integration_tests.sh --maxfail 10 ...` (or equivalent pytest args passed through the runner).
- For timeout-prone tests, actively inspect logs during execution and terminate early once the root cause is clear (do not wait for full timeout).
- After a targeted pass is green, widen in this order:
  1. single test
  2. adjacent tests in the same file
  3. any known prior collection/import blockers
  4. relevant marker/file subset
  5. full suite
- If the harness itself becomes unstable or non-productive, stop widening and fix the concrete blocker first.

**Suggestion**: if a test fails primarily due to long timeout (e.g. 5 minutes), please spare me the time of waiting, and check logs while the test is running. After 30 seconds it would normally already reach its failure point. Again, you can use a subagent.

### Early-stop timeout triage (save time)

Use this workflow when a test, by design, requires long timeouts (you can find test design in @specs/integration-tests.md - broadly speaking, any test that requires long polling and can't exit immediately for verification):

1. Start the narrowest run through the integration runner.
   - Example: `./scripts/run_integration_tests.sh tests/integration/...::test_int_xxx --maxfail=1`
2. In parallel, tail service logs while the test is running *using a subagent* to read the outputs (the subagent is important to not fill your context window).
   - `tail -n 200 logs/controller.log`
   - `tail -n 200 logs/worker_light.log`
   - `tail -n 200 logs/worker_heavy.log`
   - `tail -n 200 logs/temporal_worker.log`
3. As soon as failure signature is confirmed (same error path repeating, clear crash, missing artifact gate, etc.), stop the run early.
   - If attached interactively: send `Ctrl-C`.
   - If not attached: terminate the runner process (e.g. `pkill -f run_integration_tests.sh` or kill the specific pytest PID).
4. Immediately capture evidence before rerun:
   - `tail -n 200 logs/integration_tests/full_test_output.log`
   - `tail -n 200 logs/integration_tests/controller.log`
   - `tail -n 200 logs/integration_tests/worker_heavy.log`
5. Apply fix, then rerun the same narrow scope first.

Rules for early termination:

- Only terminate early after you have enough log evidence to identify a concrete failure cause.
- Keep one failing run artifact set (`logs/integration_tests/*`) for traceability before starting the next run.
- Do not switch to plain `pytest`; always rerun via `./scripts/run_integration_tests.sh`.
- Runner default: `INTEGRATION_EARLY_STOP_ON_BACKEND_ERRORS=1` in `scripts/run_integration_tests.sh`. When enabled, the runner watches dedicated backend error JSON logs and reports the first **non-allowlisted** line; pytest then fails the active test through the backend-error fixture (no runner SIGINT injection). Matching uses global `BACKEND_ERROR_ALLOWLIST_REGEXES` plus per-test `@pytest.mark.allow_backend_errors` patterns mapped by `session_id` `INT-xxx` prefixes. Use `INTEGRATION_EARLY_STOP_ON_BACKEND_ERRORS=0` to disable this detector for a run.
- The per-test prefix map for the rule above is auto-generated/cached at `logs/integration_tests/json/backend_error_allowlisted_prefixes.json`.

## Triage procedure

1. Identify failing `INT-xxx` and file/test path from output.
2. Reproduce with the smallest scope through the integration runner.
3. Inspect runtime logs:
   - `tail -100 logs/controller.log`
   - `tail -100 logs/worker_light.log`
   - `tail -100 logs/worker_heavy.log`
   - `tail -100 logs/temporal_worker.log`
   - `tail -100 logs/frontend.log` (frontend tests)
4. Classify failure type (startup/port/schema/simulation/frontend/dependency/migration).
5. Validate behavior through boundaries only (HTTP payloads, logs, DB/S3/events).
6. Implement the smallest fix that preserves system-level behavior.

Special case: collection/import drift

- If many tests fail at collection because a request/response model moved modules, first restore a compatibility export at the API boundary (`controller.api.schemas`, `controller.api.tasks`, similar boundary modules) before editing many test imports individually.
- Re-run the exact collection blockers first after adding the compatibility export.

## Log storage and run history

- Primary current-run logs live in `logs/integration_tests/`. Treat this as the canonical location.
- Machine-readable backend error logs live in `logs/integration_tests/json/` as:
  - `controller_errors.json`
  - `worker_light_errors.json`
  - `worker_heavy_errors.json`
  - `temporal_worker_errors.json`
  - `backend_error_allowlisted_prefixes.json` (autogenerated prefix allowlist cache used by early-stop)
- `logs/controller.log`, `logs/worker_light.log`, `logs/worker_heavy.log`, `logs/temporal_worker.log`, and `logs/frontend.log` are convenience symlinks to files in `logs/integration_tests/`.
- Structured backend error lines are mirrored to `logs/integration_tests/json/{controller,worker_light,worker_heavy,temporal_worker}_errors.json` as JSON Lines (one JSON object per line).
- At the start of each integration run, prior `logs/integration_tests/` contents are moved to `logs/archives/run_YYYYMMDD_HHMMSS/`.
- Old log archives are cleaned automatically (the runner removes `logs/archives/run_*` older than 24 hours).
- Structured test-run summaries are persisted separately under `test_output/`:
  - `test_output/integration_test_history.json` stores recent runs (latest 5)
  - `test_output/integration_test_archive.json` stores older runs
  - `test_output/junit.xml` stores the latest pytest junit output
- The live integration runner also keeps its current-selection metadata in `/tmp/problemologist-integration.run.json`; treat it as the source for the active run's requested INT set when deciding whether to reuse logs or queue.
- Non-mandatory suggestion: when more or less confident about an error case, use a subagent to read logs; it will help you keep your context low. The agent would output the issue causes with exact line evidence.

Note: if I'm pasting you a error log, it's very likely (95%) you'll have fresh logs in these directories. If not, they could be in logs/archives/run\_\*, but usually I would paste you more relevant output.

Again, I suggest doing triage of logs via explorer subagents to avoid you loading too much into your context and forgetting it later on.

## Common failure classes

- `Connection refused` on `:18000/:18001/:18002`: service crashed; inspect corresponding log.
- Port in use: stale prior process; terminate stale server processes.
- `422 Unprocessable Entity`: request/schema mismatch; confirm current API and migrations.
- Migration failures: DB schema drift; resolve migration state then rerun.
- Simulation timeout: use integration-friendly settings and inspect heavy worker logs.
- Frontend timeout/flakiness: static build/serve issue; inspect `logs/frontend.log`.
- `ModuleNotFoundError`: missing env/dependency activation.
- Teardown failure `Unexpected backend errors/exceptions detected in dedicated service error logs.`: inspect `logs/integration_tests/{controller,worker_light,worker_heavy,temporal_worker}_errors.log`; allow intentional noise with `@pytest.mark.allow_backend_errors` or `BACKEND_ERROR_ALLOWLIST_REGEXES` (`;;`-separated regexes). Strict mode is controlled by `STRICT_BACKEND_ERRORS` (default `1`), and this teardown gate is skipped when the test already failed in call phase.
- Early-stop parser source: `INTEGRATION_EARLY_STOP_ON_BACKEND_ERRORS` reads `logs/integration_tests/json/*_errors.json` (native structlog JSON events), not the rendered string `.log` files.
- Unexpected early stop while debugging: if the run exits after backend-error detection, it is usually because pytest failed the active test and then hit `--maxfail`. Rerun with `INTEGRATION_EARLY_STOP_ON_BACKEND_ERRORS=0` to keep pytest running and collect longer traces.
- Stale junit/log mismatch: if `test_output/junit.xml` or archived logs mention failures that do not match the live run you just launched, discard them as stale and capture fresh evidence from the active invocation.

## Writing/updating integration tests

1. Find the corresponding `INT-xxx` in `specs/integration-tests.md`.
2. Place tests in the right suite:
   - `tests/integration/architecture_p0/`
   - `tests/integration/architecture_p1/`
   - `tests/integration/evals_p2/`
   - `tests/e2e/` for frontend (`integration_frontend`)
3. Backend tests use `httpx.AsyncClient`; frontend tests use Playwright.
4. Use worker/controller endpoints and real payloads, not internal function calls.
5. Keep generous integration timeouts where needed.
6. Re-run targeted test first, then relevant marker suite.
7. For authored-script fixtures, follow the current agent contract:
   - prefer `from utils.metadata import PartMetadata, CompoundMetadata`
   - prefer `from utils.submission import validate, simulate, submit_for_review`
   - prefer `python script.py` over wrapper commands that import `script.py` via `python -c`
   - treat `build()` as compatibility-only unless the test is specifically asserting loader fallback behavior
8. Reuse `tests/integration/agent/helpers.py` for shared seeding helpers such as `seed_benchmark_assembly_definition(...)` when a test needs deterministic benchmark-setup YAML or cost/weight-cap fixtures.

## Common endpoint patterns

- Worker FS: `POST /fs/write`, `POST /fs/read`, `POST /fs/bundle`
- Worker runtime/sim: `POST /runtime/execute`, `POST /benchmark/validate`, `POST /benchmark/simulate`, `POST /benchmark/verify`
- Controller APIs: use public endpoints only for orchestration and event assertions

## Session and metadata conventions

- Session ID format: `INT-{test_number}-{uuid4().hex[:8]}`
- Include `INT-xxx` in test name/docstring.
- In `build123d` scripts used by tests, assign `PartMetadata(...)` to every part.

## Frontend integration specifics

- Frontend URL: `http://localhost:15173`
- Prefer selectors based on stable `data-testid` or explicit UI ids.
- Validate end-to-end behavior against real backend responses, asset fetches, and persisted state.
- Backend integration tests take precedence over frontend integration tests. If a backend integration test passes but a frontend integration test fails, adapt the frontend to the backend contract unless the intended product functionality genuinely requires backend edits.

## Reference files

- `specs/integration-tests.md` (test catalog and INT mapping)
- `specs/frontend-specs.md` (frontend behavior targets)
- `specs/desired_architecture.md` (system behavior source of truth)
- `scripts/run_integration_tests.sh` (execution orchestration)
- `docker-compose.test.yaml` (infra definition)
- `tests/**/conftest.py` (fixtures)
- `tests/integration/agent/helpers.py` (shared benchmark/reviewer seeding helpers, especially `seed_benchmark_assembly_definition(...)`)
- `references/integration-infra-helpers.md` (integration-only helper files, env toggles, and run artifacts)

## Validation checklist before finishing

- The failing integration test now passes through the integration runner.
- Related tests still pass for the affected area.
- No unit-style mocks leaked into integration tests.
- `INT-xxx` IDs and session naming conventions remain consistent.
