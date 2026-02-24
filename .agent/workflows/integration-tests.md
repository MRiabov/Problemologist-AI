---
description: Guide for debugging and implementing integration tests.
---

# Integration Testing Guide

This document is the **central LLM reference** for debugging and implementing integration tests. Read this fully before writing or debugging any integration test.

## 1. Non-Negotiable Rules

> [!CAUTION]
> Violating these rules produces tests that LOOK like they pass but provide ZERO real coverage.

1. **HTTP-only traffic.** Every test action goes through HTTP endpoints. Never call Python functions directly (except importing `shared.*` models/enums for schema convenience).
2. **No internal mocks.** Do NOT use `patch`, `monkeypatch`, or fakes for `controller.*`, `worker.*`, `shared.*`. Mocks are allowed ONLY for unstable 3rd-party services (external APIs).
3. **Assert on observable boundaries.** HTTP responses, container/server logs, DB rows, S3 objects, `events.jsonl` stream.
4. **Map every test to an `INT-xxx` ID** from `specs/integration-tests.md`. Include the ID in the test name or docstring.
5. **`PartMetadata` on every part.** All `build123d` scripts must set `.metadata = PartMetadata(...)` on every part.
6. **Session IDs follow the convention:** `INT-{test_number}-{uuid[8:]}` (e.g. `INT-020-a1b2c3d4`).

## 2. Architecture — What Runs Where

```
┌─────────────────────────────────────────────────┐
│  docker-compose.test.yaml (infra only)          │
│  ┌──────────┐  ┌──────────┐  ┌──────────┐      │
│  │ Postgres │  │  MinIO   │  │ Temporal │      │
│  │  :15432  │  │  :19000  │  │  :17233  │      │
│  └──────────┘  └──────────┘  └──────────┘      │
└─────────────────────────────────────────────────┘
          ▲              ▲            ▲
          │  (host network — 127.0.0.1)
┌─────────────────────────────────────────────────┐
│  Local Python processes (started by runner)     │
│  ┌────────────┐ ┌──────────────┐ ┌────────────┐│
│  │ Controller │ │ Worker Light │ │Worker Heavy ││
│  │   :18000   │ │    :18001    │ │   :18002   ││
│  └────────────┘ └──────────────┘ └────────────┘│
│  ┌──────────────┐  ┌─────────────────────────┐ │
│  │Temporal Worker│  │Frontend (npx serve:15173)│ │
│  └──────────────┘  └─────────────────────────┘ │
└─────────────────────────────────────────────────┘
```

- **Infra** = real Docker containers (Postgres, MinIO, Temporal).
- **App servers** = local `uvicorn` processes (NOT Docker), to avoid long build times.
- **Frontend** = pre-built Vite app served statically via `npx serve -s dist -p 15173` (only started for `integration_frontend` tests).

## 3. The Test Runner — `scripts/run_integration_tests.sh`

> [!IMPORTANT]
> **Always run tests through this script.** It handles infra setup, env vars, migrations, server lifecycle, and cleanup. Running `pytest` directly will fail because servers/infra won't be running.

### Usage

```bash
# Run ALL integration tests (p0 + p1 + p2 + frontend)
./scripts/run_integration_tests.sh

# Run only P0 tests
./scripts/run_integration_tests.sh integration_p0

# Run a specific test file
./scripts/run_integration_tests.sh tests/integration/architecture_p0/test_architecture_p0.py

# Run with a specific marker expression
./scripts/run_integration_tests.sh -m "integration_p0 and not integration_frontend"

# High-fidelity simulation (disable smoke test mode)
./scripts/run_integration_tests.sh --no-smoke
```

### What the runner does (in order)

1. Sets environment (`IS_INTEGRATION_TEST=true`, ports, S3 creds, etc.)
2. Loads `.env` for API keys (OpenAI, Langfuse, etc.)
3. Runs `scripts/ensure_docker_vfs.sh` (Docker-in-Docker fix)
4. Runs `scripts/ensure_ngspice.sh` (electronics validation)
5. Populates `parts.db` if missing (COTS database)
6. Starts Docker Compose infra (`docker-compose.test.yaml`)
7. Waits for Postgres, MinIO, Temporal health
8. Runs `alembic upgrade head` (migrations)
9. Starts Xvfb (headless rendering, `DISPLAY=:99`)
10. Starts Worker Light (:18001), Worker Heavy (:18002), Controller (:18000), Temporal Worker
11. **If Playwright:** builds frontend (`npm run build:fast`), serves on :15173
12. Waits for all services to be healthy
13. Runs `pytest -v -o "addopts=-n0" --maxfail=3 -s ...`
14. Persists results via `scripts/persist_test_results.py`
15. Cleanup trap: kills all processes, tears down Docker infra

### Environment variables set by the runner

| Variable | Value | Purpose |
|---|---|---|
| `IS_INTEGRATION_TEST` | `true` | Signals integration mode |
| `POSTGRES_URL` | `postgresql+asyncpg://postgres:postgres@127.0.0.1:15432/postgres` | DB |
| `TEMPORAL_URL` | `127.0.0.1:17233` | Temporal |
| `S3_ENDPOINT` | `http://127.0.0.1:19000` | MinIO |
| `S3_ACCESS_KEY` / `S3_SECRET_KEY` | `minioadmin` | MinIO auth |
| `WORKER_URL` | `http://127.0.0.1:18001` | Worker Light |
| `WORKER_HEAVY_URL` | `http://127.0.0.1:18002` | Worker Heavy |
| `WORKER_SESSIONS_DIR` | temp dir | Shared session filesystem |
| `GENESIS_FORCE_CPU` | `1` | Avoid GPU flakiness |
| `DISPLAY` | `:99` | Xvfb for headless rendering |
| `VITE_API_URL` | `http://localhost:18000` | Frontend → Controller |

## 4. Test Organization & Markers

### Directory structure

```
tests/integration/
├── architecture_p0/     # INT-001..030, INT-053..056, INT-061..063, INT-101..128
│   ├── conftest.py      # Shared fixtures (controller_client, worker_*_client)
│   ├── scripts/         # Helper build123d scripts used by tests
│   └── test_*.py
├── architecture_p1/     # INT-031..045, INT-057..069, INT-131..141
├── evals_p2/            # INT-046..052, INT-151..156
└── frontend/            # INT-157..171 (Playwright, in tests/e2e/)

tests/e2e/               # Playwright frontend tests
├── conftest.py           # Playwright fixtures
└── test_*.py
```

### Pytest markers

| Marker | Scope | CI Gate |
|---|---|---|
| `@pytest.mark.integration_p0` | Must pass before merge to `main` | PR gate |
| `@pytest.mark.integration_p1` | Must pass nightly/pre-release | Nightly |
| `@pytest.mark.integration_p2` | Extended production-quality suite | Weekly |
| `@pytest.mark.integration_frontend` | Playwright browser tests | With frontend |

Default `pyproject.toml` excludes all integration markers from normal `pytest` runs via `addopts`.

## 5. Writing a New Integration Test

### Step-by-step

1. **Find the `INT-xxx` ID** in `specs/integration-tests.md`. Check the "Per-test Unit→Integration Implementation Map" column to understand the correct approach vs. the anti-pattern.
2. **Choose the right directory** based on priority (P0/P1/P2/frontend).
3. **Use `httpx.AsyncClient`** for backend tests (see fixtures in `conftest.py`):

   ```python
   @pytest.mark.integration_p0
   @pytest.mark.asyncio
   async def test_int_005_planner_artifact_gate():
       """INT-005: Planner handoff blocked without required artifacts."""
       session_id = f"INT-005-{uuid4().hex[:8]}"
       async with httpx.AsyncClient(timeout=300.0) as client:
           # Write files via worker API
           await client.post(
               f"{WORKER_LIGHT_URL}/fs/write",
               json={"path": "script.py", "content": "..."},
               headers={"X-Session-ID": session_id},
           )
           # Hit the endpoint under test
           resp = await client.post(f"{CONTROLLER_URL}/agent/run", ...)
           assert resp.status_code == ...
   ```

4. **For simulation tests**, use the bundle pattern:

   ```python
   # Write files to worker light, then get a gzipped workspace bundle
   bundle_resp = await client.post(
       f"{WORKER_LIGHT_URL}/fs/bundle",
       headers={"X-Session-ID": session_id}, timeout=120.0,
   )
   bundle_b64 = base64.b64encode(bundle_resp.content).decode("utf-8")
   
   # Send bundle to worker heavy for simulation
   resp = await client.post(
       f"{WORKER_HEAVY_URL}/benchmark/simulate",
       json={"script_path": "script.py", "bundle_base64": bundle_b64, "smoke_test_mode": True},
       headers={"X-Session-ID": session_id}, timeout=600.0,
   )
   ```

5. **For Playwright/frontend tests**, use synchronous Playwright API:

   ```python
   @pytest.mark.integration_frontend
   def test_int_170_feedback(page: Page):
       """INT-170: Post-run feedback UX."""
       page.set_viewport_size({"width": 1280, "height": 720})
       page.goto("http://localhost:15173", timeout=60000)
       page.evaluate("localStorage.clear()")
       page.reload()
       page.wait_for_selector('[data-testid="app-layout"]', timeout=30000)
       # ... interact with real UI against real backend
   ```

### Key patterns

- **Session IDs:** `f"INT-{number}-{uuid4().hex[:8]}"` or `f"INT-{number}-{int(time.time())}"`.
- **Timeouts:** Use generous timeouts (300s for clients, 600s for simulations). The runner sets `--maxfail=3`.
- **Build scripts:** Every `build123d` script must import and use `PartMetadata`:

  ```python
  from shared.models.schemas import PartMetadata
  part.metadata = PartMetadata(material_id="aluminum_6061", fixed=True)
  ```

- **Frontend port:** All Playwright tests use `http://localhost:15173` (static build, not dev server).
- **Worker endpoints used heavily:**
  - `POST /fs/write` — write files to session workspace.
  - `POST /fs/read` — read files from session workspace.
  - `POST /fs/bundle` — get gzipped workspace as binary.
  - `POST /runtime/execute` — run arbitrary Python code.
  - `POST /benchmark/simulate` — run simulation.
  - `POST /benchmark/validate` — validate a build script.
  - `POST /benchmark/verify` — multi-seed verification.

## 6. Debugging Failures

### Quick triage

1. **Identify the failing `INT-xxx`** from pytest output.
2. **Run just that test:**

   ```bash
   ./scripts/run_integration_tests.sh tests/integration/architecture_p0/test_architecture_p0.py::test_int_020_simulation_failure_taxonomy
   ```

3. **Check logs immediately after failure:**

   ```bash
   tail -100 logs/controller.log
   tail -100 logs/worker_light.log
   tail -100 logs/worker_heavy.log
   tail -100 logs/temporal_worker.log
   tail -100 logs/frontend.log   # if Playwright
   ```

### Common failure categories

| Symptom | Likely cause | Fix |
|---|---|---|
| `Connection refused` on :18000/18001/18002 | Server crashed on startup | Check respective log; often a missing migration or import error |
| `Temporal failed to start` | Temporal auto-setup migrations still running | Increase settle time or check Docker logs |
| Port clash (address already in use) | Stale processes from previous run | `pkill -f "uvicorn.*18000"` etc., or the script does this automatically |
| `422 Unprocessable Entity` | Schema mismatch — request body doesn't match current API | Run `alembic upgrade head`; check if models changed |
| Simulation timeout | Genesis/MuJoCo taking too long | Check `GENESIS_FORCE_CPU=1` is set; use `smoke_test_mode: True` |
| Frontend test flaky / timeout | Build stale or serve not ready | Re-run; check `logs/frontend.log`; ensure `npm run build:fast` succeeds |
| `ModuleNotFoundError` | Missing dependency or wrong venv | `source .venv/bin/activate` before running, or use `uv run` |
| `alembic` migration error | DB schema doesn't match models | `uv run alembic upgrade head` or check for conflicting migrations |

### Debugging tools

- **MinIO Console:** `http://127.0.0.1:19001` (minioadmin/minioadmin) — browse S3 artifacts.
- **Postgres:** `psql postgresql://postgres:postgres@127.0.0.1:15432/postgres` — query episodes, events.
- **Temporal UI:** `http://127.0.0.1:17280` — inspect workflow executions.
- **Events stream:** Query `GET /episodes/{id}/events` to inspect emitted events.

## 7. Reference Documents

| Document | Purpose |
|---|---|
| [specs/integration-tests.md](file:///specs/integration-tests.md) | Full `INT-xxx` catalog, required assertions, unit-vs-integration map |
| [specs/frontend-specs.md](file:///specs/frontend-specs.md) | Frontend architecture, UI components, Playwright test targets |
| [kitty-specs/desired_architecture.md](file:///kitty-specs/desired_architecture.md) | Central source of truth for the entire system |
| [scripts/run_integration_tests.sh](file:///scripts/run_integration_tests.sh) | Test runner script (always run through this) |
| [docker-compose.test.yaml](file:///docker-compose.test.yaml) | Infra services definitions |
| [tests/integration/architecture_p0/conftest.py](file:///tests/integration/architecture_p0/conftest.py) | Shared fixtures (`controller_client`, `worker_light_client`, `worker_heavy_client`) |

## 8. Frontend-Specific Testing

### Architecture

- **Tech stack:** Vite + React. TypeScript types auto-generated from controller OpenAPI schema.
- **Stability:** Frontend is built (`npm run build:fast`) and served statically on `:15173` via `npx serve -s dist`. This avoids HMR reload flakiness during tests.
- **Before Playwright runs**, the runner generates fresh OpenAPI specs (`scripts/generate_openapi.py`) and rebuilds the frontend.
- Tests use the `@pytest.mark.integration_frontend` marker and **synchronous** Playwright API (`from playwright.sync_api import Page`).

### UI structure (3-column layout)

The frontend uses a 3-column layout (~3:3:6 ratio):

1. **Sidebar** — session history.
2. **Chat UI** — prompt input, agent output, reasoning traces, tool call activity.
3. **Viewer** — CAD viewer (three.js), code viewer, simulation playback, circuit viewer (tscircuit).

### Key frontend test targets

- `[data-testid="app-layout"]` — main layout container.
- `#chat-input` — prompt input field.
- `button:has-text("CREATE NEW")` — new session button.
- `[data-panel-resize-handle]` — column resize handles.
- `[data-testid="sidebar-thumbs-up"]` / `sidebar-thumbs-down` — feedback buttons.
- `button:has-text("Send Feedback")` — feedback modal submit.
- Worker assets fetched via controller proxy (GET `/api/episodes/{id}/assets/{path}`).

### Frontend INT-xxx IDs (INT-157 to INT-171)

These cover: session history, workflow parity, plan approval, reasoning traces, tool-call activity, interrupt UX, steerability context, code viewer, CAD topology selection, simulation viewer, worker asset fetch, circuit viewer, theme toggle, feedback, and layout persistence. See `specs/integration-tests.md` for full details.
