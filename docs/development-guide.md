# Problemologist-AI - Development Guide

**Date:** 2026-03-20

## Prerequisites

| Tool | Why It Is Needed |
| --- | --- |
| Python 3.12 | Runs the controller, workers, scripts, and tests |
| `uv` | Installs and runs the Python environment |
| Docker or Podman | Starts PostgreSQL, MinIO, Temporal, and the service containers |
| Node.js and npm | Only needed for frontend work or OpenAPI client regeneration |

## Local Environment

| Task | Command | Notes |
| --- | --- | --- |
| Start the full local stack | `./scripts/env_up.sh` | Starts infra, applies migrations, launches controller, both workers, Temporal workers, and the frontend when the port is free |
| Stop the full local stack | `./scripts/env_down.sh` | Shuts down processes, stops containers, and clears local object-store state |
| Run the canonical integration suite | `./scripts/run_integration_tests.sh` | Uses the project runner and real HTTP boundaries |
| Start only the frontend | `cd frontend && npm run dev` | Optional Vite dev server for UI work |
| Build the frontend | `cd frontend && npm run build` | Optional type-check and production build for UI work |

## Service Ports

| Service | Local Port | Container Port |
| --- | --- | --- |
| Controller | 18000 | 8000 |
| Worker Light | 18001 | 8001 |
| Worker Heavy | 18002 | 8002 |
| PostgreSQL | 15432 | 5432 |
| MinIO S3 API | 19000 | 9000 |
| MinIO Console | 19001 | 9001 |
| Temporal | 17233 | 7233 |
| Temporal UI | 18081 | 8080 |
| Frontend dev server | 15173 | n/a |

## Testing

| Area | Command | Notes |
| --- | --- | --- |
| Integration suite | `./scripts/run_integration_tests.sh` | Primary verification path for backend behavior |
| Single test module | `./scripts/run_integration_tests.sh tests/integration/<file>.py` | Use for focused debugging |
| Frontend tests | `cd frontend && npm run test` | Optional Vitest-based UI checks |
| Frontend build check | `cd frontend && npm run build` | Only needed when changing shared UI contracts |

The repository policy favors integration tests over unit tests for product behavior. New controller, worker, or simulation changes should be validated against the real HTTP services and not against mocked internal calls.

## Backend-First Workflow

| Change Area | Practical Rule |
| --- | --- |
| Dataset-generation and benchmark workflow changes | Start with the backend reference and the integration suite |
| Agent graphs | Keep planner, reviewer, and coder gating strict and update the matching tests |
| Worker-light | Preserve session isolation and read-only mount behavior |
| Worker-heavy | Preserve single-flight admission and the validation-preview split |
| Shared schemas | Update the runtime models and any generated client used by the UI |
| Render policy | Update `config/agents_config.yaml` and the inspection paths together |
| UI changes | Regenerate the frontend OpenAPI client and verify the dashboard pages |

## Configuration Files

| File | Purpose |
| --- | --- |
| `config/agents_config.yaml` | Agent permissions, render policy, execution limits, and visual inspection rules |
| `config/prompts.yaml` | System prompt templates for the controller agent graphs |
| `config/manufacturing_config.yaml` | Global material and workbench cost model |
| `config/generator_config.yaml` | Benchmark-generation tuning |
| `config/reward_config.yaml` | Reward shaping and eval metric weights |
| `config/lint_config.yaml` | Linting policy |

## Common Debugging Locations

| Symptom | First Places to Check |
| --- | --- |
| Planner or reviewer handoff failure | `controller/agent/node_entry_validation.py`, `controller/agent/review_handover.py`, `worker_heavy/utils/file_validation.py` |
| Filesystem permission failure | `config/agents_config.yaml`, `controller/middleware/remote_fs.py`, `shared/agents/config.py` |
| Simulation failure | `worker_heavy/runtime/simulation_runner.py`, `worker_heavy/simulation/`, `worker_heavy/api/routes.py` |
| UI does not show traces or assets | `controller/api/routes/episodes.py`, `frontend/src/context/EpisodeContext.tsx` |
| Backup or persistence issue | `controller/api/ops.py`, `controller/persistence/`, `docker-compose.yml` |
