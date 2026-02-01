---
work_package_id: WP04
title: Process Isolation & API Service
lane: "doing"
dependencies: []
subtasks:
- T013
- T014
- T015
- T016
review_status: "has_feedback"
reviewed_by: "MRiabov"
agent: "gemini"
shell_pid: "182241"
---

# WP04: Process Isolation & API Service

## Objective

Wrap the simulation in a separate process to protect the main application from agent script crashes (segfaults, infinite loops) and expose it as a web service.

## Context

Agent code is "untrusted". If it does `while True: pass`, the server would hang. We must run it in a subprocess with a timeout.

## Subtasks

### T013: Implement ProcessRunner

**Goal**: Run simulation in `multiprocessing.Process`.
**Implementation**:

- File: `src/simulation_engine/runner.py`.
- Function `run_isolated(bundle) -> Result`.
- Create Queue.
- Spawn Process targeting `_run_sim_wrapper`.
- `_run_sim_wrapper`: Reconstructs scene, runs loop, puts result in Queue.

### T014: Process Safety (Timeout/Crash)

**Goal**: Handle failures gracefully.
**Implementation**:

- `p.start()`.
- `p.join(timeout=30.0)`.
- If `p.is_alive()`: `p.terminate()`; return `TimeoutError`.
- If `p.exitcode != 0`: return `CrashError`.

### T015: Implement FastAPI Service

**Goal**: HTTP Interface.
**Implementation**:

- File: `src/simulation_engine/main.py`.
- File: `src/simulation_engine/api.py`.
- Endpoint `POST /simulate`.
- Payload: JSON/MsgPack containing geometries and script.

### T016: API Models

**Goal**: define input/output schema.
**Implementation**:

- Use Pydantic.
- `SimulationRequest`:
  - `env_stl`: bytes (base64) ? OR reference?
  - `agent_script`: str.
  - `config`: dict.
- `SimulationResponse`:
  - `outcome`: str.
  - `metrics`: dict.

## Definition of Done

- [ ] `POST /simulate` accepts request.
- [ ] Runs sim in separate PID.
- [ ] Returns 200 OK with metrics on success.
- [ ] Returns 400/500 with readable error on Script Crash or Timeout.

## Activity Log

- 2026-02-01T08:42:56Z – unknown – lane=planned – Moved to planned
- 2026-02-01T10:15:07Z – gemini – shell_pid=182241 – lane=doing – Started implementation via workflow command
