---
work_package_id: "WP06"
title: "Temporal Orchestration"
lane: "planned"
dependencies: ["WP05", "WP04"]
subtasks: ["T027", "T028", "T029", "T030", "T031"]
---

## WP06: Temporal Orchestration

**Goal**: Move long-running tasks (Simulation) to Temporal workflows.

## Context

Simulations are slow. Worker containers might die. Temporal ensures durability.

## Subtasks

### T027: Temporal Service

Update `docker-compose.yml`:

- Add `temporal` server image.
- Add `temporal-ui`.
- Configure postgres for temporal.

### T028: Temporal Controller Worker

File: `src/controller/worker.py` (Temporal Worker, not our "Node Worker")

- Initialize a Temporal Worker that listens to the `simulation-task-queue`.
- Register Activities and Workflows.

### T029: Definition

File: `src/controller/workflows/simulation.py`

**Activity**: `ExecuteSimulationActivity`

- Input: `compound_json`
- Logic: Calls `WorkerClient.execute` running the `simulate` util.
- Timeout: 5 minutes.

**Workflow**: `SimulationWorkflow`

- Logic: Calls Worker to compile MJCF -> Runs Simulation -> Renders Video -> Uploads to S3 -> Updates Postgres Trace.
- Retry Policy: Standard Temporal retry for transient failures.

### T030: Implement `ScriptExecutionWorkflow`

File: `src/controller/workflows/execution.py`

- The coding agent runs scripts that can last minutes. Wrap the entire execution in a Temporal Workflow.
- If the Controller restarts, it resumes waiting for the Worker's `ExecuteResponse`.

### T031: Update `utils.simulate()` Trigger

- Ensure `utils.simulate()` in the Worker communicates with the Controller to start/join a `SimulationWorkflow`.
- The Controller exposes an internal endpoint for this.

## Verification

1. Start Temporal.
2. Trigger a script execution.
3. Kill Controller.
4. Restart Controller.
5. Verify script completion is eventually recorded.
