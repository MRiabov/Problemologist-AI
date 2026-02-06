---
work_package_id: "WP06"
title: "Temporal Orchestration"
lane: "planned"
dependencies: ["WP05", "WP04"]
subtasks: ["T027", "T028", "T029", "T030", "T031"]
---

# WP06: Temporal Orchestration

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

- Logic: Executes the Activity.
- Retry Policy: Retry on network errors, potentially on container crash.

### T030: Update Utils Trigger

The Agent tools (WP04) need to trigger this workflow.
Update `exec_tool` or create a specific `simulate_tool`?
Ideally, the *Agent* calls `utils.simulate()` in python. That python script runs in the *Worker*.
The *Worker* executes the script synchronously.
**Problem**: If the script takes 10 mins, the HTTP request times out.
**Solution**: The `simulate` util (WP05) should probably *not* run the sim locally if it's huge, OR (`deepagents` style) the `runtime/execute` endpoint returns a Job ID or streams logs.
**Revised Plan**:
For MVP/Spec 001, we might keep it synchronous (up to 5 mins) or rely on Temporal *wrapping* the agent step.
**Spec says**: "For every operation expected to run for more than 30 seconds, store that to Temporal."
**Implementation**:
The `Controller` should wrap the `WorkerClient.execute` call in a Temporal Activity if it detects a "Simulation" intent? No, that's magic.
Better: The `simulate(component)` util in Python code communicates back to Controller to "schedule a simulation"?
**Simpler approach**: The `simulate` util runs the simulation *in the worker* (it's powerful). The *Controller* wraps the entire `run_script` tool call in a Temporal Activity.
So when Agent calls "Execute Script", Controller starts a Workflow.
**Task**: Implement `ScriptExecutionWorkflow` in Controller.

### T031: Integration

- Update Controller Tool `exec_tool` to `start_workflow` instead of direct execution.
- Or simply ensure `WorkerClient` has a long timeout and Controller is robust.
*Decision*: Let's stick to the Spec's implication.
Task: "Update `simulate` util to trigger Temporal".
Actually, the spec says "Temporal is used to orchestrate the workers... It is not used to run or retry the agent."
So the Controller (API) should start a workflow when a request comes in?
Let's define T030 as: "Create `ScriptExecutionWorkflow` that calls Worker API, so if Controller dies, we can resume waiting for Worker."

## Verification

1. Start Temporal.
2. Trigger a script execution.
3. Kill Controller.
4. Restart Controller.
5. Verify script completion is eventually recorded.
