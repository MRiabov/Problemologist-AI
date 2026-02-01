---
work_package_id: WP03
title: Simulation Runtime Core
lane: "done"
dependencies: []
subtasks:
- T009
- T010
- T011
- T012
agent: "gemini-cli"
shell_pid: "128439"
reviewed_by: "MRiabov"
review_status: "approved"
---

# WP03: Simulation Runtime Core

## Objective

Implement the physics loop that steps the simulation, executes agent control code, and monitors success criteria.

## Context

The core loop:

1. `step()`
2. Get Observations (Sensor readings)
3. Call Agent `control(obs)` -> Get Action
4. Apply Action (Control inputs)
5. `mujoco.mj_step()`
6. Check termination.

## Subtasks

### T009: Implement SimulationLoop

**Goal**: The main stepping class.
**Implementation**:

- File: `src/simulation_engine/simulation.py`
- Class `SimulationLoop`.
- Init: Load `MjModel`, `MjData`.
- Method `step()`: Performs one physics tick.

### T010: Implement AgentInterface

**Goal**: standardized way to call the untrusted agent code.
**Implementation**:

- Define `AgentProtocol`.
- In `SimulationLoop`, method `run(agent_script, max_steps)`:
- Create safe scope/namespace.
- Exec `agent_script`.
- Extract `control` function.
- In loop: `action = control(obs)`.
- Apply `data.ctrl[:] = action`.

### T011: Implement MetricCollector

**Goal**: Track stats.
**Implementation**:

- Accumulate `energy`: sum(torque *velocity* dt).
- Track `time`: step_count * dt.
- Track `collisions`: `data.ncon`, iterate contacts.

### T012: Implement Termination Conditions

**Goal**: Decide when to stop (Win/Loss).
**Implementation**:

- `check_termination(data) -> Status`:
- WIN: Target Object center specific geom/site?
  - Use `data.site_xpos['goal']` vs `data.xpos['target']`.
  - Distance < threshold.
- FAIL: Collision with "Forbid" zone.
  - Check contact pairs. If (AgentGeom, ForbidGeom) in contacts -> FAIL.

## Definition of Done

- [ ] `SimulationLoop` runs without crashing.
- [ ] Agent script controls actuators.
- [ ] Metrics are captured.
- [ ] Goal/Forbid detection works reliability.

## Activity Log

- 2026-02-01T08:51:52Z – Antigravity – shell_pid=119698 – lane=doing – Started implementation via workflow command
- 2026-02-01T09:00:17Z – Antigravity – shell_pid=119698 – lane=for_review – Implemented SimulationLoop, AgentProtocol, Metrics and Termination logic. Verified with unit tests.
- 2026-02-01T09:05:23Z – gemini-cli – shell_pid=128439 – lane=doing – Started implementation via workflow command
- 2026-02-01T10:13:30Z – gemini-cli – shell_pid=128439 – lane=done – Review passed: SimulationLoop, AgentInterface, and Metrics are implemented. Added missing energy calculation and verified with tests.
