---
work_package_id: WP04
title: Physics Simulation Integration
lane: "doing"
dependencies: []
base_branch: main
base_commit: 64a983a01a67000d26b958dc92f89c0e241730e6
created_at: '2026-02-15T09:19:57.061535+00:00'
subtasks: [T014, T015, T016, T017, T018]
shell_pid: "111120"
agent: "Gemini"
review_status: "has_feedback"
reviewed_by: "MRiabov"
---

# WP04: Physics Simulation Integration

## Objective
Couple the electrical circuit state with the physics simulation engine (MuJoCo/Genesis) to enforce power gating and wire breakage.

## Context
A motor should only exert torque if it is powered. If a wire is torn mid-simulation, the motor should stop. We use MuJoCo `tendons` to represent physical wires.

## Detailed Guidance

### T014: Update `worker/simulation/builder.py` (MJCF Injection)
**Purpose**: Represent wires in the physics engine.

**Steps**:
1. For each `WireConfig`, create a `tendon` in the MJCF.
2. Inject `site` elements at the waypoints.
3. Set tendon properties (stiffness, damping) based on wire material.

### T015: Update `worker/simulation/loop.py` (Power Gating)
**Purpose**: Modulate motor torque based on circuit state.

**Steps**:
1. In each `step()`, query the circuit simulation (or a pre-calculated state).
2. Multiply the requested actuator control input by `is_powered` (0 or 1).
3. If a wire is torn (see T016), `is_powered` must drop to 0 permanently.

### T016: Implement tension monitoring
**Purpose**: Detect wire breakage.

**Steps**:
1. Read the tension (force) on each tendon from the physics engine (`mj_data.ten_force`).
2. Compare with the wire's `tensile_strength_n`.
3. If exceeded, mark the wire as "torn" and emit an event.

### T017: Implement mid-simulation failure handling
**Purpose**: Report electrical failures during runtime.

**Steps**:
1. Capture `wire_torn` events.
2. If a critical wire is torn, the simulation should potentially fail (depending on objective).

### T018: Add simulation tests
**Purpose**: Verify the "powered" logic.

**Steps**:
1. Create a test scene with a motor and a wire.
2. Pull the motor away until the wire tears.
3. Verify the motor stops spinning immediately.

## Definition of Done
- Motors in simulation respect electrical power status.
- Wire tearing is detected and causes motor stoppage.
- MJCF correctly includes tendons for all defined wires.

## Risks
- MuJoCo tendons being unstable if waypoints are too close or paths too complex.
- Performance overhead of monitoring every wire every timestep.

## Activity Log

- 2026-02-15T09:19:57Z – Antigravity – shell_pid=22429 – lane=doing – Assigned agent via workflow command
- 2026-02-15T09:48:02Z – Antigravity – shell_pid=22429 – lane=planned – Moved to planned
- 2026-02-15T09:55:03Z – gemini-cli – shell_pid=102091 – lane=doing – Started implementation via workflow command
- 2026-02-15T10:03:00Z – gemini-cli – shell_pid=102091 – lane=for_review – Integrated electrical circuit state with physics simulation. Added MuJoCo spatial tendon support for wires, implemented power gating for actuators, and added mid-simulation wire breakage detection based on tensile strength. Added comprehensive unit and integration tests.
- 2026-02-15T10:04:10Z – Gemini – shell_pid=111120 – lane=doing – Started implementation via workflow command
