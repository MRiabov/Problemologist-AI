---
work_package_id: WP03
title: Simulation Bridge (MuJoCo)
lane: "doing"
dependencies: []
subtasks:
- T011
- T012
- T013
- T014
phase: Phase 3 - Physics
agent: "Antigravity"
shell_pid: "103573"
history:
- timestamp: '{{TIMESTAMP}}'
  lane: planned
  agent: system
  action: Prompt generated via /spec-kitty.tasks
---

# Work Package Prompt: WP03 – Simulation Bridge (MuJoCo)

## Objectives & Success Criteria

- **Goal**: Enable the system to run physics simulations to evaluate designs.
- **Success Criteria**:
  - Can load a base MJCF XML template.
  - Can inject a generated mesh (from WP02) into the XML at a specific body/geom location.
  - Can run the simulation for T seconds headless.
  - Returns metrics (energy, success).

## Context & Constraints

- **Spec**: 4.4 Simulation Bridge.
- **Library**: `mujoco` (the python binding, not mujoco-py).
- **Format**: MJCF (XML).

## Subtasks & Detailed Guidance

### Subtask T011 – MJCF Template Loading

- **Purpose**: Load the scenario definitions.
- **Steps**:
  - Create `src/compiler/mujoco_bridge.py`.
  - Create a directory `src/compiler/templates/` and add a `standard.xml` (basic ground plane + gravity).
  - Implement `load_template(template_path) -> str|etree`.
- **Files**: `src/compiler/mujoco_bridge.py`, `src/compiler/templates/standard.xml`.
- **Parallel?**: No.

### Subtask T012 – Geometry Injection Logic

- **Purpose**: Place the agent's CAD output into the physics world.
- **Steps**:
  - In `src/compiler/mujoco_bridge.py`, implement `inject_design(xml, mesh_path, location) -> final_xml_string`.
  - Needs to add `<asset><mesh file="..."/></asset>` and `<body ...><geom type="mesh" .../></body>`.
  - Be careful with paths (absolute vs relative to XML).
- **Files**: `src/compiler/mujoco_bridge.py`.
- **Parallel?**: No.

### Subtask T013 – Simulation Loop

- **Purpose**: Run the physics.
- **Steps**:
  - Implement `run_simulation(xml_string, duration=5.0) -> SimResult`.
  - Use `mujoco.MjModel.from_xml_string`.
  - Loop: `while data.time < duration: mujoco.mj_step(model, data)`.
- **Files**: `src/compiler/mujoco_bridge.py`.
- **Parallel?**: No.

### Subtask T014 – Metrics Extraction

- **Purpose**: Score the run.
- **Steps**:
  - Extend `run_simulation` to track data.
  - **Energy**: Accumulate `data.qfrc_actuator * data.qvel` or similar work term.
  - **Damage**: Monitor `data.contact`. If contact involves "forbidden" geoms, accumulate impulse.
  - **Success**: Check if target body is in goal zone (simple coordinate check).
- **Files**: `src/compiler/mujoco_bridge.py`.
- **Parallel?**: No.

## Test Strategy

- **Test File**: `tests/test_mujoco.py`.
- **Cases**:
  - Load template -> Works.
  - Inject Design -> XML contains mesh tag.
  - Run Sim (Empty) -> Returns 0 energy.
  - Run Sim (Falling object) -> Coordinates change (z goes down).

## Risks & Mitigations

- **Mesh Errors**: Bad meshes crash MuJoCo. Ensure WP02 output is clean. If crash, wrap in try/except and return penalty.
- **Headless**: Keep `mujoco.gl_context` null or use `osmesa` if rendering is needed (but here we just need physics step). Note: `preview_design` does visuals, this does physics. Using `mujoco` for physics only doesn't require a window.

## Activity Log

- 2026-02-01T08:26:19Z – Antigravity – shell_pid=103573 – lane=doing – Started implementation via workflow command
