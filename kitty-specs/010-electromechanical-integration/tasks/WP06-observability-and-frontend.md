---
work_package_id: WP06
title: Observability & Frontend
lane: "doing"
dependencies: []
base_branch: main
base_commit: 43f6ee1c33219f89f6f6bb567af9814f3208bcdf
created_at: '2026-02-15T10:35:05.906139+00:00'
subtasks: [T025, T026, T027, T028, T029]
shell_pid: "601944"
agent: "Gemini"
reviewed_by: "MRiabov"
review_status: "has_feedback"
---

# WP06: Observability & Frontend

## Objective
Provide visual and telemetry feedback for the electromechanical systems in the dashboard and 3D viewer.

## Context
Users (and developers) need to see the electrical nets and 3D wires to verify that the agents are designing correctly. We use `tscircuit` for 2D schematics.

## Detailed Guidance

### T025: Update `shared/observability/schemas.py`
**Purpose**: Track electrical performance.

**Steps**:
1. Add events: `circuit_simulated`, `wire_torn`, `power_budget_warning`.
2. Ensure they are captured in LangFuse.

### T026: Update `controller/api/` (Schematic Export)
**Purpose**: Provide data for `tscircuit`.

**Steps**:
1. Create an endpoint `GET /episodes/{id}/electronics/schematic`.
2. Map the `ElectronicsSection` model to a format compatible with `tscircuit`.

### T027: Implement 3D wire rendering
**Purpose**: Visualize wiring in the CAD viewer.

**Steps**:
1. Update the frontend 3D viewer (Three.js) to render tubes along the wire splines.
2. Color-code wires (e.g., Red for Vcc, Black for Gnd).

### T028: Add "Electronics View" toggle
**Purpose**: Switch between mechanical and electrical focus.

**Steps**:
1. Add a button in the dashboard to toggle the electronics overlay.
2. In this mode, make mechanical parts semi-transparent and highlight wires.

### T029: Integrate `tscircuit` component
**Purpose**: Render the schematic.

**Steps**:
1. Add the `tscircuit` React component to the dashboard.
2. Connect it to the API endpoint from T026.

## Definition of Done
- Schematics are visible in the dashboard.
- 3D wires are rendered correctly in the viewer.
- "Wire torn" events are visible in the simulation logs.

## Risks
- `tscircuit` integration being complex or requiring heavy dependencies.
- Frontend performance degradation with many 3D wires.

## Activity Log

- 2026-02-15T10:35:06Z – Gemini – shell_pid=142576 – lane=doing – Assigned agent via workflow command
- 2026-02-15T11:04:08Z – Gemini – shell_pid=142576 – lane=for_review – Implemented electromechanical observability events, schematic export API, 3D wire rendering, and tscircuit integration.
- 2026-02-15T11:24:35Z – gemini-cli – shell_pid=192451 – lane=doing – Started review via workflow command
- 2026-02-15T11:31:42Z – gemini-cli – shell_pid=192451 – lane=done – Review passed: Implementation successfully delivers the observability and frontend requirements for electromechanical integration. Key achievements: 1. Added critical electrical events (circuit_simulated, wire_torn, power_budget_warning) to observability schemas. 2. Implemented a new controller API endpoint for tscircuit schematic export. 3. Enhanced the 3D viewer with spline-based wire rendering and color-coding heuristics. 4. Integrated tscircuit React component for logic visualization. 5. Added a functional 'Electronics View' toggle to focus on electrical systems.
- 2026-02-16T07:34:00Z – Antigravity – shell_pid=484488 – lane=doing – Started review via workflow command
- 2026-02-16T07:40:56Z – Antigravity – shell_pid=484488 – lane=planned – Moved to planned
- 2026-02-16T07:49:18Z – Gemini – shell_pid=601944 – lane=doing – Started implementation via workflow command
