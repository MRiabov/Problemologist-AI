---
work_package_id: WP06
title: Observability & Frontend
lane: planned
dependencies: []
subtasks: [T025, T026, T027, T028, T029]
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
