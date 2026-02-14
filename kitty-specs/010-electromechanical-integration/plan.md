# Implementation Plan: Electromechanical Integration (WP3)

*Path: [kitty-specs/010-electromechanical-integration/plan.md](kitty-specs/010-electromechanical-integration/plan.md)*

**Branch**: `feat/wp3-electronics` | **Date**: 2026-02-14 | **Spec**: [spec.md](spec.md)
**Input**: Feature specification from `/kitty-specs/010-electromechanical-integration/spec.md`

## Summary

This work package enables the AI to design and verify electromechanical systems. The technical approach involves:

- **Circuit Simulation**: Using `PySpice` (Ngspice) to validate electrical soundness and determine motor power status.
- **Power Gating**: Enforcing that motors only operate if properly powered in the physics simulation (`is_powered` scaling of control inputs).
- **3D Wire Routing**: Defining wires as 3D splines with waypoints, enforced by clearance checks in `build123d` and tension/breakage monitoring in MuJoCo tendons.
- **Visual Coordination**: Providing the Electrical Agent with a "Visual Map" of the assembly and a "Spline Query Tool" to propose waypoints autonomously.

## Technical Context

**Language/Version**: Python 3.11+
**Primary Dependencies**: `pyspice`, `skidl`, `build123d`, `trimesh`, `mujoco` (tendons)
**Storage**: `parts.db` (SQLite via SQLAlchemy) for COTS electronics indexing.
**Testing**: `pytest` for circuit logic and routing validation.
**Target Platform**: Linux server (Worker environment)
**Project Type**: Multi-worker web application (FastAPI backend + React/Three.js frontend).
**Performance Goals**: Circuit simulation < 1s; Wire routing validation < 2s.
**Constraints**: 100% power gating correctness; Zero penetration of solid volumes by wires.

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

[Passed - All requirements align with the project's electromechanical focus and safety standards.]

## Project Structure

### Documentation (this feature)

```
kitty-specs/010-electromechanical-integration/
├── plan.md              # This file
├── research.md          # Phase 0 results: PySpice reliability and Spline algorithms
├── data-model.md        # Phase 1: ElectricalAgent Tool schemas and Map format
├── quickstart.md        # Setup guide for local Ngspice and circuit testing
├── contracts/           # API/Pydantic schemas for ElectronicsSection
└── tasks.md             # Implementation tasks
```

### Source Code (repository root)

```
src/
├── shared/
│   ├── circuit_builder.py    # Spice netlist generation
│   ├── pyspice_utils.py       # Simulation & validation loop
│   ├── wire_utils.py          # Spline math & clearance checks
│   └── models/                # Pydantic schemas for ElectronicsSection
├── worker/
│   ├── simulation/
│   │   ├── builder.py         # Tendon/Site injection for MuJoCo
│   │   └── loop.py            # is_powered gating logic
│   └── workbenches/           # Electrical Planner node logic
├── controller/
│   └── api/                   # Electronics visualization endpoints
└── frontend/
    └── src/                   # tscircuit integration
```

**Structure Decision**: Web application structure (Option 2) as it involves both backend physics/circuit logic and frontend `tscircuit` visualization.

## Complexity Tracking

| Violation | Why Needed | Simpler Alternative Rejected Because |
|-----------|------------|-------------------------------------|
| PySpice dependency | Real-world electrical validation | Heuristic-only checks miss circuit-level failure modes (overcurrent, floating nodes). |
| MuJoCo Tendons | Physical wire failure simulation | Static spline checks don't account for dynamic tension/tearing during robot movement. |
