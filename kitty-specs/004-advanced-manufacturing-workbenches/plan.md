# Implementation Plan: Advanced Manufacturing Workbenches

*Path: templates/plan-template.md*

**Branch**: `004-advanced-manufacturing-workbenches` | **Date**: 2026-02-01 | **Spec**: [kitty-specs/004-advanced-manufacturing-workbenches/spec.md](spec.md)
**Input**: Feature specification from `kitty-specs/004-advanced-manufacturing-workbenches/spec.md`

## Summary

This feature implements two advanced manufacturing workbenches: **CNC Milling** and **Injection Molding**. It introduces strict Design for Manufacturing (DFM) validation (draft angles, undercuts, tool access) using `trimesh` for geometric analysis. The system includes a volume-dependent cost model configurable via YAML, and a `check_manufacturability` tool for the agent, optimized with caching to ensure performance.

## Technical Context

**Language/Version**: Python 3.12
**Primary Dependencies**:

- `build123d` (Geometry generation/manipulation)
- `trimesh` (Geometric analysis: raycasting, draft angles)
- `numpy` (Vector math)
- `PyYAML` (Configuration loading - standard lib or `pyyaml`)
**Storage**: Stateless analysis; Configuration in `src/workbenches/manufacturing_config.yaml`
**Testing**: `pytest`
**Target Platform**: Linux (Development/Execution)
**Project Type**: Single project (Library/Agent Environment)
**Performance Goals**: DFM checks < 5s per part.
**Observability**: Record detailed analysis traces and thoughts for DFM violations.
**Constraints**:
- Must handle standard mesh formats (STL/OBJ) or internal `build123d` representations converted to mesh.
- Cost parameters must be hot-swappable via config.
**Scale/Scope**: ~3 new modules, integration into existing tool system.

## Constitution Check

*GATE: Skipped (Constitution file not found)*

## Project Structure

### Documentation (this feature)

```
kitty-specs/004-advanced-manufacturing-workbenches/
├── plan.md              # This file
├── research.md          # Phase 0 output
├── data-model.md        # Phase 1 output
├── quickstart.md        # Phase 1 output
├── contracts/           # Phase 1 output
└── tasks.md             # Phase 2 output
```

### Source Code (repository root)

```
src/
├── workbenches/
│   ├── cnc.py                    # CNC Workbench implementation
│   ├── injection_molding.py      # Injection Molding Workbench implementation
│   ├── manufacturing_config.yaml # Cost & Constraints configuration
│   └── analysis_utils.py         # Shared DFM logic (using trimesh)
├── environment/
│   └── tools.py                  # Updated to expose check_manufacturability
```

**Structure Decision**: Extending existing `src/workbenches/` package with new modules. Shared analysis logic (raycasting, etc.) separated into `analysis_utils.py` to keep workbench classes clean.

## Complexity Tracking

| Violation | Why Needed | Simpler Alternative Rejected Because |
|-----------|------------|-------------------------------------|
| `trimesh` dependency | Advanced geometric queries (raycasting for undercuts) | `build123d`/OCP native kernels are complex/slow for discrete raycasting tasks; `trimesh` is industry standard for this. |
| YAML Config | Hot-swappable pricing/constraints | Hardcoding prevents easy tuning of the economic model without code changes. |
