---
work_package_id: WP03
title: Genesis FEM Implementation
lane: "for_review"
dependencies: []
base_branch: main
base_commit: 796a208b00640600aa94c20604bb5a7ccf216a7e
created_at: '2026-02-15T09:49:14.986957+00:00'
subtasks: [T010, T011, T012, T013]
shell_pid: "102449"
agent: "gemini-agent"
---

# WP03: Genesis FEM Implementation

## Objective
Implement deformable body simulation in Genesis, including real-time stress monitoring and part breakage detection.

## Context
This is the core physics implementation. We are moving from rigid-body to FEM. Genesis provides the solver; we need to hook it into our reward/failure system.

## Guidance

### T010: Scene Loading in Genesis
- Implement `load_scene` in `GenesisBackend`.
- Support both rigid (MuJoCo-like) and deformable (`.msh` + FEM) entities.
- Assign material properties (Young's modulus, etc.) from the assembly definition to Genesis entities.

### T011: Stress Calculation & Summary
- During simulation steps, extract von Mises stress from the Genesis FEM solver.
- Aggregate per-part `StressSummary`:
  - `max_von_mises_pa`
  - `mean_von_mises_pa`
  - `safety_factor` (Ultimate Stress / Max Stress)
  - `utilization_pct`

### T012: Part Breakage Detection
- Compare `max_von_mises_pa` against the material's `ultimate_stress_pa` every timestep.
- If exceeded, stop simulation immediately.
- Return `failure_reason: PART_BREAKAGE` with part label and location.

### T013: Material Model Support
- Support linear FEM for `rigid` materials.
- Support hyperelastic (Neo-Hookean) for `soft` and `elastomer` materials.
- Verify stability for large deformations.

## Definition of Done
- [ ] Simulation shows visible deformation for load-bearing parts.
- [ ] Exceeding ultimate stress triggers `PART_BREAKAGE` failure.
- [ ] `SimulationResult` contains valid `StressSummary` for all deformable parts.
- [ ] Both linear and hyperelastic models are active based on material class.

## Risks
- Numerical instability in the FEM solver (NaNs).
- Significant slowdown compared to rigid-body simulation.

## Activity Log

- 2026-02-15T09:49:15Z – Gemini – shell_pid=98628 – lane=doing – Assigned agent via workflow command
- 2026-02-15T09:50:56Z – Gemini – shell_pid=98628 – lane=for_review – Ready for review: Genesis FEM implementation complete. Added scene loading for soft meshes, material model support, real-time stress extraction, and breakage detection.
- 2026-02-15T09:55:21Z – gemini-agent – shell_pid=102449 – lane=doing – Started review via workflow command
- 2026-02-15T10:15:40Z – gemini-agent – shell_pid=102449 – lane=for_review – Ready for review: Genesis FEM implementation complete. Added scene loading for soft meshes, material model support, real-time stress extraction, and breakage detection. (Re-moving to for_review as requested)
