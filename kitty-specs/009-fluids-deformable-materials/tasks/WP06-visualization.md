---
work_package_id: WP06
title: Visualization & Frontend Integration
lane: planned
dependencies: []
subtasks: [T022, T023, T024]
---

# WP06: Visualization & Frontend Integration

## Objective
Ensure that the results of fluid and FEM simulations are visible to the user in the frontend.

## Context
A simulation isn't useful if the user can't see the stress or the fluid. We need to bridge the gap between Genesis data and the React frontend.

## Guidance

### T022: MP4 Particle Rendering
- Update Genesis video rendering (via Genesis's built-in renderer or a custom shader).
- Ensure fluid particles are rendered with the correct colour and transparency in the final MP4.

### T023: Stress Heatmap Generation
- Generate static PNG/JPG renders of the assembly with a von Mises stress heatmap overlay (Blue to Red).
- These should be part of the standard `SimulationResult` artifacts.

### T024: Frontend UI Updates
- Update the `DesignViewer` and `SimulationResults` components.
- Display `StressSummary` tables.
- Show stress heatmap images alongside the multi-view renders.
- Display `FluidMetricResult` progress bars or stats.

## Definition of Done
- [ ] Simulation videos show realistic fluid particles.
- [ ] Stress heatmaps are available in the results gallery.
- [ ] Frontend displays numerical stress data and fluid metrics.

## Risks
- Rendering overhead significantly increasing simulation time.
- Frontend clutter from too many new UI elements.
