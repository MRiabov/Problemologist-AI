# Code Review: WP2 Fluid and Deformable Material Implementation

**Date**: February 14, 2026
**Scope**: WP2 Simulation Backends, Meshing, Validation, and Agent Reasoning

## Executive Summary

The infrastructure for WP2 is partially implemented and well-instrumented with observability events. However, the core physics simulation engine (Genesis integration) and the meshing pipeline are currently represented by skeletons and stubs. Agent reasoning for stress and fluids is not yet updated in the prompt templates.

---

## 1. Simulation Backends & factory

- **[PASS]** `PhysicsBackend` protocol is correctly defined in `shared/simulation/backends.py`.
- **[PASS]** `MuJoCoBackend` is fully functional for rigid-body baseline.
- **[GAP]** `GenesisBackend` is a skeleton (`worker/simulation/genesis_backend.py`).
  - Core methods (`step`, `get_body_state`) are largely stubs.
  - `get_stress_field` and `get_particle_positions` return **dummy data** (fixed values) to satisfy integration tests.
  - Integration with Genesis Python API (`import genesis as gs`) is missing or minimal.

## 2. Asset Pipeline & Builders

- **[PASS]** `GenesisSimulationBuilder` correctly distinguishes between `mesh` (rigid) and `soft_mesh` (deformable) types.
- **[BLOCKER]** `GenesisSimulationBuilder.build_from_assembly` calls `self.backend.build(assembly_data)`, but `GenesisBackend` does not implement a `build()` method.
- **[GAP]** `tetrahedralize` in `worker/utils/mesh_utils.py` is a placeholder. It logs "TetGen not found" (expected if not in env) and doesn't output valid `.msh` files for Genesis.

## 3. Simulation Loop & Objectives

- **[PASS]** `SimulationLoop` (`worker/simulation/loop.py`) is well-instrumented for WP2.
  - Correctly evaluates `fluid_objectives` (containment, flow rate).
  - Handles `PART_BREAKAGE` failure based on `ultimate_stress_pa`.
  - Collects `StressSummary` and `FluidMetricResult`.
- **[NOTE]** This logic depends on dummy data from the current `GenesisBackend` stub, giving a false sense of completion in integration tests.

## 4. Validation & Tools

- **[PASS]** `validate_and_price` accurately enforces `youngs_modulus_pa`, `yield_stress_pa`, etc., if `fem_enabled` is set.
- **[GAP]** `preview_stress` in `worker/utils/validation.py` is an empty `pass`. No rendering logic for stress heatmaps exists.
- **[GAP]** Fluid particle rendering is mentioned as "automatic" in Genesis, but without a working backend, this is not verified.

## 5. Agent Integration

- **[GAP]** `Architect`, `Engineer`, and `Critic` prompt templates (`config/prompts.yaml`) have NOT been updated with WP2 instructions. They do not reason about material stress or fluid interactions.
- **[GAP]** `Benchmark Generator` agents are not yet capable of designing fluid-based or stress-based puzzles.

---

## Recommendation & Prioritization

1. **High Priority**: Implement the actual Genesis scene loading and stepping in `GenesisBackend`.
2. **High Priority**: Fix the `GenesisSimulationBuilder` vs `GenesisBackend.build()` API mismatch.
3. **Medium Priority**: Implement a functional `tetrahedralize` utility (likely using `Gmsh` or fixing `TetGen` integration).
4. **Medium Priority**: Update Agent prompts to include stress analysis and fluid containment goals.
5. **Low Priority**: Implement `preview_stress` heatmap rendering for frontend delivery.

## Status: Partially Implemented (Infrastructure Ready, Engine Missing)
