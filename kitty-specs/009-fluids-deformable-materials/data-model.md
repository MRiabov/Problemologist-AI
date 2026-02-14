# Data Model: WP2 Fluids & Deformable Materials

## Entities

### `StressReportArray`

- **Fields**:
  - `part_label`: str
  - `nodes`: list[float] (flat array of [x, y, z, ...])
  - `von_mises`: list[float] (per-node scalar)
- **Rationale**: Transmitting full node/stress arrays to the frontend for interactive visualization as requested.

### `FluidParticleData`

- **Fields**:
  - `fluid_id`: str
  - `positions`: list[float] (flat array of [x, y, z, ...])
- **Rationale**: Particle visualization in the frontend.

### `SimulationResult` (Extended)

- **New Fields**:
  - `stress_data`: list[StressReportArray]
  - `fluid_data`: list[FluidParticleData]
  - `failure_reason`: str (Adding `PART_BREAKAGE`, `FLUID_OBJECTIVE_FAILED`, `PHYSICS_INSTABILITY`)

## Validation Rules

- `max_von_mises_pa` MUST be greater than 0.
- `ultimate_stress_pa` MUST be provided for all manufactured parts in FEM mode.
- `containment_zone` MUST be a valid AABB (min < max).
