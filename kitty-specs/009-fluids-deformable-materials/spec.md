# Feature Specification: Fluids & Deformable Materials

**Feature Branch**: `009-fluids-deformable-materials`
**Created**: 2026-02-14
**Status**: Draft
**Input**: WP2 from `roadmap.md` — Extend the system from rigid-body-only dynamics to deformable materials (FEM) and fluids (MPM).

## User Scenarios & Testing *(mandatory)*

### User Story 1 — Deformable Part Simulation (Priority: P1)

An agent designs a mechanism with manufactured parts. When the simulation runs, every manufactured part deforms under load according to its material properties. If any part exceeds its ultimate stress, the simulation stops and the agent is told exactly which part broke, where, and at what stress level, so it can fix the design.

**Why this priority**: Deformable materials are the core change — the system must correctly simulate FEM for all manufactured parts before any other WP2 feature makes sense.

**Independent Test**: Run a benchmark with a load-bearing part made of a known material (e.g., ABS plastic). Apply a force that exceeds the material's ultimate stress. Verify the simulation fails with `PART_BREAKAGE`, reports the correct part label, stress value, and location.

**Acceptance Scenarios**:

1. **Given** a manufactured part with `material_id: abs-plastic` under a load exceeding 44 MPa ultimate stress, **When** the simulation runs with `fem_enabled: true`, **Then** the simulation terminates immediately with `failure_reason: PART_BREAKAGE` and includes the part label, step number, max von Mises stress, and failure location.
2. **Given** a manufactured part under moderate load (safety factor > 1.5), **When** the simulation completes, **Then** `SimulationResult.stress_summaries` contains a `StressSummary` for the part with correct `max_von_mises_pa`, `safety_factor`, and `utilization_pct`.
3. **Given** a part with `material_class: elastomer` (e.g., silicone rubber), **When** it undergoes large deformation, **Then** the simulation uses a hyperelastic model and the part deforms realistically without spurious instabilities.

---

### User Story 2 — Togglable Physics Backends (Priority: P1)

An operator can choose between MuJoCo (rigid-body only, fast) and Genesis (FEM + fluids) per episode. Existing rigid-body benchmarks continue to work unchanged on MuJoCo. New benchmarks use Genesis by default.

**Why this priority**: Backend abstraction is the foundation for all WP2 work. Both backends must coexist without regressions.

**Independent Test**: Run the same rigid-body benchmark on both MuJoCo and Genesis (with `fem_enabled: false`). Compare results for parity. Then run a FEM-enabled benchmark on Genesis and verify deformation occurs.

**Acceptance Scenarios**:

1. **Given** a configuration with `physics.backend: mujoco`, **When** a rigid-body benchmark runs, **Then** the simulation completes identically to the pre-WP2 behaviour.
2. **Given** a configuration with `physics.backend: genesis, fem_enabled: false`, **When** the same rigid-body benchmark runs, **Then** the results are comparable to the MuJoCo run (within a tolerance defined by physics engine differences).
3. **Given** a configuration with `physics.backend: genesis, fem_enabled: true`, **When** the benchmark involves load-bearing parts, **Then** stress summaries are produced and deformation is visible.

---

### User Story 3 — Fluid Simulation (Priority: P2)

Benchmarks can include fluids (liquids). Agents design mechanisms that interact with fluids. Fluid behaviour is evaluated via containment and flow-rate objectives.

**Why this priority**: Fluids are the second major capability of WP2, building on the Genesis backend from P1.

**Independent Test**: Create a benchmark with a fluid containment objective. Run the simulation. Verify particles are tracked and the containment metric passes or fails correctly.

**Acceptance Scenarios**:

1. **Given** a benchmark with a fluid defined (`fluid_id: water_fill`, `density: 1000`, `viscosity: 1.0`) and a `fluid_containment` objective with `threshold: 0.95`, **When** the simulation runs, **Then** the system tracks particle positions and reports a `FluidMetricResult` with the containment ratio.
2. **Given** a `flow_rate` objective with a gate plane, **When** the simulation runs, **Then** the system counts particles crossing the plane and reports a measured flow rate.
3. **Given** a fluid benchmark with `physics.backend: mujoco`, **When** the benchmark is submitted, **Then** validation rejects it with a clear error: fluid simulation requires the Genesis backend.

---

### User Story 4 — Agent Stress Reasoning (Priority: P2)

After simulation, agents can query stress reports for load-bearing parts and use them to make design decisions. Agents balance adding material where stress is high and removing material where stress is low, preventing both breakage and overdesign.

**Why this priority**: This is the agent behaviour change that makes FEM useful — without it, stress data is collected but never acted upon.

**Independent Test**: Run a benchmark where a part has a safety factor below 1.5. Verify the agent receives the stress report and adjusts the design. Run another where safety factor is above 5.0 and verify the agent reduces material.

**Acceptance Scenarios**:

1. **Given** a simulation that has completed, **When** the agent calls `get_stress_report(part_label)`, **Then** it receives a `StressSummary` with `max_von_mises_pa`, `safety_factor`, `utilization_pct`, and `location_of_max`.
2. **Given** a part with `safety_factor < 1.5`, **When** the agent reviews the stress report, **Then** the agent's plan includes adding material or changing geometry to increase the safety factor.
3. **Given** a part with `safety_factor > 5.0`, **When** the agent reviews the stress report, **Then** the agent's plan includes removing unnecessary material to reduce cost and weight.

---

### User Story 5 — Stress & Fluid Visualization (Priority: P3)

Users can view simulation results in the frontend: fluid particles in simulation videos and stress heatmaps as rendered images alongside existing multi-view renders.

**Why this priority**: Frontend delivery is important for user verification but is not blocking the core simulation and agent capabilities.

**Independent Test**: Run a fluid benchmark and open the results in the frontend. Verify the MP4 video shows fluid particles. Verify stress heatmap images are displayed alongside standard renders.

**Acceptance Scenarios**:

1. **Given** a completed fluid simulation, **When** the user views the episode in the frontend, **Then** the simulation video includes visible fluid particle rendering.
2. **Given** a completed FEM simulation, **When** the user views the episode results, **Then** stress heatmap images (blue=low, red=high) are displayed for each deformable part alongside existing views.

---

### User Story 6 — Fluid & Stress Benchmarks (Priority: P2)

The benchmark planner and CAD engineer can create new benchmark types: fluid containment, fluid transport, stress-limited, and hybrid (fluid + mechanism) benchmarks.

**Why this priority**: Without new benchmark types, the system cannot evaluate agent performance in fluid/stress scenarios.

**Independent Test**: Use the benchmark planner to generate a fluid containment benchmark. Verify the resulting `objectives.yaml` includes `physics`, `fluids`, and `fluid_objectives` sections with correct schema.

**Acceptance Scenarios**:

1. **Given** the benchmark planner is asked to create a fluid containment challenge, **When** it generates `objectives.yaml`, **Then** the file includes a `physics` section with `backend: genesis`, a `fluids` section with fluid properties, and a `fluid_objectives` section with type `fluid_containment`.
2. **Given** the benchmark planner creates a stress-limited benchmark, **When** it generates `objectives.yaml`, **Then** the file includes `stress_objectives` with `type: max_stress` and a valid `max_von_mises_mpa` threshold.

---

### Edge Cases

- What happens when a material in `manufacturing_config.yaml` is missing FEM fields and `fem_enabled: true`? → Validation fails early with a clear error message listing the missing fields.
- What happens when TetGen fails to mesh a self-intersecting geometry? → Mesh repair is attempted (via trimesh). If repair fails, the task fails with `FAILED_ASSET_GENERATION` and the agent gets a diagnostic message with approximate error location.
- What happens when a GPU runs out of memory during fluid simulation? → The system auto-retries with 25% fewer particles. Results are labelled as approximate. A `gpu_oom_retry` event is emitted.
- What happens when the simulation becomes numerically unstable (NaN/infinite kinetic energy)? → Simulation aborts with `PHYSICS_INSTABILITY`. Agent is advised to reduce timestep or increase mesh density.
- What happens when no GPU is available and `compute_target: auto`? → The system falls back to CPU. In smoke-test mode, particle count is capped to 5000 and results are labelled `approximate`.
- What happens when running fluid benchmarks with `backend: mujoco`? → Schema validation rejects the configuration — fluids require Genesis.

## Requirements *(mandatory)*

### Functional Requirements

- **FR-001**: System MUST support two physics backends (MuJoCo and Genesis) behind a common interface, selectable per episode via configuration.
- **FR-002**: System MUST apply finite-element modelling (FEM) to all manufactured parts when `fem_enabled: true`, using material properties from `manufacturing_config.yaml`.
- **FR-003**: System MUST immediately terminate simulation when any part exceeds its `ultimate_stress_pa`, reporting the part label, step, stress value, and failure location.
- **FR-004**: System MUST compute periodic stress summaries (`StressSummary`) for all FEM bodies during simulation, including max/mean von Mises stress, safety factor, and utilization percentage.
- **FR-005**: System MUST support fluid definition with viscosity, density, surface tension, and colour properties.
- **FR-006**: System MUST evaluate fluid-based objectives: containment (fraction of particles within a zone) and flow rate (particles crossing a gate plane per second).
- **FR-007**: System MUST convert manufactured parts through the asset pipeline: BREP → tessellation → tetrahedralization (TetGen) → `.msh` for Genesis FEM loading.
- **FR-008**: System MUST provide agents with tools to query stress state (`get_stress_report`) and render stress heatmaps (`preview_stress`) after simulation.
- **FR-009**: System MUST provide agents with a tool to define fluids (`define_fluid`) for use in simulation.
- **FR-010**: System MUST use linear FEM for `material_class: rigid` parts and hyperelastic (Neo-Hookean) models for `material_class: soft` and `elastomer` parts.
- **FR-011**: System MUST handle meshing failures with an automatic repair attempt (trimesh) followed by retry, failing gracefully with a diagnostic message to the agent if repair fails.
- **FR-012**: System MUST detect physics instability (kinetic energy exceeding threshold) and abort simulation with `PHYSICS_INSTABILITY`.
- **FR-013**: System MUST handle GPU OOM during fluid simulation by auto-retrying with reduced particle count and labelling results as approximate.
- **FR-014**: System MUST validate that all materials referenced by parts have FEM fields populated when `fem_enabled: true`, failing early with a clear error if any are missing.
- **FR-015**: System MUST support a `smoke_test_mode` configuration that caps particles to 5000 for CI/CD and local development, labelling results as approximate.
- **FR-016**: System MUST persist only MP4 video, JSON summary metrics, and stress summaries to storage. Raw particle data MUST be deleted after processing.
- **FR-017**: Benchmark planner MUST be able to create fluid-based and stress-based benchmarks with appropriate `physics`, `fluids`, `fluid_objectives`, and `stress_objectives` sections in `objectives.yaml`.
- **FR-018**: Engineer agent MUST include "Stress Considerations" in risk assessments for FEM-enabled benchmarks.
- **FR-019**: Engineer agent MUST call `get_stress_report()` on load-bearing parts after simulation and balance material usage (add where safety factor < 1.5, remove where > 5.0).
- **FR-020**: Reviewer agent MUST verify that no parts have utilization > 80% without justification and that fluid containment metrics pass for fluid benchmarks.
- **FR-021**: System MUST emit observability events for: backend selection, part breakage, fluid containment checks, flow rate checks, stress summaries, meshing failures, physics instability, and GPU OOM retries.
- **FR-022**: System MUST render fluid particles in simulation videos and stress heatmaps as rendered images available in the frontend.
- **FR-023**: Forward-compatible failure mode `ELECTRONICS_FLUID_DAMAGE` MUST be defined for fluid–electronics interaction (activation deferred to WP3).

### Key Entities

- **PhysicsBackend**: Interface encapsulating simulator operations (load scene, step, get body/stress/particle state, render). Implementations: MuJoCo, Genesis.
- **SimulatorBackendType**: Enum (`mujoco`, `genesis`) for backend selection.
- **StressSummary**: Per-part stress data — part label, max/mean von Mises stress, safety factor, utilization percentage, location of max stress.
- **FluidDefinition**: Fluid material properties — viscosity, density, surface tension, colour.
- **FluidMetricResult**: Evaluation result for fluid objectives — metric type, fluid ID, measured vs target value, pass/fail.
- **SimulationResult** (extended): Gains `stress_summaries`, `fluid_metrics`, and extended `failure_reason` values (`PART_BREAKAGE`, `ELECTRONICS_FLUID_DAMAGE`, `STRESS_OBJECTIVE_EXCEEDED`, `FLUID_OBJECTIVE_FAILED`, `PHYSICS_INSTABILITY`).

## Assumptions

- Genesis physics engine is available as a Python package and supports the required FEM and MPM features as documented.
- TetGen is available for installation in the worker container and produces usable tetrahedral meshes from STL input.
- The existing `manufacturing_config.yaml` structure can be extended with FEM-specific fields without breaking existing consumers.
- MuJoCo-based benchmarks will continue to work without modification (no breaking changes to the MuJoCo backend).
- GPU is not strictly required — CPU execution is always available, albeit slower.
- The roadmap items marked `(extra, not strictly required)` for pipes/hoses and fluid-based goals are out of scope for this spec.

## Non-Goals

- Phase change (boiling, freezing)
- Chemical reactions between fluids
- Capillary-scale surface tension accuracy
- Pipes and enclosed fluid channels
- Fluid-based positional goals (fluid reaching a specific location)
- PCBs, sensors, or electronics integration (WP3, WP7)
- Topology optimization (WP5)
- Interactive 3D stress viewer (depends on WASM CAD viewer from main spec)
- Per-part deformation animation scrubber or fluid particle replay in frontend
- Fastener failure modelling under FEM load
- More realistic fasteners (deferred from roadmap)

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: All manufactured parts in FEM-enabled simulations produce valid `StressSummary` data with nonzero stress values — 100% of cases.
- **SC-002**: Part breakage is correctly detected and terminates the simulation within one timestep of exceeding ultimate stress — 100% of cases.
- **SC-003**: Fluid containment objectives evaluate correctly (particle counting matches expected values within 5% error) — 100% of cases.
- **SC-004**: Existing rigid-body benchmarks produce equivalent results on both MuJoCo and Genesis (with `fem_enabled: false`) — pass rate ≥ 95%.
- **SC-005**: Agent designs do not cause `PART_BREAKAGE` — 90% of solutions after stress-aware prompting.
- **SC-006**: Average safety factor across all parts in agent solutions is between 1.5 and 5.0 — 80% of solutions.
- **SC-007**: Given a fluid benchmark, the engineer produces a solution that passes fluid containment in 70% of attempts.
- **SC-008**: Schema validation catches 100% of invalid configurations (missing FEM fields, fluid on MuJoCo, etc.) before simulation starts.
- **SC-009**: Meshing failures trigger automatic repair and retry. Repaired meshes proceed to simulation successfully in ≥ 80% of repair attempts.
- **SC-010**: The planner includes "Stress Considerations" in risk assessments for FEM-enabled benchmarks — 90% of cases.
