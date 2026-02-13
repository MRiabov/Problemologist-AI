# Desired Architecture: WP2 — Fluids & Deformable Materials

## Objective

Extend the system from rigid-body-only dynamics to deformable materials (FEM) and fluids (MPM), making simulations physically realistic enough for actual engineering. After WP2, **all manufactured parts** deform under load, fluid benchmarks become a first-class objective type, and the agent pipeline (planner → engineer → reviewer) can reason about stress, breakage, and fluid behaviour.

### Goals

1. **Deformable materials as default.** Every manufactured part gets FEM properties from its `material_id`. Parts bend, and break if their strength is exceeded.
2. **Fluid simulation.** Benchmarks can contain, transport, or mix fluids. Agents must design around fluid behaviour.
3. **Hybrid interaction.** Rigid mechanisms interact with fluids and soft bodies (e.g., a pump, a gripper holding a deformable object).
4. **Togglable backends.** MuJoCo (rigid-only, fast) and Genesis (FEM + fluids) coexist behind an interface, selectable per episode.

### Non-Goals (this WP)

- Phase change (boiling, freezing)
- Chemical reactions between fluids
- Capillary-scale surface tension accuracy
- PCBs or sensors (WP3, WP7)
- Topology optimization (WP5)

---

## Physics Engine

### Simulator Interface

We introduce a `PhysicsBackend` abstraction so that MuJoCo and Genesis are interchangeable:

```python
class PhysicsBackend(Protocol):
    """Interface for physics simulators."""
    def load_scene(self, scene: SimulationScene) -> None: ...
    def step(self, dt: float) -> StepResult: ...
    def get_body_state(self, body_id: str) -> BodyState: ...
    def get_stress_field(self, body_id: str) -> StressField | None: ...
    def get_particle_positions(self) -> np.ndarray | None: ...
    
    # Rendering & Visualization
    def render_camera(self, camera_name: str, width: int, height: int) -> np.ndarray: ...
    def get_camera_matrix(self, camera_name: str) -> np.ndarray: ...
    
    # Advanced interactions (superset of MuJoCo)
    def set_site_pos(self, site_name: str, pos: np.ndarray) -> None: ...
    def get_contact_forces(self) -> list[ContactForce]: ...

class SimulatorBackendType(str, Enum):
    MUJOCO = "mujoco"      # Rigid-body only, fast, no FEM/fluids
    GENESIS = "genesis"     # FEM + MPM fluids, requires more compute
```

### Backend Selection

The backend is selected per episode via `config/simulation_config.yaml` (or typically just `objectives.yaml` in previous versions, now separated for clarity):

```yaml
# config/simulation_config.yaml
physics:
  backend: "genesis"        # "mujoco" | "genesis"
  fem_enabled: true         # Toggle FEM on/off (Genesis only)
  compute_target: "cpu"     # "cpu" | "gpu" — which device to run on
```

- **MuJoCo** continues to be used for pure rigid-body benchmarks and fast evals.
- **Genesis** is the default for any benchmark involving deformable materials or fluids.
- The `fem_enabled` flag allows running Genesis in rigid-only mode (faster) when FEM is not needed. This serves as a migration path: existing benchmarks run with `backend: genesis, fem_enabled: false` before graduating to full FEM.

### Why Genesis?

- **Unified multi-material support**: Native interactions between rigid bodies, MPM fluids, and FEM soft bodies.
- **Differentiable**: Enables gradient-based optimization in future work (not used in this WP).
- **Python-native**: Pythonic configuration, avoiding XML (unlike MuJoCo MJCF).

---

## Deformable Materials (FEM)

### Core Change: All Parts Deform

After WP2, every manufactured part is subject to finite-element modelling. Material properties come from the existing `manufacturing_config.yaml` `materials` section, which gains new FEM-specific fields:

```yaml
materials:
  aluminum-6061:
    # Existing fields...
    density_kg_m3: 2700
    friction_coef: 0.47
    restitution: 0.3
    color: "#A8A8A8"
    # NEW - FEM fields (WP2)
    youngs_modulus_pa: 68.9e9
    poissons_ratio: 0.33
    yield_stress_pa: 276e6      # von Mises yield strength
    ultimate_stress_pa: 310e6   # Failure threshold
    elongation_at_break: 0.12   # Strain at fracture
    material_class: "rigid"     # "rigid" | "soft" | "elastomer"
  abs-plastic:
    density_kg_m3: 1050
    youngs_modulus_pa: 2.3e9
    poissons_ratio: 0.35
    yield_stress_pa: 40e6
    ultimate_stress_pa: 44e6
    elongation_at_break: 0.06
    material_class: "rigid"
  silicone-rubber:
    density_kg_m3: 1100
    youngs_modulus_pa: 5e6      # Much softer
    poissons_ratio: 0.49
    yield_stress_pa: 7e6
    ultimate_stress_pa: 10e6
    elongation_at_break: 4.0
    material_class: "elastomer"
```

**Focus**: We prioritize more rigid materials (metals, plastics) over very soft ones (foam, rubber). `material_class: "rigid"` parts use linear FEM (fast); `"soft"` and `"elastomer"` use Neo-Hookean hyperelastic models (slower but accurate for large deformation).

### Part Breakage

When any element in a part exceeds `ultimate_stress_pa`:

1. The simulation **fails immediately** with `PART_BREAKAGE`.
2. The stress field at failure is captured and included in `SimulationResult`.
3. The agent receives a structured message: `"Part '{label}' broke at step {step}. Max von Mises stress: {stress_mpa} MPa > ultimate: {ultimate_mpa} MPa. Location: ({x}, {y}, {z})."`

This implements the roadmap requirements: "If a part breaks, the simulation stops and is failed" and "Agents are notified when the part breaks."

### Stress Reporting

At every N-th simulation step (configurable, default every 50 steps), a stress summary is computed for all FEM bodies:

```python
class StressSummary(BaseModel):
    part_label: str
    max_von_mises_pa: float
    mean_von_mises_pa: float
    safety_factor: float            # yield_stress / max_von_mises
    location_of_max: tuple[float, float, float]
    utilization_pct: float          # max_stress / yield_stress * 100
```

This data:

- Is appended to `SimulationResult.stress_summaries`
- Is available to agents via a new tool (see [Agent Tools](#agent-tools))
- Can be visualized in the frontend as stress heatmaps (see [Delivery](#delivery))

---

## Fluid Simulation (MPM)

### Fluid Definition

Fluids are defined via new fields in `objectives.yaml` (for benchmark-defined fluids) or via agent tools (for engineer-defined fluid interactions):

```yaml
# objectives.yaml — new section
fluids:
  - fluid_id: "water_fill"
    properties:
      viscosity_cp: 1.0       # Centipoise
      density_kg_m3: 1000
      surface_tension_n_m: 0.07
    initial_volume:
      type: "cylinder"        # Primitive shape filled with particles
      center: [0, 0, 50]
      radius: 15
      height: 30
    color: [0, 0, 200]
```

### Asset Conversion Pipeline

The existing `build123d → OBJ → MuJoCo` pipeline extends as follows:

| Asset type | Source | Processing | Genesis loading |
|---|---|---|---|
| **Rigid parts** (FEM disabled) | `build123d` BREP | Tessellate to OBJ | `gs.morphs.Mesh(file='part.obj')` |
| **Deformable parts** (FEM enabled) | `build123d` BREP | Tessellate to STL → **Tetrahedralize** (TetGen) → `.msh` | `gs.morphs.SoftMesh(file='part.msh')` |
| **Fluids** | Volume primitive in `objectives.yaml` | Particle sampling (Genesis handles seeding for primitives) | `gs.morphs.Box(...)` + `gs.materials.Liquid(...)` |

- **TetGen** is installed in the worker container. A `mesh_utils.py` wrapper handles invocation and error recovery.
- **V-HACD** (currently used for MuJoCo convex decomposition) is not needed for Genesis, which handles collision natively.
- When `backend: mujoco`, the pipeline remains unchanged (no FEM/fluid support).

### Fluid–Electronics Interaction

Per the roadmap: "If fluids are on electronics, the electronics is dead."

Once WP3 (Electronics) is implemented, the simulation checks for particle overlap with any body tagged `electronics: true` in the assembly. If detected:

1. Simulation **fails** with `ELECTRONICS_FLUID_DAMAGE`.
2. Agent receives: `"Electronics component '{label}' was contacted by fluid '{fluid_id}' at step {step}. Electronics destroyed."`

Until WP3 is implemented, this check is a no-op but the failure mode and event type are defined now for forward compatibility.

---

## Validation Metrics

We introduce a **Metrics Engine** that runs per-step analysis during simulation. These metrics are first-class objective types in `objectives.yaml`:

### Metric: Fluid Containment

```yaml
objectives:
  fluid_objectives:
    - type: "fluid_containment"
      fluid_id: "water_fill"
      containment_zone:         # Geometric boundary
        min: [x, y, z]
        max: [x, y, z]
      threshold: 0.95           # Fraction of particles that must stay inside
      eval_at: "end"            # "end" | "continuous"
```

**Implementation**: Count particles $P_{in}$ inside containment zone at $t = T_{end}$. Pass if $P_{in} / P_{total} \geq threshold$.

### Metric: Flow Rate

```yaml
objectives:
  fluid_objectives:
    - type: "flow_rate"
      fluid_id: "water_fill"
      gate_plane:
        point: [200, 0, 0]     # Point on the plane
        normal: [1, 0, 0]       # Outward normal
      target_rate_l_per_s: 1.0
      tolerance: 0.2            # ±20%
```

**Implementation**: Count particles crossing gate plane per second. Convert particle count to volume flow rate via particle mass / density.

### Metric: Max Stress (Soft Body / Any FEM Part)

```yaml
objectives:
  stress_objectives:
    - type: "max_stress"
      part_label: "gripper_pad"
      max_von_mises_mpa: 1.0   # Must not exceed
```

**Implementation**: Query FEM node stress tensors; compute max von Mises stress each step. Fail if threshold exceeded.

---

## Schema Changes

### `objectives.yaml` additions

```yaml
# New top-level section
physics:
  backend: "genesis"          # "mujoco" | "genesis"
  fem_enabled: true
  compute_target: "cpu"       # "cpu" | "gpu"

# New sections alongside existing objectives
fluids: [...]                 # Fluid definitions (see above)
objectives:
  fluid_objectives: [...]     # Fluid-based success criteria
  stress_objectives: [...]    # Stress-based success criteria
```

### `preliminary_cost_estimation.yaml` (No Changes)

We do **not** add explicit FEM fields to `preliminary_cost_estimation.yaml`.

- FEM properties are inherent to the material selected in `manufacturing_config.yaml`.
- If a part's material is "rigid", it simulates as rigid (or linear FEM) - which is the case for only off-the-shelf, not manufactured parts.
- If "elastomer", it simulates as hyperelastic.
- Stress monitoring is enabled by default for all parts in WP2.

### `SimulationResult` additions

```python
class FluidMetricResult(BaseModel):
    metric_type: str                # "fluid_containment" | "flow_rate"
    fluid_id: str
    measured_value: float
    target_value: float
    passed: bool

class SimulationResult(BaseModel):
    # ... existing fields ...
    # NEW
    stress_summaries: list[StressSummary] = []
    fluid_metrics: list[FluidMetricResult] = []
    failure_reason: str | None = None  # Extended with new enum values
```

New `failure_reason` values:

- `PART_BREAKAGE` — a manufactured part exceeded its ultimate stress
- `ELECTRONICS_FLUID_DAMAGE` — fluid contacted an electronics component (WP3)
- `STRESS_OBJECTIVE_EXCEEDED` — a stress objective was violated
- `FLUID_OBJECTIVE_FAILED` — a fluid containment/flow objective was not met

### Event Tracking

New events to add to the observability system (extending the 23+ events in the main spec):

| Event | Data |
|-------|------|
| `simulation_backend_selected` | `backend`, `fem_enabled`, `compute_target` |
| `part_breakage` | `part_label`, `stress_mpa`, `ultimate_mpa`, `location`, `step` |
| `fluid_containment_check` | `fluid_id`, `ratio`, `threshold`, `passed` |
| `flow_rate_check` | `fluid_id`, `measured_rate`, `target_rate`, `passed` |
| `stress_summary` | `part_label`, `max_von_mises`, `safety_factor` |
| `meshing_failure` | `part_label`, `error`, `retry_count`, `repaired` |
| `physics_instability` | `kinetic_energy`, `threshold`, `step` |
| `gpu_oom_retry` | `original_particles`, `reduced_particles` |

---

## Agent Tools

### New Engineer Tools

#### `define_fluid(...)` — Define a fluid type for simulation

```python
def define_fluid(
    name: str,
    viscosity: float = 1.0,        # Centipoise (cP)
    density: float = 1000,         # kg/m³
    surface_tension: float = 0.07, # N/m
    color: tuple = (0, 0, 255)
) -> FluidDefinition:
    """Defines a fluid type for use in the simulation."""
```

#### `get_stress_report(part_label: str)` — Query stress state of a part

```python
def get_stress_report(part_label: str) -> StressSummary:
    """Returns the current stress summary for a simulated FEM part.
    Available after simulation. Helps the agent identify overloaded regions
    and decide where to add material or change geometry."""
```

This implements the roadmap requirement: "Agents are able to analyze stresses in mesh and propose optimizations based on their view of the model."

#### `preview_stress(component, view_angles=...)` — Render stress heatmap

```python
def preview_stress(
    component: Part | Compound,
    view_angles: list[tuple[float, float]] | None = None
) -> list[str]:
    """Renders the component with a von Mises stress heatmap overlay.
    Returns paths to rendered images. Only available after simulation."""
```

### Modified Existing Tools

#### `simulate(...)` — Extended

```python
def simulate(
    component: Compound,
    render: bool = False,
    # NEW parameters
    fem_enabled: bool | None = None,   # Override objectives.yaml setting
    particle_budget: int | None = None # Cap particle count (for testing)
) -> SimulationResult:
    """Runs simulation. Now returns stress_summaries and fluid_metrics
    in the result. If any part breaks, returns immediately with
    failure_reason='PART_BREAKAGE'."""
```

#### `validate_and_price(...)` — Extended

Existing validation gains one new check:

- If `fem_enabled` and `physics.backend == "genesis"`: verify that all parts' `material_id` has FEM fields in `manufacturing_config.yaml`. Fail early with a clear message if a material is missing FEM data.

### Agent Behaviour Changes

#### Engineer Planner

- Must consider material strength when designing mechanisms. Plans should include a "Stress Considerations" subsection in `## 5. Risk Assessment` noting which parts are load-bearing and their expected safety factor.
- Relies on material properties for FEM checking; does not need to toggle flags in `preliminary_cost_estimation.yaml`.

#### Engineer CAD

- After simulation, should call `get_stress_report()` on load-bearing parts to check safety factors.
- Must balance material usage: add material where stress is high (safety factor < 1.5), remove where it's low (safety factor > 5.0). This prevents both breakage *and* overdesign.
- Receives structured breakage notifications via `SimulationResult` (not just generic errors).

#### Engineer Reviewer

- Must check that FEM stress results are reasonable (no parts at >80% utilization without justification).
- Must verify fluid containment metrics pass for fluid benchmarks.
- Gets new review criteria in YAML frontmatter: `stress_check: passed|warning|failed`.

#### Benchmark Planner

- Can now create fluid-based benchmarks with `fluid_objectives` in `objectives.yaml`.
- Must specify `physics.backend: genesis` for any benchmark involving fluids or requiring FEM.
- Can define initial fluid volumes and containment/flow objectives.

---

## Error Handling

### 1. Meshing Failures ("The Non-Manifold Nightmare")

**Scenario**: The engineer generates a self-intersecting CAD model. `build123d` exports it, but TetGen crashes.

**Handling**:

1. `mesh_utils.py` catches TetGen's non-zero exit code.
2. Runs a mesh repair pass (using `trimesh`): `remove_duplicate_faces()`, `repair_self_intersections()`.
3. Retries TetGen.
4. If still fails, task is marked `FAILED_ASSET_GENERATION`. Agent receives: `"Your geometry is topologically invalid. Please check for self-intersections near ({x}, {y}, {z})."`

### 2. Physics Instability ("The NaN Event")

**Scenario**: Simulation becomes unstable due to huge forces or bad timesteps.

**Detection**: Monitor total kinetic energy. If $E_k > E_{threshold}$, abort.

**Handling**:

1. Abort simulation.
2. Return `SimulationResult(success=False, failure_reason="PHYSICS_INSTABILITY")`.
3. Suggest to agent: `"Reduce timestep or increase mesh density."`

### 3. GPU OOM

**Scenario**: Too many particles for available VRAM.

**Handling**:

1. Genesis throws CUDA OOM.
2. System catches and auto-retries with fewer particles (25% reduction -> 75% of original).
3. Adds warning to result: `"Simulation resolution was reduced to fit VRAM. Results may be less accurate."`
4. Emits `gpu_oom_retry` event.

### 4. Part Breakage (new)

**Scenario**: A manufactured part exceeds its ultimate stress during simulation.

**Handling**:

1. Simulation stops immediately at the offending step.
2. Stress field at failure is captured.
3. `SimulationResult` includes the breakage location, stress value, and the part label.
4. Agent is expected to either strengthen the part (add material, change geometry) or change the material.

---

## Data Storage

Fluid simulations produce large data volumes. Strategy:

1. **Transient**: Raw particle data lives in `/tmp` on the worker. A 10-second simulation with 100k particles generates GBs.
2. **Processed**: Rendering and metric computation happen *on the worker* immediately after simulation.
3. **Persisted**: Only **MP4 video**, **JSON summary metrics**, and **stress summaries** are uploaded to S3.
4. **Deleted**: Raw particle cache is wiped after upload.

---

## Compute Configuration

Rather than splitting into separate microservices prematurely, we use a **configuration flag** on the existing worker:

```yaml
# Worker config
simulation:
  backend: "genesis"      # Default backend
  compute_target: "auto"  # "auto" | "cpu" | "gpu"
  max_particles: 100000   # GPU default
  smoke_test_mode: false  # If true: cap particles to 5000, label results as approximate
```

- **`auto`**: Checks `torch.cuda.is_available()`. Uses GPU if available, CPU otherwise.
- **`smoke_test_mode`**: For CI/CD and local development without GPU. Results are labelled `"approximate"` in `SimulationResult.confidence` and cannot be used for final validation. This is explicitly *not* a "CPU fallback" — it's a smoke test that checks the pipeline runs end-to-end, not that physics are accurate.
- **Docker**: A single `Dockerfile` with optional CUDA layers. No separate `Dockerfile.cpu`.

When the workload justifies it (multiple concurrent GPU simulations), the worker can be scaled horizontally with standard container orchestration. No new microservice types needed.

---

## Evaluation Framework Integration

### New Fast Evals

1. `objectives.yaml` with `physics` section passes schema validation — 100% of cases.
2. `manufacturing_config.yaml` materials have all FEM fields if `fem_enabled: true` — 100%.
3. Fluid objectives schema is valid (required fields present, thresholds numeric) — 100%.

### New Medium Evals

1. Given a fluid benchmark, the engineer designs a solution that passes fluid containment in 70% after 30 turns.
2. Given a stress-constrained benchmark, the engineer produces parts with safety factor > 1.2 in 80% of cases.
3. The planner includes "Stress Considerations" in risk assessment for FEM-enabled benchmarks in 90% of cases.
4. The reviewer catches parts with utilization > 80% (max_stress/yield_stress) and flags them in 70% of cases.

### New Slow Evals

1. End-to-end fluid benchmark: planner designs a fluid challenge → engineer solves it → passes containment metric — 50% success rate target initially.
2. Breakage prevention: engineer solutions do not cause `PART_BREAKAGE` failure in 90% of cases.
3. No overdesign: average safety factor across all parts is between 1.5 and 5.0 in 80% of solutions.

---

## Delivery (Frontend)

### Simulation Results Viewing

Users can view simulation results in the frontend, extending existing video/render infrastructure:

1. **Fluid visualization**: Particle-based rendering in the simulation video (Genesis handles this natively in the MP4 output).
2. **Stress heatmaps**: Rendered images with von Mises stress colour overlay (blue = low, red = high), available alongside existing 24-view renders.

### Future Delivery (extra, not strictly required)

- Interactive 3D viewer with stress colouring (depends on WASM CAD viewer from main spec).
- Per-part deformation animation scrubber.
- Fluid particle replay.

These depend on the frontend 3D viewer infrastructure defined in the main spec and are not part of the WP2 MVP.

---

## Benchmark Generator Changes

The benchmark planner and CAD engineer gain the ability to create fluid-based and stress-based benchmarks:

### New Benchmark Types

1. **Fluid containment**: "Design a container that holds 95% of water under vibration."
2. **Fluid transport**: "Design a channel that delivers 1 L/s to the target."
3. **Stress-limited**: "Support this load without any part exceeding 80% utilization."
4. **Hybrid**: "Transport fluid through a mechanism without spilling."

### Benchmark Planner Output

`objectives.yaml` gains the new `physics`, `fluids`, `fluid_objectives`, and `stress_objectives` sections as defined above. The benchmark planner must specify these when creating fluid/stress benchmarks. Existing rigid-body benchmarks remain unchanged (they use `backend: mujoco` or `backend: genesis, fem_enabled: false`).

---

## Migration Path

### Phase 1: Backend abstraction (no behaviour change)

1. Implement `PhysicsBackend` protocol.
2. Wrap existing MuJoCo code behind it.
3. Add `physics.backend` to `objectives.yaml` schema. Default: `"mujoco"`.
4. All existing benchmarks continue to work unchanged.

### Phase 2: Genesis rigid-body (validation)

1. Implement Genesis backend (rigid-body only, `fem_enabled: false`).
2. Run existing benchmarks on both backends. Compare results.
3. Fix discrepancies until parity is achieved.

### Phase 3: FEM + fluids

1. Add FEM fields to `manufacturing_config.yaml` materials.
2. Enable `fem_enabled: true` for new benchmarks.
3. Add tetrahedralization to asset pipeline.
4. Implement breakage detection and stress reporting.
5. Add fluid simulation support and fluid objectives.

### Phase 4: Agent adaptation

1. Update agent prompts and skills for stress reasoning.
2. Create fluid benchmark dataset (5-10 benchmarks).
3. Run evals and iterate on prompt quality.

---

## Tech Stack

- **Genesis**: Primary physics engine for FEM + fluids. Python package `genesis`.
- **TetGen**: Tetrahedral meshing of parts for FEM. Wrapped by `worker/utils/mesh_utils.py`.
- **trimesh**: Mesh repair for non-manifold geometry before TetGen.
- **PyVista**: Debug visualization of meshes and stress fields (development only).
- **CUDA**: Optional but strongly recommended for fluid simulations with >5k particles.

---

## Limitations (MVP)

1. **Surface tension accuracy**: MPM models surface tension approximately. Small-scale capillary action may be inaccurate.
2. **Phase change**: No boiling or freezing.
3. **Chemical reactions**: Fluids do not mix chemically.
4. **Linear FEM for rigid materials**: We use linear (not nonlinear) FEM for `material_class: "rigid"`. This is inaccurate for plastic deformation but sufficient for detecting breakage.
5. **No pipes/hoses** (extra from roadmap): Fluid transport through enclosed channels is not modelled. Fluids flow freely in the simulation domain. This may be added later if needed.
6. **Fastener strength not simulated**: Fastener connections remain rigid (weld constraints). Fastener failure under FEM load is not modelled in this WP.

---

## Implementation Status (Feb 2026)

- **Backend Abstraction**: `PhysicsBackend` protocol implemented in `shared/simulation/backends.py`.
- **MuJoCo Backend**: Fully implemented in `worker/simulation/mujoco_backend.py`.
- **Genesis Backend**: Skeleton with scene loading and basic entity mapping in `worker/simulation/genesis_backend.py`.
- **Deformable Materials**: FEM fields added to `manufacturing_config.yaml`. Linear FEM support via `gs.morphs.SoftMesh`.
- **Tetrahedralization**: Intermediate asset pipeline added using TetGen via `worker/utils/mesh_utils.py`.
- **Stress Reporting**: `StressSummary` implemented in `SimulationLoop`.
- **Breakage Detection**: Automatic simulation abort on `ultimate_stress_pa` violation.
- **Fluid Support**: MPM definitions added to `ObjectivesYaml`.
- **Validation Metrics**: `fluid_containment` and `max_stress` objective evaluation added to `SimulationLoop`.
- **Agent Tools**: `define_fluid`, `get_stress_report`, and `preview_stress` added to `worker/utils/validation.py`.
