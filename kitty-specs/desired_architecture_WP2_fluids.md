# Desired Architecture: WP2 - Fluids & Deformable Materials

## Objective

To extend the system's capability from rigid-body dynamics to verifying designs involving fluids (liquids, gases) and deformable materials (soft bodies, cloth).

### Goals

1. **Fluid Simulation**: Verify designs that must contain, transport, or mix fluids (e.g., a cup, a pipe, a mixer).
2. **Soft Body Interaction**: Verify designs that interact with deformable objects (e.g., a gripper holding a rubber duck).
3. **Hybrid Interaction**: Simulation of rigid mechanisms interacting with fluids (e.g., a pump).

## Physics Engine Upgrade

We transition our simulation backend from **MuJoCo** to **Genesis**.

### Why Genesis?

* **Unified Multi-Material Support**: Genesis natively creates interactions between rigid bodies, MPM (Material Point Method) fluids, and FEM (Finite Element Method) soft bodies.
* **Differentiable**: It allows for gradient-based optimization in the future (though we don't use it in this phase).
* **Python-Native**: Configuration is purely Pythonic, avoiding XML hell (unlike MuJoCo MJCF).

<!-- note: MuJoCo continues to be used for pure rigid-body benchmarks due to its speed. The `SimulationService` will dispatch to MuJoCo or Genesis based on the `benchmark_type` tag. -->

## Simulation Pipeline

### Asset Conversion

The pipeline for converting CAD assertions to Simulation assets becomes more complex:

1. **Rigid Parts**:
    * Source `build123d` (BREP).
    * Process: Tesselate to STL/OBJ (High resolution for visual, low for collision).
    * Genesis Loading: `gs.morphs.Mesh(file='part.obj')`.
2. **Soft Bodies**:
    * Source: `build123d` (BREP).
    * Process: Tesselate to Surface Mesh (STL) -> **Tetrahedralization** (using `TetGen` or `fTetWild`).
    * Output: `.vtk` or `.msh` file containing volumetric tet-mesh.
    * Genesis Loading: `gs.morphs.SoftMesh(file='part.msh')`.
3. **Fluids**:
    * Source: A volume defined in B123D (e.g., a cylinder representing filled water).
    * Process: **Particle Sampling**. We define a grid of particles inside the volume.
    * Genesis Loading: `gs.morphs.Box(pos=..., material=gs.materials.Liquid())` ( Genesis handles particle seeding automatically for primitives).

### Agent Tools

The Engineer agent gains new tools in `simulate()` to define the physics of the world.

#### `define_fluid(...)`

```python
def define_fluid(
    name: str,
    viscosity: float = 1.0, # Centipoise (cP)
    density: float = 1000, # kg/m^3
    surface_tension: float = 0.07, # N/m
    color: tuple = (0, 0, 1)
):
    """Defines a fluid type for use in the simulation."""
    ...
```

#### `define_soft_material(...)`

```python
def define_soft_material(
    name: str,
    youngs_modulus: float, # Pa
    poissons_ratio: float,
    density: float
):
    """Defines a hyperelastic material (Neo-Hookean model)."""
    ...
```

## Validation Criteria & Metrics

Simulation success is now defined by more complex states than "AABB containment". We introduce a **Metrics Engine** that runs frame-by-frame analysis during simulation.

### Metric: Fluid Containment

* **Goal**: "Keep at least 95% of water inside the cup."
* **Implementation**:
    1. Define a geometric zone (the "Cup Interior").
    2. Count particles $P_{in}$ inside usage zone at $t=T_{end}$.
    3. Count total particles $P_{total}$.
    4. Ratio $R = P_{in} / P_{total}$.
    5. Pass if $R > 0.95$.

### Metric: Flow Rate

* **Goal**: "Transport 1L/s through the pipe."
* **Implementation**:
    1. Define a "Gate Plane" at the pipe exit.
    2. Count particles crossing the plane per second.
    3. Convert particle count to volume flow rate (based on particle mass/density).

### Metric: Max Stress (Soft Body)

* **Goal**: "Don't crush the rubber duck (Max von Mises stress < 1MPa)."
* **Implementation**:
    1. Query FEM node stress tensors at each step.
    2. Compute max Von Mises stress.
    3. Fail if threshold exceeded.

## Infrastructure Impact

### GPU Requirement

Fluid (MPM) and Soft-body (FEM) simulations are significantly more compute-intensive than rigid body. CPU execution is non-viable (minutes to hours). GPU execution brings this down to seconds/minutes.

* **Hardware**: Workers running WP2 tasks **must** have GPU acceleration (NVIDIA CUDA). 8GB VRAM minimum.
* **Task Routing**: The Controller's `TaskQueue` must be split:
  * `queue_cpu`: Rigid body tasks (MuJoCo).
  * `queue_gpu`: Fluid/Soft tasks (Genesis).
* **Docker Configuration**:
  * The `worker` container image must include `nvidia-container-toolkit`.
  * Base image changes from `python:3.12-slim` to `nvidia/cuda:12.x-runtime-ubuntu22.04`.

### Data Storage

* **Particle Caches**: A 10-second simulation with 100k particles generates GBs of data. We **cannot** store this permanently.
* **Artifact Strategy**:
    1. **Transient**: Simulation raw data lives in `/tmp` on the worker.
    2. **Processed**: Rendering happens *on the worker* immediately after simulation.
    3. **Persisted**: Only the **MP4 Video** and the **JSON Summary Metrics** are uploaded to S3/Postgres.
    4. **Deleted**: Raw cache is wiped after upload.

## Workflows

### 1. Benchmark Generation (The "Problem Setter")

The Generator Agent now places "Liquid Sources" and "Liquid Sinks" in the environment.

* *Task*: "Design a nozzle that sprays water into a 5cm target hole 20cm away."
* *Output*: XML/Python script defining:
  * Source: Emitter at $(0,0,0)$, velocity $(1,0,0)$.
  * Goal: Metric `FlowThroughGate` at $(0.2, 0, 0)$.

### 2. Engineering (The "Solver")

The Engineer Agent designs the nozzle.

* *Reasoning*: "I need to narrow the stream. I will create a cone shape."
* *CAD*: Generates the internal geometry.
* *Simulate*: Runs the simulation locally (if GPU available) or submits to queue.

### 3. Verification

The system runs the official verification.

* Code checks if fluid particles reached the Sink.
* Video is generated for human review.

## Tech Stack Details

* **Genesis**: The primary physics engine source.
  * We use the `genesis` python package.
* **TetGen**: For tetrahedral meshing of soft bodies.
  * We wrap this internal binary with a python helper `mesh_utils.py`.
* **CUDA**: Hard dependency for reasonable simulation times.
* **PyVista**: Used for debugging visualization of meshes before simulation (sanity check).

## Limitations (MVP)

1. **Surface Tension Accuracy**: MPM models surface tension approximately. Small-scale capillary action might be inaccurate.
2. **Phase Change**: No boiling or freezing logic.
3. **Chemical Reactions**: Fluids do not mix chemically, they just occupy space.

## Error Handling & Edge Cases

Simulations fail. The architecture must handle this gracefully.

### 1. Meshing Failures ("The Non-Manifold Nightmare")

* **Scenario**: The Engineer generates a self-intersecting CAD model. `B123d` exports it, but `TetGen` crashes.
* **Handling**:
    1. The `AssetConverter` catches the `TetGen` non-zero exit code.
    2. It runs a `MeshRepair` pass (using `PyMeshLab` or `Trimesh`): `remove_duplicate_faces()`, `repair_self_intersections()`.
    3. Retry TetGen.
    4. If it still fails, the Task is marked `FAILED_ASSET_GENERATION` and the Agent receives a log: "Your geometry is topologically invalid. Please check for self-intersections."

### 2. Physics Explosions ("The NaN Event")

* **Scenario**: The simulation becomes unstable (energetic explosion) due to huge forces or bad timesteps.
* **Detection**: The `SimulationRunner` monitors the total kinetic energy of the system. If $E_k > E_{threshold}$, it aborts.
* **Handling**:
    1. Abort simulation.
    2. Return `SimulationResult(success=False, error="Physics Instability detected")`.
    3. Suggest to Agent: "Reduce `dt` (timestep) or increase particle density."

### 3. GPU OOM

* **Scenario**: The Agent requests too many particles for the 8GB VRAM.
* **Handling**:
    1. `Genesis` throws `CUDA OOM`.
    2. The System catches this.
    3. It auto-retries with a **Lower Resolution** (fewer particles).
    4. It adds a warning to the Report: "Simulation resolution was reduced to fit VRAM."

## Future Calibration

To ensure our "Digital Twin" matches reality:

1. **Physical Benchmarking**: We will build a real "Water into Cup" rig.
2. **Sim-to-Real**: We compare the real video with the sim video.
3. **Parameter Tuning**: We use the **Optimizer Agent** (WP1) to tune `viscosity` and `friction` parameters until Sim matches Real.
