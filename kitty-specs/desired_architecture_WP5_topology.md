# Desired Architecture: WP5 - Topology Optimization

## Objective of the system

To leverage computational algorithms (Generative Design) to create optimal structures that minimize weight while maximizing stiffness/strength. The agent moves from "Designer" to "Constraint Definer".

### Outputs and end goals

1. **Automated Weight Reduction**: Remove material where it isn't needed, creating lightweight organic structures.
2. **Complex Geometry Handling**: Generate shapes that are difficult to model manually but highly efficient.
3. **Manufacturability Awareness**: Ensure the optimized shape can actually be 3D printed (overhangs) or cast (draft angles).
4. **Integration**: Seamlessly integrate the optimized part back into the larger assembly.

## Workflow: The Generative Design Loop

The workflow fundamentally shifts from "Direct Modeling" to "Boundary Value Problem Definition".

1. **Define Design Space**: The agent creates a "Block" of material (the maximum volume).
2. **Define Non-Design Space**: The agent defines "Keep Out" zones (where parts move) and "Must Keep" zones (bolt holes, mounting surfaces).
3. **Define Loads & Constraints**: The agent applies forces (vectors) and constraints (fixed points).
4. **Run Solver**: The Topology Optimization (TO) solver iterates to remove material based on the density field method (SIMP).
5. **Reconstruct**: The resulting Voxel/Mesh data is smoothed and converted back to a B-Rep (or kept as a mesh) for final usage.

## The Solver Infrastructure

We integrate an **Open Source TO Solver**. We choose **ToPy** (Python-based topology optimization) or a custom implementation on top of **FEniCS** (Finite Element Method).

### Solver Input

A strictly typed JSON or Python Configuration object:

```python
problem = {
    "domain_size": [100, 50, 20], # mm
    "voxel_size": 1.0, # mm
    "materials": {"E": 210e9, "nu": 0.3}, # Steel
    "loads": [
        {"position": [100, 25, 10], "vector": [0, -1000, 0]} # 1kN down
    ],
    "supports": [
        {"position": [0, 0, 0], "type": "fixed"} # Wall mount
    ],
    "passive_regions": [
        {"type": "keep", "box": [[0,0,0], [10, 50, 20]]} # Keep the mount
    ],
    "target_volume_fraction": 0.3 # Keep 30% of material
}
```

### Solver Output

* **Density Field**: A 3D array of floats (0.0 to 1.0) representing material density.
* **Compliance Trace**: Graph of stiffness optimization over iterations.

## Infrastructure & Compute

TO is extremely compute-heavy (solving FEM matrices thousands of times).

* **Offloading strategy**: TO jobs are **never** run on the standard worker queue.
* **Dedicated Nodes**: We dispatch these to high-CPU/Memory nodes (likely AWS Spot Instances or a dedicated HPC queue).
* **Asynchronous Pattern**:
    1. Agent calls `submit_topology_job(config)`.
    2. System returns `job_id`.
    3. Agent goes to sleep (suspend workflow).
    4. Temporal wakes up the workflow when the job completes (hours later).

## Post-Processing (The "Smoothing" Step)

The raw output of TO is a jagged voxel map. We need to smooth it to make it usable.

1. **Isosurface Extraction**: We use Marching Cubes (via `scikit-image` or `PyVista`) to extract the 0.5 density isosurface.
2. **Laplacian Smoothing**: We iteratively smooth the mesh vertices to remove "stairstep" artifacts.
3. **Mesh Decimation**: We reduce the polygon count while preserving features (e.g., `QuadricDecimation`).
4. **Validation**: We run a final FEA (Finite Element Analysis) pass on the *smoothed* mesh to verify it still holds the load.

## Agent Strategy

How does the LLM know how to set up the problem?

### Tools

* `create_design_space(compound)`: Retains the bounding box.
* `add_load(face_selector, force_vector)`: Applies force to a semantic feature.
* `add_support(face_selector)`: Fixes a face in space.

### Reasoning Pattern

1. **Identify Load Path**: "The force comes from the motor and goes to the gripper."
2. **Connect Points**: "I must connect the Motor Mount to the Gripper Mount."
3. **Avoid Obstacles**: "I must not intersect the battery."
4. **Formulate**: "Design Space = Bounding Box of (Motor + Gripper + Battery) - Battery."

## Tech Stack

* **FEniCS / SciPy**: For the underlying FEM solver. Reliable, mathematically rigorous.
* **ToPy**: A lightweight wrapper for topology optimization in Python.
* **PyVista**: For mesh post-processing, visualization, and IO (`.vtk`, `.stl`).
* **Temporal**: For long-running job orchestration. This is essential as jobs can take 1-12 hours.
* **Trimesh**: For mesh boolean operations and repair.

## Limitations (MVP)

1. **Isotropic Only**: We assume materials are isotropic (metal, plastic). We do not support composite layups (CFRP).
2. **Single Physics**: We optimize for structural compliance only. No thermal-fluid-structural coupling yet.
3. **Mesh Output**: We return an STL (Mesh). Converting this back to a clean Parametric STEP (B-Rep) is an open research problem ("Reverse Engineering") and is out of scope. The agent must start treating STLs as first-class citizens in `build123d`.

## Future Work

1. **Hybrid Manufacturing Constraints**: "Must be millable from 3-axis" (Draft angles, no undercuts).
2. **Lattice Structures**: Instead of solid/void, generating variable-density lattice infills (gyroids) for SLA printing.
3. **AI Accelerators**: Using Neural Networks (CNNs) to predict the TO result in seconds as a "preview" before running the full solver.
