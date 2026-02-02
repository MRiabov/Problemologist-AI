# Problemologist Project Ideas & Roadmap

This document tracks future features, infrastructure components, and research directions identified during the specification analysis. These items complement the core "Design-Simulate" loop (Specs 001, 002, 003).

## 1. Core Extensions

### 004-manufacturing-workbenches

* **Status**: Missing
* **Priority**: **Critical** (Defined in Scope)
* **Description**: Adds domain-specific constraints beyond simple 3D printing. The agent must design parts that are actually manufacturable.
* **Key Functionality**:
  * **Injection Molding**: Undercut detection (raycasting/voxels), Draft angle validation (normals check), Wall thickness uniformity.
  * **Sheet Metal**: Unfolding validation, bend radius checks.
  * **Cost Modeling**: Real-time cost estimation based on material volume + process complexity.
  <!-- dev note: yes, let's do it.-->

### 005-benchmark-scenarios

* **Status**: Missing
* **Priority**: **High**
* **Description**: The actual "Levels" or "Exams" the agent must pass. Defines the scientific metrics for the project.
* **Key Functionality**:
  * **Tier 1 (Spatial)**: Peg-in-Hole, Gravity Trap (Catching).
  * **Tier 2 (Kinematic)**: Class 1 Lever (Force balancing), Crank-Slider (Motion conversion).
  * **Tier 3 (Integrated)**: Motor Mounting Bracket, Under-actuated Gripper.
  <!-- dev note: I'm not sure about tier 3... it's kind of.. off. I think I'll leave it. Spatial and kinematic go first at least + we don't have the simulator that can actually compute FEM for something like motor mounting bracket (yet. But still.) -->
  * **Artifacts**: Pre-built MJCF templates with defined "Start", "Build" (may be merged with Start") "Goal", and "Forbid" zones.
  <!-- dev note: YES! the last time, I had to do it by hand. However ...-->

## 2. Infrastructure & Tooling

### 006-dataset-pipeline ("The Data Distillery")

* **Status**: Missing
* **Priority**: **Critical** (For RL/Fine-tuning)
* **Description**: A pipeline to convert raw SQL session logs into clean training datasets. Turns the environment into a "Data Engine."
* **Key Functionality**:
  * **Filtering**: Discard/Flag broken code or failed simulations.
  * **Formatting**: Export to SFT (User->Code) and DPO (Prompt->[Win, Lose]) formats.
  * **Versioning**: Manage dataset snapshots (e.g., `problemologist-v0.1.jsonl`).
  <!-- dev note: I'll need to research on how to do it best. -->

### 007-visual-dashboard ("Human-in-the-Loop")

* **Status**: Missing
* **Priority**: **High** (For Debugging)
* **Description**: A Web UI (Streamlit/React) to visualize the agent's thought process and simulation results. Text logs are insufficient for 3D debugging.
* **Key Functionality**:
  * **Live Preview**: Real-time rendering of `build123d` geometry as it's generated.
  * **Replay Theater**: Interactive timeline scrubbing for MuJoCo simulation recordings.
  * **Intervention**: Allow humans to edit generated code and "fix" the agent, creating "Gold Standard" data.
  <!-- dev note: yes, definitely. Ideally a streamlit repo with some customization for viewing 3d models and images, maybe (streamlit is customizable) -->

### 008-execution-sandbox

* **Status**: Missing
* **Priority**: **High** (Safety)
* **Description**: Isolates the agent's generated Python code to prevent system damage or resource exhaustion.
* **Key Functionality**:
  * **Containerization**: Docker or Firecracker microVMs for every code execution.
  * **Resource Limits**: Hard caps on RAM (e.g., 4GB) and CPU Time (e.g., 10s).
  * **Air-Gapping**: Network isolation for the execution environment.
  <!-- dev note: I think it exists... -->

### 009-component-library ("Knowledge Base")

* **Status**: Missing
* **Priority**: **Medium**
* **Description**: A semantic database of high-quality CAD snippets. Instead of writing every screw from scratch, the agent retrieves standard patterns.
* **Key Functionality**:
  * **Ingestion**: Indexing `build123d` recipes (Gears, Fasteners, Hinges).
  * **Skill Promotion**: Automatically saving successful agent designs (e.g., a working linkage) back into the library for future reuse.
  * **Semantic Search**: Embedding-based retrieval (Vector DB).
  * **Build123d Documentation Reference**: A condensed, LLM-friendly reference (`llms.txt`) of `build123d` syntax and best practices to minimize hallucination.
  <!-- DEV NOTE: yes - also a library called bd_warehouse exists with all of those snippets. https://bd-warehouse.readthedocs.io/en/latest/ -->

<!-- ### 010-cloud-infrastructure

* **Status**: Missing
* **Priority**: **Medium** (Optimization)
* **Description**: Scaling the simulation backend to the cloud using Skypilot.
* **Key Functionality**:
  * **Remote Execution**: Offloading heavy MuJoCo/FEM simulations to cloud workers.
  * **Parallelism**: Running 100 simulations in parallel for RL training. -->

### 011-cots-assembly-system

* **Status**: Missing
* **Priority**: **High** (Realism)
* **Description**: A system to handle "Commercial Off-The-Shelf" parts reliably. Prevents the agent from hallucinating arbitrary dimensions for real-world items (motors, bearings, screws).
* **Key Functionality**:
  * **Digital Twin Catalog**: Parametric definitions of standard parts (NEMA motors, 608 bearings) with rigorous metadata (mass, torque, dimensions).
  * **Port-Based Mating**: Every COTS part exposes named "Ports" (Coordinate Systems) for attachment (e.g., `motor.mount_face`, `motor.shaft`).
  * **Virtual Fasteners**: Helper tools to "drill" matching holes in custom parts (`bracket -= motor.hole_pattern`) and generate `<weld>` constraints in MuJoCo to simulate screws. -->
  <!-- DEV NOTE: yes - also a library called bd_warehouse exists with all of those snippets. https://bd-warehouse.readthedocs.io/en/latest/ -->

### 012-topology-optimization-module

* **Status**: Missing
* **Priority**: **Medium** (Advanced Design)
* **Description**: Integrates algorithmic structural optimization. Instead of the agent manually sketching ribs, it defines load cases and lets an optimizer "grow" the ideal geometry.
* **Key Functionality**:
  * **Load Case Definition**: Domain-specific DSL for the agent to specify fixed supports and applied forces (e.g., `fix(face_a)`, `apply_force(face_b, direction=(0,0,-1), magnitude=100N)`).
  * **SIMP Integration**: Lightweight Solid Isotropic Material with Penalization (SIMP) framework for 2D/3D density-based optimization (e.g., using `TopOpt` or `jax-topopt`).
  * **Meshing to B-Rep**: Pipeline to convert voxel/density clouds back into smooth `build123d` Solids (via Marching Cubes + Surface Fitting).
  * **Material Efficiency**: Automatic mass reduction while maintaining safety factors.
  <!-- DEV NOTE: yes - several open-source frameworks like `TopOpt` (Python), `FEniCS`, or `NLopt` can be integrated. This turns the agent from a "drafter" into a "requirements engineer" who defines the problem space rather than the solution geometry. -->
