# Specification: Benchmark Scenario Generator

**Feature**: 005-benchmark-scenario-generator
**Status**: Draft
**Mission**: software-dev

## 1. Introduction

The **Benchmark Scenario Generator** is an automated pipeline designed to rapidly populate the `Problemologist` benchmark suite. Instead of manually authoring 40+ unique physics puzzles (a process that previously took days), this system leverages an LLM to procedurally generate parametric Python scripts. These scripts define the "Levels" the agent must solve—ranging from simple spatial tasks (Peg-in-Hole) to complex kinematic assemblies (Cranks/Levers).

The system ensures that every generated scenario is robust, randomized, and physically valid before it reaches the human-in-the-loop for final approval.

### 1.1 Goals

- **Acceleration**: Reduce the time to create a valid benchmark scenario from hours to minutes.
- **Diversity**: Generate high variance in problem geometry, goal positions, and obstacle placement ("forbid zones") to prevent overfitting.
- **Validity**: Ensure all generated scenarios are geometrically consistent (watertight meshes) and simulation-ready (loadable MJCF).
- **Automation**: Provide a "Generate → Compile → Validate" loop that only requires human intervention at the final review stage.

### 1.2 Out of Scope

- **Runtime Generation**: This is a _design-time_ tool to build the dataset, not a feature for the end-user agent to generate its own problems at runtime (yet).
- **Tier 3 Scenarios**: Complex integrated mechanisms (Motor Mounts) are out of scope for the MVP; focus is on Spatial and Kinematic tiers.
- **Matplotlib Hallucination**: We will not use 2D plot hallucination for validation; we rely on direct 3D generation and rendering.

## 2. User Scenarios

### 2.1 The "Dataset Architect" Flow

**Actor**: Main Developer / Dataset Curator
**Goal**: Populate the "Kinematic" benchmark tier with 10 new lever-based puzzles.

1. **Prompting**: The user invokes the generator with a high-level intent: "Generate 10 variations of a Class 1 Lever problem where the fulcrum position varies."
2. **Generation**: The system's LLM drafts 10 distinct Python scripts. Each script utilizes `build123d` for geometry and `mujoco` logic for physics constraints.
3. **Compilation & Validation**: The system automatically runs each script:
    - Generates STL meshes for the base, lever, and load.
    - Generates the MJCF (XML) file.
    - Checks for mesh manifoldness and XML syntax errors.
    - Simulates 100 steps to ensure the scene doesn't explode immediately (stability check).
4. **Review**: The user opens the review interface (CLI or future Dashboard), sees 8 successes and 2 failures.
5. **Approval**: The user visually inspects the 8 successful renders. They approve 6 that look challenging but solvable.
6. **Commit**: The approved scenarios are saved to `datasets/benchmarks/tier2_kinematic/` with their randomization parameters locked.

### 2.2 Interactive Human-in-the-Loop Generation

**Actor**: Main Developer
**Goal**: Precisely tune a complex benchmark scenario through iterative feedback.

1. **Initial Prompt**: User describes a benchmark goal (e.g., "A robotic task to teach gear meshing").
2. **Plan Approval**: The system generates a high-level plan (teaching goals, rough geometry, kinematic chain, self-collision strategy). The user reviews and edits this plan.
3. **Iterative CAD Coding**: The system generates the `build123d` code. It self-validates through MuJoCo simulation until a stable model is found.
4. **Visual Review**: The system provides multiple rendering angles of the stable model. The user reviews the code and visuals, potentially providing manual edits.
5. **Fulfillment**: Once the user approves the CAD model, the system generates the final MJCF XML and manifests.

## 3. Functional Requirements

### 3.1 Scenario Script Generation

- **FR-01**: The system MUST generate standalone Python scripts that import `build123d` and define a `build(seed: int, scale_factors: tuple[float, float, float]) -> str` function.
- **FR-02**: Generated scripts MUST be deterministic for a given seed and scale factor.
- **FR-03**: The `build()` function MUST return a valid MJCF (MuJoCo XML) string.
- **FR-04**: Scripts MUST include logic to generate "Forbid Zones" (red obstacles) and "Goal Zones" (green targets) within the workspace.

### 3.2 Interactive Pipeline (Human-in-the-Loop)

- **FR-17**: The system MUST support a multi-stage interactive generation loop: Plan -> Code -> XML.
- **FR-18**: The Plan stage MUST include explicit "Learning Objectives" and a "Self-Verification" strategy for collisions.
- **FR-19**: The system MUST allow the user to edit and override the Agent's Plan before implementation begins.
- **FR-20**: The system MUST support a "Self-Correction" loop where the Coder and Validator iterate up to 3 times to find a stable geometry without user intervention.
- **FR-21**: The system MUST allow the user to provide manual code edits or natural language feedback on the stable CAD model before final XML generation.

### 3.3 Randomization Engine

- **FR-05**: The system **MUST** be able to vary the "Domain Box" (workspace size) dimensions (Length/Width/Height).
- **FR-06**: The system **MUST** support randomization of start/goal/obstacle positions by at least **±40% of the workspace dimension** in which they are moving, as specified by the user.
- **FR-30**: **Overlap Prevention**: Randomized objects MUST NOT intersect with `zone_start` or `zone_goal` volumes. The system MUST perform a boolean intersection check during randomization and retry up to 5 times to find a clear position before failing.
- **FR-07**: Randomization logic **MUST** ensure objects remain within the global workspace bounds (no spawning outside the box).

### 3.4 Advanced Randomization (Rescaling)

- **FR-22**: **Random Rescaling**: After the core geometry is planned, the system MUST support randomly rescaling the entire environment by a factor of **0.5 to 2.0** independently in all three directions (X, Y, Z).
- **FR-23**: **Agent Override**: The Agent (Planner) MUST be able to specify custom rescaling limits for specific directions (e.g., "X: 0.8-1.2, Y: 0.5-5.0") based on the geometric constraints of the puzzle.
- **FR-24**: Rescaling MUST be applied at the CAD generation level to ensure all kinematic relationships (joints, pivots) remain consistent.
  - **Joint Scaling**: Joint positions and axes MUST be scaled proportionally with the bodies they connect to maintain kinematic integrity.
- **FR-29**: **Bound Enforcement**: If a rescale operation causes any body to exceed the **100x100x100mm** Domain Box, the scale factor MUST be automatically clipped to the maximum allowed value for that dimension.

### 3.5 Economic Constraints & Record System

- **FR-25**: **Economic Targets**: Every benchmark scenario MUST define a `target_quantity` (e.g., 1, 100, 10,000) and a `max_unit_cost` that the solution part must satisfy.
- **FR-26**: **Cost-Driven Optimization**: The system MUST support a "Record" system that stores the lowest unit cost achieved by any agent for a specific scenario.
  - **Persistence**: Records MUST be persisted in the shared `history.db` SQLite database using the `cost_records` table.
  - **Cost Model Integration**: Unit costs MUST be calculated using the workbench models defined in **Spec 004 (Advanced Manufacturing Workbenches)**.
- **FR-27**: **Iterative Rollouts**: The generator MUST be able to produce "Optimization Tasks" where the goal is to beat the current cost record.
  - **Quantification**: A record is considered "beaten" if the new unit cost is strictly lower than the existing record ($UnitCost_{new} < UnitCost_{record}$).
  - **Cold Start**: If no record exists for a scenario, the `max_unit_cost` defined in the scenario manifest serves as the initial threshold to beat.
  - **Context Delivery**: The generator MUST inject the current record cost into the agent's prompt if a record exists for the scenario (e.g., "Current Best: $4.25. Your goal: < $4.25").

### 3.6 Artifact Compilation

- **FR-08**: The system **MUST** execute the generated Python scripts to produce simulation assets.
- **FR-09**: It **MUST** export all solid parts as independent STL or OBJ files.
- **FR-10**: It **MUST** generate a valid MJCF (XML) file that references these meshes and defines the necessary joints/actuators.

### 3.7 Automated Validation

- **FR-11**: **Geometric Check**: The system **MUST** verify that generated meshes are watertight (manifold) and have non-zero volume.
  - **Volume Threshold**: Every solid part MUST have a volume $> 10 mm^3$.
  - **Workspace Bounds**: All generated geometry MUST reside within the **100x100x100mm** Domain Box (or the rescaled equivalent).
- **FR-12**: **Physics Check**: The system **MUST** attempt to load the generated MJCF in MuJoCo and run for 1 second (sim time).
  - **Simulation Parameters**: The check MUST use a default timestep of **0.002s**, the **RK4** integrator, and standard Earth gravity (**-9.81 m/s²** on Z).
- **FR-13**: **Stability Check**: A generation is marked as "Failed/Exploded" if any of the following occur during the 1s run:
  - **Velocity**: Maximum absolute velocity of any body exceeds **100.0 m/s**.
  - **Energy**: Total system energy (kinetic + potential) increases by more than **10%** between steps (detecting divergence).
  - **NaNs**: Any state vector component (position or velocity) becomes NaN.
- **FR-28**: **Kinematic Validity (Tier 2)**: Scenarios involving joints MUST:
  - Define explicit `range` limits for all hinge and slide joints.
  - Ensure that the "Base" components (fixed to world) do not intersect with the mobile components in the initial state.

### 3.8 Review & Export

- **FR-14**: The system **MUST** save successful runs to a staging directory.
- **FR-15**: It **MUST** generate a static preview image (snapshot) of the scene for the reviewer.
- **FR-16**: It **MUST** provide a CLI command to "promote" a staged scenario to the permanent benchmark suite.
  - **Promotion Criteria**: A scenario is eligible for promotion ONLY if it:
    1. Passes all **Geometric** and **Stability** checks.
    2. Has a valid `target_quantity` and `max_unit_cost` defined.
    3. Is visually inspected for "solvability" (e.g., goal isn't completely encased in an obstacle).
  - **Reference Solution**: Every promoted scenario SHOULD be accompanied by at least one "Reference Solution" CAD script that successfully achieves the goal zone.
  - **Duplicate Detection**: The promotion process MUST check for geometric duplication by comparing the SHA-256 hashes of the STL meshes against the existing suite. Duplicates MUST be rejected.

## 4. Technical Constraints

- **Language**: Python 3.10+
- **CAD Kernel**: `build123d` (essential for programmatic generation).
- **Physics Engine**: `mujoco` (standard for the project).
- **LLM Interface**: `langchain` or direct API calls (consistent with Agent implementation).
- **File Structure**:
  - `src/generators/`: The generation logic.
  - `scenarios/staging/`: Temporary hold for generated items.
  - `scenarios/benchmarks/`: Final home for approved levels.

## 5. Success Criteria

- **Efficiency**: Can generate and validate 10 candidate scenarios in under 5 minutes.
- **Yield Rate**: At least 50% of generated scripts compile and pass the stability check (Simulation Load + 1s run).
- **Coverage**: Successfully generates at least one scenario for **Tier 1 (Spatial)** and one for **Tier 2 (Kinematic)** that meets human approval.
- **Randomization**: A single generated script can produce at least 5 distinct, valid geometric variations (verified by differing mesh hashes/volumes).

## 6. Assumptions

- **Default Workspace**: Unless otherwise specified, the system assumes a standard "Domain Box" of **100x100x100mm** for all benchmark scenarios.
- **Documentation Necessity**: Previous experience shows that the LLM lacks sufficient knowledge of specific `build123d` syntax, the syntax as a whole. The system MUST provide the agent with a `search_docs` tool to verify syntax during plan and code generation.
- We can run MuJoCo headless in the dev environment for the stability checks.
- The "Human-in-the-loop" UI (Feature 007) will consume the file structure defined here, but this feature (005) does not build the UI itself (only the CLI and artifacts).