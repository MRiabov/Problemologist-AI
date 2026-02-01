# Specification: Benchmark Scenario Generator

**Feature**: 005-benchmark-scenario-generator
**Status**: Draft
**Mission**: software-dev

## 1. Introduction

The **Benchmark Scenario Generator** is an automated pipeline designed to rapidly populate the `Problemologist` benchmark suite. Instead of manually authoring 40+ unique physics puzzles (a process that previously took days), this system leverages an LLM to procedurally generate parametric Python scripts. These scripts define the "Levels" the agent must solve—ranging from simple spatial tasks (Peg-in-Hole) to complex kinematic assemblies (Cranks/Levers).

The system ensures that every generated scenario is robust, randomized, and physically valid before it reaches the human-in-the-loop for final approval.

### 1.1 Goals

*   **Acceleration**: Reduce the time to create a valid benchmark scenario from hours to minutes.
*   **Diversity**: Generate high variance in problem geometry, goal positions, and obstacle placement ("forbid zones") to prevent overfitting.
*   **Validity**: Ensure all generated scenarios are geometrically consistent (watertight meshes) and simulation-ready (loadable MJCF).
*   **Automation**: Provide a "Generate → Compile → Validate" loop that only requires human intervention at the final review stage.

### 1.2 Out of Scope

*   **Runtime Generation**: This is a *design-time* tool to build the dataset, not a feature for the end-user agent to generate its own problems at runtime (yet).
*   **Tier 3 Scenarios**: Complex integrated mechanisms (Motor Mounts) are out of scope for the MVP; focus is on Spatial and Kinematic tiers.
*   **Matplotlib Hallucination**: We will not use 2D plot hallucination for validation; we rely on direct 3D generation and rendering.

## 2. User Scenarios

### 2.1 The "Dataset Architect" Flow

**Actor**: Main Developer / Dataset Curator
**Goal**: Populate the "Kinematic" benchmark tier with 10 new lever-based puzzles.

1.  **Prompting**: The user invokes the generator with a high-level intent: "Generate 10 variations of a Class 1 Lever problem where the fulcrum position varies."
2.  **Generation**: The system's LLM drafts 10 distinct Python scripts. Each script utilizes `build123d` for geometry and `mujoco` logic for physics constraints.
3.  **Compilation & Validation**: The system automatically runs each script:
    *   Generates STL meshes for the base, lever, and load.
    *   Generates the MJCF (XML) file.
    *   Checks for mesh manifoldness and XML syntax errors.
    *   Simulates 100 steps to ensure the scene doesn't explode immediately (stability check).
4.  **Review**: The user opens the review interface (CLI or future Dashboard), sees 8 successes and 2 failures.
5.  **Approval**: The user visually inspects the 8 successful renders. They approve 6 that look challenging but solvable.
6.  **Commit**: The approved scenarios are saved to `datasets/benchmarks/tier2_kinematic/` with their randomization parameters locked.

## 3. Functional Requirements

### 3.1 Scenario Script Generation

*   **FR-01**: The system **MUST** use an LLM (e.g., Claude 3.5 Sonnet or GPT-4o) to write Python scripts that define a scenario.
*   **FR-02**: Generated scripts **MUST** utilize `build123d` for all constructive solid geometry (CSG) operations.
*   **FR-03**: Generated scripts **MUST** define a parameterized `build()` function that accepts a randomization seed or config dict.
*   **FR-04**: Scripts **MUST** include logic to generate "Forbid Zones" (red obstacles) and "Goal Zones" (green targets) within the workspace.

### 3.2 Randomization Engine

*   **FR-05**: The system **MUST** be able to vary the "Domain Box" (workspace size) dimensions (Length/Width/Height).
*   **FR-06**: The system **MUST** support randomization of start/goal/obstacle positions by at least ±40% of their bounding volumes, as specified by the user.
*   **FR-07**: Randomization logic **MUST** ensure objects remain within the global workspace bounds (no spawning outside the box).

### 3.3 Artifact Compilation

*   **FR-08**: The system **MUST** execute the generated Python scripts to produce simulation assets.
*   **FR-09**: It **MUST** export all solid parts as independent STL or OBJ files.
*   **FR-10**: It **MUST** generate a valid MJCF (XML) file that references these meshes and defines the necessary joints/actuators.

### 3.4 Automated Validation

*   **FR-11**: **Geometric Check**: The system **MUST** verify that generated meshes are watertight (manifold) and have non-zero volume.
*   **FR-12**: **Physics Check**: The system **MUST** attempt to load the generated MJCF in MuJoCo and run for 1 second (sim time).
*   **FR-13**: **Stability Check**: If the maximum velocity of any body exceeds a threshold (e.g., 100 m/s) during the stability check, the generation is marked as "Failed/Exploded".

### 3.5 Review & Export

*   **FR-14**: The system **MUST** save successful runs to a staging directory.
*   **FR-15**: It **MUST** generate a static preview image (snapshot) of the scene for the reviewer.
*   **FR-16**: It **MUST** provide a CLI command to "promote" a staged scenario to the permanent benchmark suite.

## 4. Technical Constraints

*   **Language**: Python 3.10+
*   **CAD Kernel**: `build123d` (essential for programmatic generation).
*   **Physics Engine**: `mujoco` (standard for the project).
*   **LLM Interface**: `langchain` or direct API calls (consistent with Agent implementation).
*   **File Structure**:
    *   `src/generators/`: The generation logic.
    *   `scenarios/staging/`: Temporary hold for generated items.
    *   `scenarios/benchmarks/`: Final home for approved levels.

## 5. Success Criteria

*   **Efficiency**: Can generate and validate 10 candidate scenarios in under 5 minutes.
*   **Yield Rate**: At least 50% of generated scripts compile and pass the stability check (Simulation Load + 1s run).
*   **Coverage**: Successfully generates at least one scenario for **Tier 1 (Spatial)** and one for **Tier 2 (Kinematic)** that meets human approval.
*   **Randomization**: A single generated script can produce at least 5 distinct, valid geometric variations (verified by differing mesh hashes/volumes).

## 6. Assumptions

*   The LLM has sufficient knowledge of `build123d` syntax (or we provide few-shot examples in the prompt).
*   We can run MuJoCo headless in the dev environment for the stability checks.
*   The "Human-in-the-loop" UI (Feature 007) will consume the file structure defined here, but this feature (005) does not build the UI itself (only the CLI and artifacts).
