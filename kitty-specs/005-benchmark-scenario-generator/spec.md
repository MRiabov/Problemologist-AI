# Specification: Benchmark Scenario Generator

**Feature**: 005-benchmark-scenario-generator
**Status**: Draft
**Mission**: software-dev

## 1. Introduction

The **Benchmark Scenario Generator** is an autonomous agentic pipeline that creates the training and evaluation dataset for the **Engineer Agent**. It operates as a specialized design-time agent that generates randomized, physics-verified CAD scenarios (Benchmarks).

## 2. Goals & Success Criteria

### 2.1. Primary Goals

1. **Generate randomized benchmarks**: Create parameterizable Python scripts that produce `build123d` geometry and MuJoCo XML (MJCF).
2. **Verify Validity**: Ensure every generated benchmark is geometrically valid (manifold) and physically stable (doesn't explode in simulation).
3. **Output Dataset**: Produce a structured dataset of scenarios, each with metadata (`target_quantity`, `max_unit_cost`) and assets.

### 2.2. Success Criteria

- **Validity**: 100% of "Promoted" benchmarks pass the `simulate(stability_check=True)` runtime check.
- **Distribution**: Randomization covers at least +/- 40% of workspace dimensions.
- **Review**: Includes a "Reviewer" agent loop that visually inspects (via rendered images) the scenarios before acceptance.

## 3. Functional Requirements

### 3.1. Generator Pipeline

The pipeline is an Agent Graph (Planner -> Coder -> Reviewer) executed via `deepagents`.

1. **Planner**: Interprets the user's "Theme" (e.g., "Lever problems") and defines the randomization strategy.
2. **Coder**: Writes a Python script in the **local worker sandbox**.
    - Script must define a `build(seed, scale)` function.
    - Script imports `utils.simulate`.
3. **Validation**:
    - The script calls `simulate()` to verify stability.
    - If unstable, the Coder iterates.
4. **Reviewer**:
    - Inspects the 24-angle render produced by the simulation.
    - Approves or Requests Changes.

### 3.2. Output Assets

A "Benchmark" consists of:

- `script.py`: The generator script (stored in sandbox during generation).
- `manifest.json`: Metadata (Tier, Description, Cost Targets).
- `assets/`: Pre-rendered images/videos (saved to `/renders/` which routes to S3) and reference MJCFs.

### 3.3. Randomization

- **Workspace**: Domain box size varies (rescaling).
- **Positions**: Start/Goal/Obstacles vary by +/- 40%.
- **Conflict Resolution**: Generator script must handle self-collisions during randomization (retry logic).
- **Rescaling**: Global scaling (0.5x - 2.0x) supported.

## 4. Technical Design

### 4.1. Tech Stack

- **Framework**: `deepagents` (Generator Agent).
- **Runtime**: Worker Node (Podman).
- **Libraries**: `build123d` (CAD), `mujoco` (Physics/Validation).

### 4.2. Verification Logic

Verification uses the same `simulate` util as the Engineer Agent, but with different acceptance criteria:

- **Engineer**: Did I reach the goal?
- **Generator**: Is the simulation stable (No NaN, No Explosion)? Are the start/goal zones reachable (optionally checked via reference solver)?

## 5. Assumptions

- **Design-Time**: This runs offline to build the dataset, not at runtime during user queries.
- **Headless**: All validation runs headless in the Worker container.
