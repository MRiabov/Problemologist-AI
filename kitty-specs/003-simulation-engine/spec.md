# Feature Specification: MuJoCo Simulation Engine

**Feature**: 003-simulation-engine
**Status**: Draft
**Mission**: software-dev

## 1. Overview

The **MuJoCo Simulation Engine** is the physics validation backend for the **Agentic CAD Environment**. It operates within the **Worker Node** and is invoked via the `simulate` utility function available to agents.

Its primary responsibility is to take a CAD assembly, simulate it in a physics environment ensuring correct "Definition of Done" (DoD), and produce high-fidelity artifacts (videos, logs). Large media is stored in **S3** via the worker's **CompositeBackend** routing.

## 2. Goals & Success Criteria

### 2.1. Primary Goals

1. **Physics Verification**: Verify if a CAD design achieves the task objective (e.g., "move object A to zone B").
2. **High-Fidelity Artifacts**: Render and compress simulation videos (`.mp4`) for human review and agent feedback.
3. **Long-Running Execution**: Integrate with **Temporal** to handle simulations that exceed immediate HTTP timeout thresholds.
4. **Automated Pipeline**: `CAD -> MJCF -> Simulation -> Video -> S3`.

### 2.2. Success Criteria

- **Accuracy**: Convex decomposition accurately approximates collision geometry.
- **Durability**: Simulations surviving container restarts via Temporal checkpoints (optional, nice to have).
- **Observability**: 100% of simulations produce a video URL and a structured Markdown summary.

## 3. User Stories

- **As an Agent**, I want to call `simulate(my_model)` and get back a Pass/Fail result with a reason.
- **As a User**, I want to see a video of the simulation to understand what the agent built.
- **As an Agent**, I want my simulation to include "Sensors" (Goal Zones, Forbidden Zones) that automatically trigger success/failure.

## 4. Functional Requirements

### 4.1. The `simulate` Pipeline

The `simulate(component)` python function triggers the following steps on the Worker:

1. **Snapshot**: `git commit` current workspace state.
2. **Scene Builder**:
    - Parse `component` (Agent's design).
    - Parse `environment` (Benchmark constraints).
    - Identify "Zones" (Goal, Forbid, Start) by name convention (`zone_goal_*`, `zone_forbid_*`).
3. **Compilation**:
    - Generate Meshes (STL).
    - Compute Convex Hulls (`vhacd` / internal).
    - Generate MJCF XML (`scene.xml`).
4. **Simulation Loop**:
    - Step physics (500Hz).
    - Check triggers (Goal Entry, Forbid Collision).
    - Render frames (30fps).
5. **Post-Processing**:
    - Encode frames to MP4 (`ffmpeg`).
    - Save MP4 and MJCF to `/renders/` (automatically routed to S3).
    - Generate Simulation Report (Markdown).

### 4.2. Temporal Integration

For simulations predicted to run > 30s:

1. Worker returns a "Pending" status or registers a Temporal Activity.
2. Temporal orchestrates the execution, monitoring for timeouts.
3. Upon completion, Temporal updates the Task status in the Database.
*Note: Temporal is mandatory for all simulations expected to run > 30s to ensure durability against worker preemption.*

### 4.3. Zone Definitions

| Object Name | Behavior |
| :--- | :--- |
| `zone_goal_*` | Detects "Target Object" entry. Triggers SUCCESS. |
| `zone_forbid_*` | Detects ANY collision. Triggers FAILURE. |
| `zone_start_*` | Spawns objects/agents. |

## 5. Technical Design

### 5.1. Tech Stack

- **Physics**: MuJoCo.
- **Rendering**: MuJoCo Native Renderer (GLContext) + `ffmpeg`.
- **Orchestration**: Temporal (Activities).
- **Storage**: MinIO (S3 compatible) for artifacts.

### 5.2. Output Schema

The `simulate` function returns a `SimulationResult` object:

```python
class SimulationResult(BaseModel):
    outcome: Literal["SUCCESS", "FAIL"]
    reason: str
    video_url: str
    metrics: dict[str, float]  # energy, time, damage
    markdown_report: str
```
