# Architecture Review - February 14, 2026

## Overview

This review compares the current implementation against the desired architecture specified in `kitty-specs/desired_architecture.md`.

## Critical Gaps & Inconsistencies

### 1. Missing `events.jsonl` (Spec lines 104-110)

- **Spec**: "Each run MUST append to `events.jsonl` in the root of the episode... This is our training data."
- **Status**: **Missing**. The system uses structured logging (`structlog`), DB persistence (`Episode`, `Asset` tables), and Langfuse for tracing. While the data is captured, it is not being exported to a raw `events.jsonl` file per episode as required for the training dataset pipeline.

### 2. Batch Support (Spec lines 95-100)

- **Spec**: "Batch support is first-class. The system must handle hundreds of problem generations in parallel."
- **Status**: **Underspecified/Partial**. Temporal is correctly used for distributed execution (`SimulationWorkflow`), allowing for horizontal scaling. however, the **API layer** (`controller/api/routes/episodes.py`) lacks a dedicated `/batch` endpoint for submitting multiple tasks at once. Currently, tasks must be submitted individually.

### 3. Agent Node Discrepancy (Spec lines 1700-1800)

- **Spec**: Describes a 5-node chain: Planner -> Coder -> Validator -> Reviewer -> Optimizer.
- **Status**: **Inconsistent**.
  - The `BenchmarkGeneratorState` graph (`controller/agent/benchmark/graph.py`) uses: `planner` -> `coder` -> `reviewer` -> `skills`.
  - `coder` likely performs validation internally, but there is no separate `Validator` node to provide an independent check or specialized validation logic as implied by the spec.
  - The `Optimizer` role is partially filled by the `SkillsNode` (Sidecar Learner), but the specific "CAD Optimization" loop (taking journal patterns to refactor `build123d` code) is not explicitly implemented as a standalone stage.

### 4. Optimizer Agent (Spec lines 1780-1800)

- **Spec**: "Optimizer agent... takes the journal... tries to find patterns... suggesting code fixes."
- **Status**: **Missing/Partial**. While `SkillsNode` (`controller/agent/nodes/skills.py`) suggests new skills and syncs them to Git, it doesn't appear to perform the proactive CAD optimization loop to improve the current design's performance metrics (weight/cost) as a second-pass operation.

### 5. Frontend Visualization Gaps

- **Spec**: "Rich interactive 2D graphs... 24-view renders."
- **Status**:
  - **24-view Renders**: **Implemented** in `worker/simulation/renderer.py`.
  - **2D Graphs**: **Basic**. `CircuitTimeline.tsx` exists, but the "rich interactive" metric dashboards described in the spec are not fully realized in the current component library.

## Missing Functionality (Omissions)

- **CAD Refusal Handling** (Spec lines 1760-1770): The `ReviewFrontmatter` schema includes `confirm_plan_refusal`, but the logic for the agent to explicitly "refuse" to implement a plan if it's physically impossible needs more robust implementation in the `coder` node.
- **Asset Catalog Search** (Spec lines 1850+): The UI has `ModelBrowser.tsx`, but it lacks the advanced filtering capabilities (searching by specific telemetry like "voltage drop < 0.5V") mentioned as an end-goal.

## Technical Debt / Risks

- **MuJoCo Stress Field**: `MuJoCoBackend.get_stress_field` is currently a placeholder (returns `None`). This limits the "Part Breakage" detection to heuristics or requires the Genesis backend to be fully operational for FEM.
- **Electronics Gating**: The gating of actuators by electronics in `SimulationLoop.step` is implemented via a simple `is_powered_map`. This is effective but might miss complex transient behaviors.

## Recommendations

1. **Implement `events.jsonl` export**: Add a Temporal activity to collect all episode logs and traces into a single JSONL file at the end of a run.
2. **Add `/batch` API endpoint**: Allow users to submit a list of prompts.
3. **Formalize the Validator Node**: Split validation logic out of the `coder` node to allow for independent "Adversarial Validation".
4. **Expand Optimizer Logic**: Enhance the `SkillsNode` to not just suggest skills but to also propose CAD refactors to the `planner`.
