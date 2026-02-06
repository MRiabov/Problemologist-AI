# Implementation Plan: Benchmark Scenario Generator

*Path: kitty-specs/005-benchmark-scenario-generator/plan.md*

**Branch**: `005-benchmark-scenario-generator` | **Date**: 2026-02-05 | **Spec**: [spec.md](spec.md)
**Input**: Feature specification from `/kitty-specs/005-benchmark-scenario-generator/spec.md`

## Summary

Implement the **Benchmark Scenario Generator** as a `deepagents` based tool. It consists of a "Generator Agent" that writes randomized `build123d` scripts, validates them using the Worker's `simulate` validated-execution pipeline, and packages them into the `dataset/` structure.

## Technical Context

**Language/Version**: Python 3.10+
**Frameworks**:

- `deepagents`: Agent orchestration.
- `build123d`: CAD.
- `mujoco`: Stability checks.
**Infrastructure**:
- **Worker Node**: Runs the generation and validation scripts in a local sandbox.
- **S3**: Stores the generated dataset media via the `/renders/` routed path.

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

[No conflicts. Aligned with Dataset creation goals.]

## Project Structure

### Documentation

```
kitty-specs/005-benchmark-scenario-generator/
├── plan.md              # This file
├── research.md          # Research
├── data-model.md        # Dataset Schema
└── tasks.md             # Tasks
```

### Source Code

```text
src/
├── generators/
│   ├── benchmark/       # Generator Agent Logic
│   │   ├── graph.py     # Planner -> Coder -> Reviewer
│   │   └── templates/   # Script Templates
├── worker/
│   └── utils/
│       └── validation.py # Stability Check Logic
```

## Complexity Tracking

| Violation | Why Needed | Simpler Alternative Rejected Because |
|-----------|------------|-------------------------------------|
| Agentic Generation | Diversity. | Hand-coding 1000s of benchmarks is impossible. Static templates lack variation. |
| Stability Check | Quality Assurance. | Proceeding with unstable physics simulations wastes Engineer Agent compute time and confuses the learning process. |
