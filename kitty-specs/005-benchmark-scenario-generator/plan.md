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
- `LangChain` / `LangGraph`: Underlying agentic infrastructure.
**Infrastructure**:
- **Temporal**: Orchestrates the distributed worker nodes for long-running generation/validation jobs.
- **Langfuse**: Provides full observability into LLM traces, prompts, and tool calls.
- **Worker Node**: Runs the generation and validation scripts in a `SandboxFilesystemBackend`.
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
├── tasks.md             # Tasks
└── tasks/               # Individual Task Prompt Files
```

### Source Code

```text
src/
├── generators/
│   ├── benchmark/       # Generator Agent Logic (Controller)
│   │   ├── graph.py     # Planner -> Coder -> Reviewer Graph
│   │   ├── state.py     # LangGraph State
│   │   ├── models.py    # Pydantic Models
│   │   └── templates/   # Script Templates
├── worker/
│   ├── utils/
│   │   ├── validation.py # simulate() / validate() implementation
│   │   └── filesystem.py # Standard agent filesystem setup
│   └── agent_files/      # Template for agent filesystem
│       ├── journal.md
│       ├── todo.md
│       └── script.py
```

## Complexity Tracking

| Violation | Why Needed | Simpler Alternative Rejected Because |
|-----------|------------|-------------------------------------|
| Agentic Generation | Diversity. | Hand-coding 1000s of benchmarks is impossible. Static templates lack variation. |
| Stability Check | Quality Assurance. | Proceeding with unstable physics simulations wastes Engineer Agent compute time and confuses the learning process. |
