# Implementation Plan: Benchmark Scenario Generator

*Path: kitty-specs/005-benchmark-scenario-generator/plan.md*

**Branch**: `005-benchmark-scenario-generator` | **Date**: 2026-02-07 | **Spec**: [spec.md](spec.md)

## Summary

Implement the **Benchmark Scenario Generator** using the `deepagents` framework over LangChain/LangGraph. This system will autonomously generate, verify, and review mechanical engineering benchmarks. It features a distributed architecture with a central Controller and specialized Worker nodes for safe execution and simulation.

## Technical Context

- **Framework**: `deepagents` (with `FilesystemMiddleware` and `TodoListMiddleware`).
- **Orchestration**: LangGraph for agent flow, Temporal for durable execution of worker tasks.
- **CAD/Physics**: `build123d` for geometry, MuJoCo for dynamics simulation.
- **Observability**: Langfuse for trace logging, Postgres for state/metadata.
- **Storage**: S3-compatible buckets for videos and renders.

## Project Structure

### Source Code

```text
src/
├── agent/
│   ├── benchmark/           # Generator Agent (Controller)
│   │   ├── graph.py         # LangGraph definition
│   │   ├── state.py         # Agent state schema
│   │   ├── planner.py       # Planner node logic
│   │   ├── coder.py         # Coder node logic
│   │   └── reviewer.py      # Reviewer node logic
│   └── learner/             # Sidecar Skill Learner agent
├── worker/
│   ├── sandbox/             # Sandbox configuration & templates
│   ├── utils/               # Python utils (simulate, validate)
│   └── tools/               # Agent-native tools (read, write, execute)
└── shared/
    ├── models/              # Pydantic & SQLAlchemy models
    └── schema/              # API and Task schemas
```

## Complexity Tracking

| Violation | Why Needed | Simpler Alternative Rejected Because |
|-----------|------------|-------------------------------------|
| `deepagents` Framework | Required for advanced filesystem/TODO/Subagent abstractions. | Manual implementation of middlewares is error-prone and non-standard. |
| Temporal Integration | Ensures durable execution of long-running simulations. | Standard HTTP retries don't handle container preemption or 10-minute tasks well. |
| Python Tool Imports | Agents perform better calling scripts through code than via JSON tool calls (ReAct). | Traditional tool calls often confuse agents regarding context and object persistence. |
| Async Skill Learner | Continuous improvement of build123d proficiency without polluting main agent context. | Main agents are often too "in the moment" to perform high-level architectural reflection. |

## Implementation Phases

### Phase 1: Infrastructure & Data (WP01)
- Set up Postgres/Temporal/Langfuse.
- Define SQLAlchemy/Pydantic models for Sessions, Benchmarks, and Assets.
- Implement `SandboxFilesystemBackend` for workers.

### Phase 2: Worker Utilities (WP02)
- Implement `simulate()` with MuJoCo stability checks and video rendering.
- Implement `validate()` for CAD constraints and randomization bounds.
- Set up S3 routing for the `/renders/` directory.

### Phase 3: Agent Graph (WP03 & WP04)
- Implement Planner (TODO list creation).
- Implement Coder (build123d script generation).
- Implement Reviewer (Visual + text summary analysis).
- Configure `deepagents` middlewares.

### Phase 4: Skills & Observability (WP05)
- Implement Journal compression and Learner agent node.
- Integrate Langfuse tracing.
- Create CLI for batch benchmark generation.