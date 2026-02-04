# Implementation Plan: Engineer Agent

**Branch**: `002-engineer-agent` | **Date**: 2026-02-04 | **Spec**: [spec.md](spec.md)

## Summary

Implement the **Engineer Agent** using the **LangGraph** framework. This cognitive engine autonomously solves geometric problems by interacting with the `001-agentic-cad-environment` via `ToolRuntime` inside a Podman sandbox. The agent uses a graph-based "Architect → Engineer → Critic" workflow.

## Technical Context

**Language/Version**: Python 3.10+
**Primary Dependencies**:
- `langgraph`: Core orchestration engine.
- `fastapi`: Host-Container communication (OpenAPI).
- `podman`: Sandbox enforcement.

**Key Roles**:
- **Architect (Planner)**: Designs the high-level approach and persists a **TODO List**.
- **Engineer (Actor)**: Implements the CAD solution. Can refuse the plan if proven impossible.
- **Critic**: Reviews the implementation against constraints.

**Storage**:
- `todo.md`: Task-specific TODO list persisted by the Architect.
- `.agent/skills/`: Persistent skill-based knowledge system.

**Testing**: `pytest` and `langsmith` for trace evaluation.
**Target Platform**: Linux (Development), Docker (Deployment).

## Project Structure

### Documentation (this feature)

```text
kitty-specs/002-vlm-cad-agent/
├── plan.md              # This file
├── research.md          # Phase 0 output
├── data-model.md        # Phase 1 output
├── contracts/           # Phase 1 output (Prompt Schemas)
└── tasks.md             # Phase 2 output
```

### Source Code (repository root)

```text
src/
├── agent/
│   ├── graph/
│   │   ├── graph.py        # Graph Builder (Nodes + Edges)
│   │   ├── state.py        # Agent State Definition
│   │   └── nodes/          # Step implementations
│   │       ├── planner.py
│   │       ├── actor.py
│   │       ├── critic.py
│   │       └── skill_populator.py
│   ├── tools/
│   │   ├── env.py          # LangChain Tool Wrappers
│   │   ├── env_adapter.py  # Async Sandbox Adapters
│   │   └── memory.py       # Journal reading tools
│   └── runner.py           # Entry point
└── environment/
    └── tools.py            # Core environmental implementations
```

**Structure Decision**: Clean separation of `src/agent` (cognitive) from `src/environment` (execution). Tools are defined in `src/environment` and wrapped in `src/agent/tools/env.py` for LangChain compatibility.

## Complexity Tracking

| Violation | Why Needed | Simpler Alternative Rejected Because |
| --- | --- | --- |
| LangGraph | Replaces custom `while` loop | Provides robust state persistence, time-travel debugging, and formalized "Planning" steps. |
| File-based Memory | Simple, human-readable long-term memory | Vector DB is overkill; file-based approach enables direct agent manipulation via `write_file`. |
| Skill Populator | Automated learning loop | Manual documentation updates are prone to being skipped. |
