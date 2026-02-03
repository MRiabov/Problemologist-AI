# Implementation Plan: [FEATURE]

*Path: [templates/plan-template.md](templates/plan-template.md)*

**Branch**: `002-vlm-cad-agent` | **Date**: 2026-02-03 | **Spec**: [spec.md](spec.md)
**Input**: Feature specification from `/kitty-specs/002-vlm-cad-agent/spec.md`

**Note**: This plan has been updated to reflect the **Tool Consolidation** (WP05) and the implementation of the **Skill Populator** node.

## Summary

Implement the **VLM CAD Agent** using the **DeepAgents** framework (powered by **LangGraph**). This cognitive engine autonomously solves geometric problems by interacting with the `001-agentic-cad-environment`. The agent uses a graph-based "Think, Plan, Act" workflow, leveraging file-system memory (Journal and Skills) for long-horizon tasks.

## Technical Context

**Language/Version**: Python 3.10+
**Primary Dependencies**:

- `langgraph`: Core orchestration engine (Graph, State, Nodes).
- `deepagents`: High-level agent capabilities (planning, file-system memory).
- `langchain-core` / `langchain-openai`: LLM Interface & Tool Abstractions.
- `pydantic`: State and Tool validation.
- `rich`: Terminal UI.

**Storage**:

- `journal.md`: Markdown-based long-term memory (read/append via `write_file`).
- `.agent/skills/`: Persistent skill-based knowledge system.
- `checkpoints.sqlite`: LangGraph state persistence (short-term session memory).

**Key Refined Tools**:

- `write_file`: Consolidated tool for writing scripts and journaling (with auto-timestamping).
- `edit_file`: Consolidated tool for targeted string replacement.
- `preview_design`: Core visual feedback loop for VLM reasoning.
- `skill_tools`: Suite of tools for persistent knowledge management (`read_skill`, `update_skill`, etc.).

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
| LangGraph + DeepAgents | Replaces custom `while` loop | Provides robust state persistence, time-travel debugging, and formalized "Planning" steps. |
| File-based Memory | Simple, human-readable long-term memory | Vector DB is overkill; file-based approach enables direct agent manipulation via `write_file`. |
| Skill Populator | Automated learning loop | Manual documentation updates are prone to being skipped. |
