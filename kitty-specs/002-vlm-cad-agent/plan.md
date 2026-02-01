# Implementation Plan: [FEATURE]

*Path: [templates/plan-template.md](templates/plan-template.md)*

**Branch**: `002-vlm-cad-agent` | **Date**: 2026-02-01 | **Spec**: [spec.md](spec.md)
**Input**: Feature specification from `/kitty-specs/002-vlm-cad-agent/spec.md`

**Note**: This template is filled in by the `/spec-kitty.plan` command. See `src/specify_cli/missions/software-dev/command-templates/plan.md` for the execution workflow.

The planner will not begin until all planning questions have been answered—capture those answers in this document before progressing to later phases.

## Summary

Implement the **VLM CAD Agent** using the **DeepAgents** framework (powered by **LangGraph**). This cognitive engine autonomously solves geometric problems by interacting with the `001-agentic-cad-environment`. The agent uses a graph-based "Think, Plan, Act" workflow, leveraging file-system memory for long-horizon tasks.

## Technical Context

**Language/Version**: Python 3.10+
**Primary Dependencies**:

- `langgraph`: Core orchestration engine (Graph, State, Nodes).
- `deepagents`: High-level agent capabilities (planning, file-system memory).
- `langchain-core` / `langchain-openai`: LLM Interface & Tool Abstractions.
- `pydantic`: State and Tool validation.
- `rich`: Terminal UI.
**Storage**:
- `journal.md`: Markdown-based long-term memory (read/append).
- `checkpoints.sqlite`: LangGraph state persistence (short-term session memory).
**Testing**: `pytest` and `langsmith` for trace evaluation.
**Target Platform**: Linux (Development), Docker (Deployment).
**Performance Goals**:
- **Reasoning Overhead**: Minimal graph traversal latency.
- **Context Management**: Managed via LangGraph checkpoints and DeepAgents file memory.
**Constraints**:
- **Async First**: LangGraph is natively async; runner needs `asyncio` loop.

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

[Constitution file missing. Proceeding with standard best practices for Agentic development.]

## Project Structure

### Documentation (this feature)

```text
kitty-specs/002-vlm-cad-agent/
├── plan.md              # This file
├── research.md          # Phase 0 output
├── data-model.md        # Phase 1 output
├── contracts/           # Phase 1 output (Prompt Schemas)
└── tasks.md             # Phase 2 output
```text

### Source Code (repository root)

```text
```text

src/
├── agent/
│   ├── graph/
│   │   ├── graph.py        # Graph Builder (Nodes + Edges)
│   │   ├── state.py        # Agent State Definition
│   │   └── nodes/          # Step implementations
│   │       ├── planner.py
│   │       ├── actor.py
│   │       └── critic.py
│   ├── tools/
│   │   ├── env.py          # Wrappers for Spec 001 Tools
│   │   └── memory.py       # Journal tools
│   └── runner.py           # AST/AsyncIO entry point

```text

**Structure Decision**: Clean separation of `src/agent` from `src/environment` (Spec 001). The agent packages its own logic and consumes the environment via standard interfaces.

## Complexity Tracking

*Fill ONLY if Constitution Check has violations that must be justified*

| Violation | Why Needed | Simpler Alternative Rejected Because |
|-----------|------------|-------------------------------------|
| LangGraph + DeepAgents | Replaces custom `while` loop | Provides robust state persistence, time-travel debugging (via checkpoints), and formalized "Planning" steps out of the box. |
| File-based Memory | Simple, human-readable long-term memory | Vector DB is overkill for the initial scope and adds infrastructure complexity |
