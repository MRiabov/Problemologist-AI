# Implementation Plan: Engineer Agent

*Path: templates/plan-template.md*

**Branch**: `002-engineer-agent` | **Date**: 2026-02-05 | **Spec**: [spec.md](spec.md)
**Input**: Feature specification from `/kitty-specs/002-vlm-cad-agent/spec.md`

## Summary

Implement the **Engineer Agent** using **LangGraph** within the **`deepagents`** framework. The agent orchestrates the creation of CAD solutions by interacting with the distributed **Agentic CAD Environment** (Spec 001). It features a distinct **Architect-Engineer-Critic** loop, persistent file-based memory (Journal/TODOs), and an asynchronous **Sidecar Learner** for skill acquisition.

## Technical Context

**Language/Version**: Python 3.10+
**Frameworks**:

- `langgraph`: State machine orchestration.
- `deepagents`: Agent framework integration.
- `langchain-google-genai` / `langchain-anthropic`: LLM providers.
**Dependencies**:
- `001-agentic-cad-environment`: The execution runtime API.
**Storage**:
- **Postgres**: LangGraph checkpoints (State/Reasoning Traces).
- **Local Sandbox**: Ephemeral memory files (Journal, TODO, Script).
- **Global S3**: Persistence of final renders, CAD assets, and media produced by tools.

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

[No conflicts. Aligned with Agentic Framework.]

## Project Structure

### Documentation

```text
kitty-specs/002-vlm-cad-agent/
├── plan.md              # This file
├── research.md          # Phase 0
├── data-model.md        # State Schema
├── contracts/           # Prompt Templates
└── tasks.md             # Tasks
```

### Source Code

```text
src/agent/
├── graph/
│   ├── graph.py        # Nodes & Edges
│   ├── state.py        # State Definition
│   └── nodes/
│       ├── architect.py
│       ├── engineer.py
│       ├── critic.py
│       └── sidecar.py  # Skill Learner
├── run.py              # Entrypoint
└── prompts/            # Jinja2 Templates
```

## Complexity Tracking

| Violation | Why Needed | Simpler Alternative Rejected Because |
|-----------|------------|-------------------------------------|
| Graph Architecture | Complex flows (Planning -> Execution -> Critique). | Linear chains cannot handle the "Refusal" and "Retry" loops required for robust engineering. |
| Sidecar Learner | Skills need to evolve without polluting the active context. | In-context learning forgets; Synchronous updates distract the main agent. |
| File-Based Memory | Agent-readable persistence (Journal). | Vector stores are opaque; Agents understand Markdown files best. |
