# Implementation Plan: [FEATURE]

*Path: [templates/plan-template.md](templates/plan-template.md)*

**Branch**: `002-vlm-cad-agent` | **Date**: 2026-02-01 | **Spec**: [spec.md](spec.md)
**Input**: Feature specification from `/kitty-specs/002-vlm-cad-agent/spec.md`

**Note**: This template is filled in by the `/spec-kitty.plan` command. See `src/specify_cli/missions/software-dev/command-templates/plan.md` for the execution workflow.

The planner will not begin until all planning questions have been answered—capture those answers in this document before progressing to later phases.

## Summary

Implement the **VLM CAD Agent**, a cognitive engine that autonomously solves geometric problems by interacting with the `001-agentic-cad-environment`. The agent uses a "Think, Plan, Act" loop to generate `build123d` code, validating its output via visual feedback from the environment.

## Technical Context

**Language/Version**: Python 3.10+
**Primary Dependencies**:

- `litellm`: Unified interface for LLM/VLM providers (OpenAI, Gemini, Anthropic)
- `pydantic`: Structured data validation for tool calls and planning
- `jinja2`: Template engine for prompt construction
- `rich`: Terminal UI for real-time thought process visualization
- `gymnasium`: Interface to the loosely-coupled environment
**Storage**:
- `journal.md`: Markdown-based long-term memory (read/append)
- `agent_trace.jsonl`: Structured execution logs
**Testing**: `pytest` with `pytest-mock` for deterministic replay of agent reasoning.
**Target Platform**: Linux (Development), Docker (Deployment).
**Performance Goals**:
- **Reasoning Overhead**: < 2s per step (excluding LLM latency).
- **Context Management**: Efficiently handle 128k+ context windows by summarizing past steps.
**Constraints**:
- **Loose Coupling**: Agent treats the Environment as a black box (Gymnasium interface).
- **Synchronous**: Simplified specific "Think -> Act -> Observe" loop.

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

[Constitution file missing. Proceeding with standard best practices for Agentic development.]

## Project Structure

### Documentation (this feature)

```
kitty-specs/002-vlm-cad-agent/
├── plan.md              # This file
├── research.md          # Phase 0 output
├── data-model.md        # Phase 1 output
├── contracts/           # Phase 1 output (Prompt Schemas)
└── tasks.md             # Phase 2 output
```

### Source Code (repository root)

```
src/
├── agent/
│   ├── core/
│   │   ├── engine.py       # Main ReAct loop & State Machine
│   │   ├── context.py      # Window management & summarization
│   │   └── memory.py       # Journaling (Long-term memory)
│   ├── clients/
│   │   ├── llm.py          # LiteLLM wrapper with retry logic
│   │   └── tools.py        # Schema generators for Env tools
│   ├── prompts/
│   │   ├── system.j2       # Persona definitions
│   │   └── tasks.j2        # Dynamic task prompts
│   └── runner.py           # CLI entry point to spin up Agent + Env
```

**Structure Decision**: Clean separation of `src/agent` from `src/environment` (Spec 001). The agent packages its own logic and consumes the environment via standard interfaces.

## Complexity Tracking

*Fill ONLY if Constitution Check has violations that must be justified*

| Violation | Why Needed | Simpler Alternative Rejected Because |
|-----------|------------|-------------------------------------|
| Custom ReAct Loop | Need explicit control over "Think", "Plan", "Act" phases and specialized logging | Standard `LangChain` agents are often opaque and harder to debug for specific VLM workflows |
| File-based Memory | Simple, human-readable long-term memory | Vector DB is overkill for the initial scope and adds infrastructure complexity |
