# Significant Abstractions in Problemologist-AI (Feb 2026)

This document outlines the core technical abstractions that implement the declarative specification in `specs/desired_architecture.md`.

## 1. LangGraph State Machine

The top-level orchestration is handled by **LangGraph**, where a cyclic graph defines the flow between different agent roles.

- **State Model**: `AgentState` ([controller/agent/state.py](file:///home/maksym/Work/proj/Problemologist/Problemologist-AI/controller/agent/state.py))
  - A Pydantic model that persists the entire state of a "session", including conversation history, current task, generated plan, and journal entries.
- **Graph Definition**: `graph.py` ([controller/agent/graph.py](file:///home/maksym/Work/proj/Problemologist/Problemologist-AI/controller/agent/graph.py))
  - Defines nodes (`planner`, `coder`, `reviewer`, etc.) and conditional edges (`should_continue`) based on `AgentStatus`.

## 2. The Node Pattern

Each agent role is implemented as a encapsulated `Node` that uses a **ReAct** (Reason+Act) pattern.

- **Implementation**: [controller/agent/nodes/](file:///home/maksym/Work/proj/Problemologist/Problemologist-AI/controller/agent/nodes/)
- **Core Node Traits**:
  - Initializes a `ChatOpenAI` instance with a specific system prompt from `PromptManager`.
  - Attaches a set of `get_common_tools` (filesystem, git, COTS search).
  - Implements a "Validation Gate" (see below) before returning state.

## 3. Strict Typed Handover (Pydantic)

To avoid the brittleness of freeform dicts, all critical configuration and handover files are strictly typed.

- **Schemas**: `shared/models/schemas.py` ([shared/models/schemas.py](file:///home/maksym/Work/proj/Problemologist/Problemologist-AI/shared/models/schemas.py))
  - `ObjectivesYaml`: Defines the "What" (goals, zones, physics config).
  - `AssemblyDefinition`: Defines the "How" of the solution (parts count, costs, weights).
  - `ReviewFrontmatter`: Controls the graph flow (approve/reject decisions).

## 4. Remote Execution Boundary

The Controller (brain) is decoupled from the Worker (muscle) via a defined API.

- **Worker Client**: `WorkerClient` ([controller/clients/worker.py](file:///home/maksym/Work/proj/Problemologist/Problemologist-AI/controller/clients/worker.py))
- **Remote FS**: `RemoteFilesystemMiddleware` ([controller/middleware/remote_fs.py](file:///home/maksym/Work/proj/Problemologist/Problemologist-AI/controller/middleware/remote_fs.py))
  - Intercepts filesystem tool calls and proxies them to the worker container.
  - Ensures agents operate in isolated environments.

## 5. Event-Driven Observability

Every significant action is recorded as a structured event, enabling offline analysis and training.

- **Observability Schemas**: [shared/observability/schemas.py](file:///home/maksym/Work/proj/Problemologist/Problemologist-AI/shared/observability/schemas.py)
  - `ObservabilityEventType`: Enum for all event types (tool usage, simulation instability, etc.).
  - `BaseEvent`: Root for all events with timestamps and agent IDs.
- **Persistence**: Logged via `Temporal` or directly to a DB during execution.

## 6. The Validation Gatekeeper

Handover between nodes is not just a status change; it is a data integrity check.

- **Gatekeeper Logic**: `validate_node_output` ([worker/utils/file_validation.py](file:///home/maksym/Work/proj/Problemologist/Problemologist-AI/worker/utils/file_validation.py))
  - Asserts existence of `plan.md` and `todo.md`.
  - Validates markdown structure using `markdown_validator.py`.
  - Ensures Pydantic models for YAML files are satisfied.
  - Checks for "template placeholders" to prevent hallucinated defaults.

## 7. Learned Skills (SKILL.md)

A dynamic knowledge base that evolves through agent execution.

- **Skill Creator**: [controller/agent/nodes/skills.py](file:///home/maksym/Work/proj/Problemologist/Problemologist-AI/controller/agent/nodes/skills.py)
- **Format**: Follows the `SKILL.md` standard ([.agent/skills/](file:///home/maksym/Work/proj/Problemologist/Problemologist-AI/.agent/skills/)).
- **Sync**: Persisted via Git commits directly from worker nodes.
