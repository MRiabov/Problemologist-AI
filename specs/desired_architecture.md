# Desired architecture

## Scope summary

- This file is a navigation index, not a long-form architecture spec.
- It tells agents where each architecture concern is documented in `specs/architecture/**`.
- When a task references `@specs/desired_architecture.md`, route to the relevant detailed file(s) below.

This file is the architecture entrypoint. The architecture has been split into focused documents under `specs/architecture/`.

When a task references `@specs/desired_architecture.md`, treat the files below as the source of truth and read the sections relevant to the task.

## Architecture index

### System goals

- [Primary system objectives](./architecture/primary-system-objectives.md): end goals, expected outputs, and product-level purpose of the benchmark and engineer system.

### Agents

- [Agents overview](./architecture/agents/overview.md): high-level map of benchmark generator and engineer graphs.
- [Agent roles](./architecture/agents/roles.md): role responsibilities, required artifacts, and planner/implementer/reviewer behavior.
- [Agent handovers and contracts](./architecture/agents/handover-contracts.md): file-level handoff contracts and refusal/review routing.
- [Agentic framework, artifacts, and filesystem](./architecture/agents/artifacts-and-filesystem.md): DSPy/LangGraph runtime, trace requirements, and path-permission policy.
- [Agent tools](./architecture/agents/tools.md): ReAct-callable tool surface, Python utility functions, and reviewer/planner submission gates.

### Runtime and infrastructure

- [Distributed execution](./architecture/distributed-execution.md): controller/worker split, worker APIs, persistence, and Temporal boundary.
- [CAD and other infrastructure](./architecture/CAD-and-other-infra.md): CAD metadata, rendering direction, schema contracts, and supporting infra assumptions.

### Evaluation and quality gates

- [Agent evaluations and gates](./architecture/evals-and-gates.md): quality tiers, pass criteria, terminalization, and fail-closed gates.

### Simulation

- [Simulation and Definitions of Done](./architecture/simulation-and-dod.md): physics assumptions, constraints model, and success/failure definitions.

### Observability

- [Observability](./architecture/observability.md): event schema, IDs/lineage, metrics, and logging/backups/user-review data.

## Document ownership rule

- Add or update architecture requirements in the corresponding `specs/architecture/**` file.
- Keep this file as an index and stable entrypoint, not as a long-form monolith.
