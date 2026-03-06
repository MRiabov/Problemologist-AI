# Desired architecture

This file is the architecture entrypoint. The architecture has been split into focused documents under `specs/architecture/`.

When a task references `@specs/desired_architecture.md`, treat the files below as the source of truth and read the sections relevant to the task.

## Architecture index

### Agents

- [Agents overview](./architecture/agents/overview.md)
- [Agent roles](./architecture/agents/roles.md)
- [Agent handovers and contracts](./architecture/agents/handover-contracts.md)
- [Agentic framework, artifacts, and filesystem](./architecture/agents/artifacts-and-filesystem.md)

### Runtime and infrastructure

- [Distributed execution](./architecture/distributed-execution.md)

### Evaluation and quality gates

- [Agent evaluations and gates](./architecture/evals-and-gates.md)

### Simulation

- [Simulation and Definitions of Done](./architecture/simulation-and-dod.md)

### Observability

- [Observability](./architecture/observability.md)

## Document ownership rule

- Add or update architecture requirements in the corresponding `specs/architecture/**` file.
- Keep this file as an index and stable entrypoint, not as a long-form monolith.
