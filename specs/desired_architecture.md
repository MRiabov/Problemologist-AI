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
- [Agent harness](./architecture/agents/agent-harness.md): DSPy/LangGraph runtime, debug Codex backend, workspace/prompt/runner contract, and skill-loading policy.
- [Agent artifacts and filesystem](./architecture/agents/artifacts-and-filesystem.md): artifact surfaces, file ownership, and path-permission policy.
- [Agent tools](./architecture/agents/tools.md): ReAct-callable tool surface, Python utility functions, checked-in shell-script bridges for custom command-like operations, and reviewer/planner submission gates. Custom command-like behavior is defined by shell scripts, not newly invented ReAct endpoints.
- [Definitions of success and failure](./architecture/agents/definitions-of-success-and-failure.md): objective AABB rules, runtime randomization, benchmark-side motion exception, and failure taxonomy.

### Runtime and infrastructure

- [Distributed execution](./architecture/distributed-execution.md): controller plus split worker plane, worker APIs, dedicated renderer worker, persistence, Temporal boundary, and backend-routing rules such as fast validation preview versus physics simulation.
- [CAD and other infrastructure](./architecture/CAD-and-other-infra.md): CAD metadata, rendering direction, dedicated renderer worker boundary, validation-preview rendering policy, schema contracts, and supporting infra assumptions.

### Evaluation and quality gates

- [Application acceptance criteria](./architecture/application-acceptance-criteria.md): legacy application acceptance surface; the current product acceptance criteria live in `_bmad-output/planning-artifacts/prd.md` and `_bmad-output/planning-artifacts/epics.md`.
- [Eval architecture](./architecture/evals-architecture.md): how eval tiers, pass criteria, terminalization, and fail-closed gates work.
- [Agent reward architecture](./architecture/agent/reward-architecture.md): reward shaping used for downstream training and optimization.

### Simulation

- [Simulation and rendering](./architecture/simulation-and-rendering.md): physics assumptions, backend responsibility split, dedicated renderer worker, constraints model, and rendering/preview ownership.
- [Fluids, FEM, and stress validation](./architecture/fluids-and-deformables.md): Genesis-backed fluid simulation, deformable-material contracts, stress objectives, smoke-test policy, and WP2-specific artifacts.
- [Electronics and electromechanical systems](./architecture/electronics-and-electromechanics.md): electrical schema, circuit-validation gate, 3D wire routing, power-gated actuation, and WP3-specific artifacts.

### Observability

- [Observability](./architecture/observability.md): event schema, IDs/lineage, metrics, and logging/backups/user-review data.

## Document ownership rule

- Add or update architecture requirements in the corresponding `specs/architecture/**` file.
- Keep this file as an index and stable entrypoint, not as a long-form monolith.
