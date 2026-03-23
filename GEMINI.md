# Mandatory rules for agents

## Important context and specs

To save me refactors and to speed up debugging, please read these architecture specs on any of my requests:

- @specs/desired_architecture.md - index/entrypoint that routes to the split architecture docs.
- @specs/architecture/agents/overview.md - high-level map of the two main agent graphs (benchmark generator and engineer).
- @specs/architecture/agents/roles.md - role behavior and outputs for planner/implementer/reviewer flows, including sample handoff artifacts.
- @specs/architecture/agents/handover-contracts.md - strict contracts for files and decision flow between planner, implementer, and reviewer stages.
- @specs/architecture/agents/artifacts-and-filesystem.md - runtime framework choices, trace requirements, and filesystem/path-permission contracts.
- @specs/architecture/distributed-execution.md - controller/light/heavy worker topology, routing, persistence, and Temporal boundaries.
- @specs/architecture/evals-and-gates.md - fast/medium/slow eval tiers, quality gates, terminal states, and fail-closed requirements.
- @specs/architecture/simulation-and-dod.md - physics assumptions, allowed mechanisms, constraints, and simulation definition-of-done logic.
- @specs/architecture/observability.md - telemetry schema, event catalog, metrics, lineage IDs, and debugging/backups/review tracking.

## Rules

In this repo, the convention is to not use unit tests because they are flaky, but to use only real integration tests. They are faster to debug against. So, when implementing new logic, verify it against integration tests. The integration test list and spec can be found at @specs/integration-tests.md

______________________________________________________________________

This is all for the critical rules, but please follow the two context rules, they reference two most important documents in the architecture.
