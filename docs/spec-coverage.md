# Problemologist-AI - Spec Coverage Matrix

**Date:** 2026-03-20

This document maps the architecture spec set to the self-contained docs bundle. It exists as an audit aid, not as a dependency for normal backend work.

| Source spec | Covered in docs | Notes |
| --- | --- | --- |
| `desired_architecture.md` | `docs/index.md`, `docs/backend-reference.md` | Navigation entrypoint and backend-first overview |
| `architecture/primary-system-objectives.md` | `docs/project-overview.md`, `docs/backend-reference.md` | Product purpose and outputs |
| `architecture/agents/overview.md` | `docs/backend-reference.md` | Benchmark and engineer graph map |
| `architecture/agents/roles.md` | `docs/backend-reference.md` | Planner, coder, reviewer behavior and artifacts |
| `architecture/agents/handover-contracts.md` | `docs/backend-reference.md` | File-level handoff contracts and refusal routing |
| `architecture/agents/artifacts-and-filesystem.md` | `docs/backend-reference.md`, `docs/development-guide.md` | Runtime framework, permissions, and path contracts |
| `architecture/agents/tools.md` | `docs/backend-reference.md`, `docs/api-contracts.md` | Tool surface, submission gates, and validation helpers |
| `architecture/distributed-execution.md` | `docs/architecture.md`, `docs/backend-reference.md`, `docs/deployment-guide.md` | Controller/worker split, Temporal boundary, and worker admission |
| `architecture/CAD-and-other-infra.md` | `docs/backend-reference.md`, `docs/data-models.md` | Metadata, rendering, workbenches, and supporting infra |
| `architecture/evals-and-gates.md` | `docs/backend-reference.md`, `docs/development-guide.md`, `docs/project-scan-report.json` | Evaluation tiers, terminal states, and fail-closed gates |
| `architecture/simulation-and-dod.md` | `docs/backend-reference.md`, `docs/architecture.md` | Physics contract, constraints realism, and definition-of-done logic |
| `architecture/observability.md` | `docs/backend-reference.md`, `docs/data-models.md` | Traces, events, lineage, backups, and feedback |
| `architecture/fluids-and-deformables.md` | `docs/backend-reference.md`, `docs/data-models.md` | Genesis-only fluid/deformable branch and stress summaries |
| `architecture/electronics-and-electromechanics.md` | `docs/backend-reference.md`, `docs/api-contracts.md`, `docs/data-models.md` | Electronics requirements, circuit validation, and wire routing |
| `integration-tests.md` | `docs/development-guide.md`, `docs/deployment-guide.md`, `docs/api-contracts.md` | Integration-first verification and HTTP boundary contracts |

## Backend-Focused Reading Order

| Need | Primary docs |
| --- | --- |
| Backend runtime and agent contracts | `docs/backend-reference.md` |
| Service topology and routing | `docs/architecture.md` |
| API behavior | `docs/api-contracts.md` |
| Runtime data and policies | `docs/data-models.md` |
| Local setup and verification | `docs/development-guide.md` and `docs/deployment-guide.md` |
| UI-only work | `docs/component-inventory.md` |
