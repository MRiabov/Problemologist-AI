# Problemologist-AI Documentation Index

**Type:** Multi-part repository with 4 primary backend runtime parts and 1 secondary UI surface
**Primary Languages:** Python, then TypeScript for the secondary UI
**Architecture:** Backend-first controller-led split-worker platform with strict shared contracts and a secondary React dashboard
**Last Updated:** 2026-03-20

## Project Summary

Problemologist-AI is a brownfield agentic CAD and benchmark-generation platform. It generates benchmark problems, solves them under physics and manufacturability constraints, and persists the resulting traces, reviews, assets, and learned skills for evaluation and dataset creation.

This documentation set is intentionally self-contained. It describes the live backend runtime, service boundaries, APIs, data models, deployment shape, and evaluation contracts without requiring the original architecture notes for day-to-day backend work.

## Primary Backend Runtime Parts

| Part | Location | Purpose | Primary Entry Point |
| -- | -- | -- | -- |
| Controller | [`controller/`](../controller) | Orchestrates episodes, agent graphs, persistence, observability, and worker routing | [`controller/api/main.py`](../controller/api/main.py) |
| Worker Light | [`worker_light/`](../worker_light) | Session filesystem, git, shell execution, linting, asset serving, and lightweight inspection | [`worker_light/app.py`](../worker_light/app.py) |
| Worker Heavy | [`worker_heavy/`](../worker_heavy) | Validation, simulation, preview rendering, workbench analysis, and heavy handoff gating | [`worker_heavy/app.py`](../worker_heavy/app.py) |
| Shared Layer | [`shared/`](../shared) | Pydantic schemas, enums, simulation models, observability models, and worker contracts | [`shared/models/schemas.py`](../shared/models/schemas.py) |

## Secondary Surface

| Surface | Location | Purpose |
| -- | -- | -- |
| Frontend | [`frontend/`](../frontend) | Secondary React dashboard for episode inspection, traces, assets, simulation views, and feedback |

## Supporting Areas

| Area | Location | Notes |
| -- | -- | -- |
| Runtime configuration | [`config/`](../config) | Agent policy, prompt, render, and manufacturability configuration |
| Local automation | [`scripts/`](../scripts) | Environment bootstrap, teardown, and integration-test runner |
| Integration tests | [`tests/`](../tests) | Primary verification suite for the repository |
| Evaluation data | [`dataset/`](../dataset) | Seed data and eval artifacts used by benchmark workflows |
| Skill repository | [`skills/`](../skills) | Runtime-mounted skills used by the agent workflows |
| Website mirror | [`website/`](../website) | Standalone static mirror of the published marketing site |
| Planning artifacts | [`_bmad-output/planning-artifacts/`](../_bmad-output/planning-artifacts) | Generated PRD, epics, stories, and supporting planning outputs |

## Core Docs

- [Project Overview](./project-overview.md)
- [Source Tree Analysis](./source-tree-analysis.md)
- [Backend Reference](./backend-reference.md)
- [System Architecture](./architecture.md)
- [API Contracts](./api-contracts.md)
- [Data Models](./data-models.md)
- [Deployment Guide](./deployment-guide.md)
- [Development Guide](./development-guide.md)
- [Spec Coverage Matrix](./spec-coverage.md)

## Secondary Docs

- [Component Inventory](./component-inventory.md)

## Machine-Readable Metadata

- [Project Parts Metadata](./project-parts.json)
- [Documentation Scan Report](./project-scan-report.json)

## Quick Reference

| Need | Start Here |
| -- | -- |
| Understand the backend runtime | [Backend Reference](./backend-reference.md) |
| Understand the product | [Project Overview](./project-overview.md) |
| Find runtime entry points | [Source Tree Analysis](./source-tree-analysis.md) |
| Understand service boundaries | [System Architecture](./architecture.md) |
| Call APIs correctly | [API Contracts](./api-contracts.md) |
| Modify schemas or persistence | [Data Models](./data-models.md) |
| Run or deploy locally | [Development Guide](./development-guide.md) and [Deployment Guide](./deployment-guide.md) |
| Work on the UI only | [Component Inventory](./component-inventory.md) |

## Getting Started

```bash
./scripts/env_up.sh
./scripts/run_integration_tests.sh
```

If you need the dashboard, start it separately with:

```bash
cd frontend && npm run dev
```

## AI-Assisted Development

| Change Type | Read First |
| -- | -- |
| Backend runtime, orchestration, or gate changes | [Backend Reference](./backend-reference.md), [System Architecture](./architecture.md), and [API Contracts](./api-contracts.md) |
| New or changed schemas | [Data Models](./data-models.md) |
| Planner, reviewer, or handoff changes | [Backend Reference](./backend-reference.md) and [System Architecture](./architecture.md) |
| Local workflow, deploy, or test changes | [Development Guide](./development-guide.md) and [Deployment Guide](./deployment-guide.md) |
| UI or dashboard work | [Component Inventory](./component-inventory.md) |
| Manufacturability, simulation, or observability changes | [Backend Reference](./backend-reference.md), [System Architecture](./architecture.md), and `worker_heavy/` |
