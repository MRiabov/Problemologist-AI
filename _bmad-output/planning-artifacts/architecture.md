---
stepsCompleted:
  - 1
inputDocuments:
  - specs/desired_architecture.md
  - specs/architecture/primary-system-objectives.md
  - specs/architecture/agents/overview.md
  - specs/architecture/agents/roles.md
  - specs/architecture/agents/handover-contracts.md
  - specs/architecture/agents/artifacts-and-filesystem.md
  - specs/architecture/agents/tools.md
  - specs/architecture/distributed-execution.md
  - specs/architecture/CAD-and-other-infra.md
  - specs/architecture/evals-and-gates.md
  - specs/architecture/simulation-and-dod.md
  - specs/architecture/observability.md
  - specs/architecture/fluids-and-deformables.md
  - specs/architecture/electronics-and-electromechanics.md
  - specs/architecture/workbenches.md
  - specs/business-usecase.md
  - specs/dataset-generation.md
  - specs/frontend-specs.md
  - specs/integration-tests.md
  - specs/todos.md
  - _bmad-output/planning-artifacts/prd.md
  - _bmad-output/project-context.md
  - docs/index.md
  - docs/architecture.md
  - docs/project-overview.md
workflowType: 'architecture'
project_name: 'Problemologist-AI'
user_name: 'Max'
date: '2026-03-21'
---

# Architecture Decision Document

_This document builds collaboratively through step-by-step discovery. Sections are appended as we work through each architectural decision together._

## Project Context Analysis

### Requirements Overview

**Functional Requirements:**

- Generate benchmark problems and engineering solutions as separate but linked workflows.
- Preserve planning, implementation, validation, simulation, review, and dataset artifacts for each episode.
- Support benchmark generation with plan/review/coder/review stages and engineering with planner/coder/review stages.
- Enforce manufacturability, cost, weight, and geometry constraints before approval.
- Support live user steerability through CAD/code context selection and interruption.
- Support MuJoCo-based static validation previews and Genesis-backed simulation where required.
- Support read-only benchmark-owned fixtures and engineer-owned manufacturable outputs with strict ownership boundaries.

**Non-Functional Requirements:**

- Fail closed on missing artifacts, stale manifests, schema violations, and missing review evidence.
- Keep integration tests as the real verification path; do not rely on unit-test-style shortcuts for runtime behavior.
- Maintain strict observability: episode lineage, trace reconstruction, review evidence, and simulation outcomes must be queryable.
- Preserve session isolation and prevent cross-episode file leakage.
- Treat visual inspection as a policy-driven requirement when render images exist.
- Avoid hidden degradation: fallback paths must be explicit, not silently treated as success.

**Scale & Complexity:**

- Project complexity appears to be: very high.
- Primary domain: backend-first agentic engineering platform with a secondary React inspection dashboard.
- Estimated architectural components: about 8 to 12 core layers/services, plus shared schemas, runtime policies, and UI surfaces.

### Technical Constraints & Dependencies

- Controller-led orchestration with split `worker-light` and `worker-heavy` services, plus Temporal for durable long-running work.
- PostgreSQL, MinIO/S3, and Langfuse are core platform dependencies.
- build123d, MuJoCo, Genesis, and strict shared Pydantic contracts shape the CAD/simulation boundary.
- COTS search is read-only and must resolve to concrete catalog identities without mutating workspace state.
- Benchmark-owned fixtures, benchmark-owned electronics, and benchmark-owned motion context are read-only task inputs.
- Engineer-owned manufactured parts live in the solution artifact set and are validated/priced separately.
- The docs corpus is in transition: `specs/` is currently the brownfield truth set, but the architecture should define a migration path rather than hard-coding permanent precedence over `docs/`.

### Cross-Cutting Concerns Identified

- Session, episode, simulation, review, and COTS lineage identity.
- Strict handoff contracts between planner, coder, and reviewer stages.
- Validation-preview versus full simulation backend split.
- Dataset persistence, replayability, and provenance.
- Visual evidence enforcement for review stages when renders exist.
- No-silent-fallback behavior for runtime degradation paths.
- Ownership boundaries between benchmark fixtures and engineer solutions.
- Source-of-truth migration risk between `specs/` and `docs/`.

### Failure Modes and Preventive Controls

1. **Source-of-truth drift between `specs/` and `docs/`**
   - Failure: architecture decisions get updated in one place but not the other, so agents follow stale guidance.
   - Cause: the repo is in a migration state, so both doc sets remain visible and usable.
   - Prevention: define an explicit migration contract and retirement path for docs ownership; do not rely on implicit precedence rules.

2. **Planner handoff succeeds without all required artifacts**
   - Failure: planner nodes reach a success-like state even though `plan.md`, `todo.md`, `benchmark_definition.yaml`, or `assembly_definition.yaml` are missing or stale.
   - Cause: permissive gating or fallback logic in submission flow.
   - Prevention: keep planner submission fail-closed, require manifest generation, and make missing/invalid artifacts hard errors.

3. **Reviewers approve stale or mismatched revisions**
   - Failure: reviewer output is based on an older manifest or an artifact set from a different revision.
   - Cause: manifest freshness is not enforced strictly enough.
   - Prevention: tie reviewer entry to latest-revision stage-specific manifests and reject stale or cross-session lookups.

4. **Visual inspection is skipped even when render evidence exists**
   - Failure: a reviewer approves based on text artifacts only while render images are present.
   - Cause: media inspection is treated as optional or UI-only.
   - Prevention: enforce config-driven `inspect_media(...)` requirements for required roles whenever images exist, and persist inspection evidence in traces.

5. **Validation preview is mistaken for full physics proof**
   - Failure: a model passes MuJoCo validation preview, but the backend-specific simulation behavior was never actually established.
   - Cause: preview and simulation responsibilities are conflated.
   - Prevention: keep validation preview explicitly separate from Genesis simulation proof and verify backend parity in dedicated simulation tests.

6. **Episode identity gets conflated with session identity**
   - Failure: trace joins and dataset exports blur one user session into one workflow run.
   - Cause: the system treats `session_id` and `episode_id` as interchangeable.
   - Prevention: propagate both IDs independently, plus `simulation_run_id` and `review_id`, so observability remains reconstructable.

7. **Engineering execution leaks benchmark-owned context into solution ownership**
   - Failure: engineer-owned pricing, manufacturability, or geometry changes accidentally mutate benchmark-owned fixtures.
   - Cause: ownership boundaries are present in spec but not strongly enforced in runtime contracts.
   - Prevention: make benchmark fixtures read-only task context and validate immutability across handoff.

8. **Hidden degradation is reported as success**
   - Failure: fallback paths or degraded behavior look like normal success to the caller.
   - Cause: runtime returns success without explicit degradation metadata.
   - Prevention: make any fallback or degraded path explicit in events and result payloads, and fail closed when degradation is not declared.

9. **Heavy-worker failures cascade into broader service outages**
   - Failure: one simulation crash takes down the service or poisons subsequent jobs.
   - Cause: crash containment and admission control are too loose.
   - Prevention: preserve single-flight admission, isolate heavy execution, and require deterministic retry/fail-closed behavior.

10. **Dataset lineage becomes unusable for training**
    - Failure: artifacts are persisted without reliable seed, variant, or episode linkage.
    - Cause: persistence is treated as storage rather than training-data provenance.
    - Prevention: treat lineage, review evidence, and run metadata as core product outputs, not optional logs.
