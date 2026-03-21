---
stepsCompleted:
  - 1
  - 2
  - 3
  - 4
  - 5
  - 6
  - 7
  - 8
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
lastStep: 8
status: 'complete'
completedAt: '2026-03-21'
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
- OpenAPI schema generation is part of the repo contract and is automated through git hooks; API changes must regenerate committed schema/client artifacts rather than relying on manual sync.
- COTS search is read-only and must resolve to concrete catalog identities without mutating workspace state.
- Benchmark-owned fixtures, benchmark-owned electronics, and benchmark-owned motion context are read-only task inputs.
- Engineer-owned manufactured parts live in the solution artifact set and are validated/priced separately.
- The docs corpus is in transition: `specs/` and `docs/` both contain brownfield context, so the architecture should acknowledge the migration path rather than assuming a permanent winner.

### Supporting Runtime and Verification Contracts

The following are architecture-adjacent contracts that support the core system but do not define the domain shape by themselves:

- Local debugging can use `codex` CLI or other coding CLIs so seeded eval workspaces can be reproduced without defaulting to paid remote provider calls.
- `dataset/evals/run_evals.py` and `dataset/evals/materialize_seed_workspace.py` belong to the reproducibility and debugging path for evals.
- Worker-side `events.jsonl` is the batch transport artifact for episode observability; the controller ingests it and normalizes it into Postgres rows at episode end. `events.jsonl` is therefore replay/input transport, while Postgres remains the canonical queryable source of truth. Ingestion should be idempotent and fail closed on truncation, duplication, or schema mismatch.
- Structured logs should be consistent across agents, services, and eval helpers, with service-scoped, episode-scoped, and test-scoped attribution.
- Integration-test helper conventions in `specs/integration-tests.md` are part of the supported verification contract, because the repository is validated through those helpers and HTTP-only boundaries.
- OpenAPI schema generation on git hooks is part of the API-contract enforcement path and should keep generated schema/client artifacts in sync with controller behavior.

### System Boundaries Summary

- Core runtime: `controller/`, `worker_light/`, `worker_heavy/`, Temporal, PostgreSQL, MinIO/S3, and Langfuse.
- Shared contracts: `shared/`, `config/`, and the schema/model layers that define boundary payloads and policy.
- Evidence surfaces: `frontend/` renders persisted traces and artifacts; `website/` is separate and does not participate in runtime.
- Supporting tools: `scripts/`, `dataset/evals/`, `tests/`, OpenAPI hooks, structured logs, and local CLI debug/eval loops.
- Deferred surfaces: auth platform, websocket transport, and any frontend rewrite beyond preserving the current dashboard contract.

### Cross-Cutting Concerns Identified

- Session, episode, simulation, review, and COTS lineage identity.
- Strict handoff contracts between planner, coder, and reviewer stages.
- Validation-preview versus full simulation backend split.
- Dataset persistence, replayability, and provenance.
- Visual evidence enforcement for review stages when renders exist.
- No-silent-fallback behavior for runtime degradation paths.
- Ownership boundaries between benchmark fixtures and engineer solutions.
- Source-of-truth migration risk between `specs/` and `docs/`.
- The frontend is a secondary evidence surface for human interpretation; deterministic evaluation and replay belong in backend/eval tooling such as `dataset/evals/run_evals.py`, not in the UI.
- Debugging and verification infrastructure must be reproducible locally through CLI-driven eval/debug loops and structured logs.
- API contract generation and frontend client regeneration are part of the supporting contract surface, not an optional maintenance step.
- Deterministic validation is a cross-cutting application rule; see the decision record below for the canonical contract.

### Episode Lifecycle

1. A benchmark seed or user task enters the controller with immutable task context.
2. The planner writes stage artifacts and manifests.
3. `worker_light` executes workspace changes and emits traces, logs, and `events.jsonl`.
4. `worker_heavy` validates, simulates, renders, and computes deterministic outputs.
5. Reviewers inspect persisted artifacts and media, then write the terminal review decision.
6. The controller ingests events into Postgres and archives artifacts for replay, evaluation, and dataset materialization.

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

11. **Debugging requires expensive remote provider calls or opaque workflows**
    - Failure: engineers cannot reproduce failures cheaply with local tooling.
    - Cause: CLI-based debugging and eval materialization are not first-class architecture concerns.
    - Prevention: make `codex` CLI-based eval reproduction and workspace materialization supported paths.

12. **Logs are too sparse to explain agent or service failures**
    - Failure: debugging requires guessing across multiple subsystems instead of reading one structured trail.
    - Cause: logs are unstructured or lack episode/service/test attribution.
    - Prevention: keep structured logs consistent across agents, services, and eval helpers.

13. **Integration-test helpers drift from runtime behavior**
    - Failure: tests become brittle or misleading because helper conventions differ from actual runtime contracts.
    - Cause: the test harness is treated as external to architecture.
    - Prevention: treat integration-test helper structure and HTTP-only boundaries as part of the supported architecture.

14. **OpenAPI/client artifacts drift from live API behavior**
    - Failure: generated schema or frontend client files no longer match controller behavior.
    - Cause: schema regeneration is skipped, bypassed, or done manually outside the hook contract.
    - Prevention: keep OpenAPI generation and client regeneration hook-driven and part of the normal change flow.

## Starter Template Evaluation

### Primary Technology Domain

Backend-first agentic platform with a secondary React dashboard.

### Starter Assessment

The repository is brownfield and already has the frontend surface, so starter selection is informational rather than prescriptive.

### Selected Starter: None

**Rationale for Selection:**
The current backend and frontend shape are custom and already established. Re-scaffolding would add churn without changing the architecture.

**Future Baseline:**
If a future isolated UI scaffold is needed, use Vite + React + TypeScript rather than rebuilding the existing brownfield frontend.

## Core Architectural Decisions

### Decision Priority Analysis

**Critical Decisions (Block Implementation):**

- Data must stay relational-first: PostgreSQL is the canonical structured store, while MinIO/S3 holds large binary artifacts and render bundles.
- Controller/worker/Temporal boundaries remain the orchestration backbone, with `worker-light` handling workspace execution and `worker-heavy` handling validation/simulation.
- The API contract stays REST-first with explicit event emission for replay, observability, and deterministic evaluation.
- Observability must remain fail-closed and reconstructable, including `events.jsonl` batch transport, controller ingest, and normalized Postgres persistence.

**Important Decisions (Shape Architecture):**

- Auth is intentionally out of scope for this pass; keep the current access model and revisit only if the product scope expands.
- Frontend is a secondary evidence surface and stays on the existing React/Vite contract; no re-platforming or UI-state redesign is authorized in this pass.
- Frontend hosting can be separated from backend hosting, with Vercel or a similar PaaS as the likely future option if the UI becomes externally hosted.

**Deferred Decisions (Post-MVP):**

- Full end-user authentication/authorization stack.
- WebSocket-based live UI transport.
- Any frontend architectural rewrite beyond preserving the current dashboard contract.

### Data Architecture

- PostgreSQL is the authoritative relational store for episodes, traces, review state, lineage, and normalized event records.
- MinIO/S3 is the artifact store for renders, videos, simulation bundles, and other large binary outputs.
- `events.jsonl` is a worker-side batch transport artifact. The controller ingests it at episode end and normalizes it into Postgres rows; the JSONL file is for replay/input transport, not the canonical query surface.
- JSONB should remain a narrow escape hatch, not the primary modeling strategy.
- SQLAlchemy and Alembic remain the persistence/migration path for schema evolution.
- Caching should stay minimal and correctness-preserving; no separate document DB, graph DB, or event-sourcing-only store is required for the current scope.

### Authentication & Security

- No new authentication platform is added in this pass.
- Keep the current controller edge protection and session-scoped worker filesystem policy as the practical security boundary.
- API-key or equivalent current gatekeeping remains acceptable for the brownfield runtime until a product requirement proves otherwise.
- If the application later expands to broader multi-user or public access, the auth model can be revisited as a separate architecture decision.

### API & Communication Patterns

- REST is the primary service/API style for controller and worker boundaries.
- Structured events are part of the contract, not a side channel: they carry traceability, evaluation data, and replay signals.
- OpenAPI generation stays part of the repository contract and should continue to regenerate committed client/schema artifacts.
- WebSockets are a later enhancement for live frontend updates if the UI needs lower-latency streaming, but they are not part of the MVP contract.
- Controller-first communication remains the principle for the frontend and all asset access.

### Frontend Architecture

- The existing frontend remains a secondary React/Vite dashboard and evidence surface.
- Do not re-open state-management, routing, or component-architecture redesign in this pass.
- Preserve the current UI contract: session history, chat/trace inspection, CAD/simulation viewers, code/file viewers, and feedback flow.
- If the frontend becomes an active product workstream after MVP, revisit architecture decisions then rather than forcing them now.

### Infrastructure & Deployment

- Backend deployment stays centered on Railway as the production-oriented host.
- Local development and integration use `docker-compose` plus `./scripts/env_up.sh` / `./scripts/env_down.sh`.
- Temporal remains the durable workflow engine alongside the controller and workers.
- PostgreSQL and MinIO remain the core state and artifact services.
- If the frontend or website needs separate hosting, use Vercel or a similar PaaS rather than coupling it to the backend runtime.
- The deployment shape should preserve process isolation between controller, workers, and heavy simulation execution.

### Decision Impact Analysis

**Implementation Sequence:**

1. Keep the relational schema and artifact storage boundaries stable.
2. Preserve the REST + events contract and OpenAPI generation flow.
3. Wire deployment/runtime assumptions to the Railway + Docker Compose + Temporal stack.
4. Leave frontend architecture unchanged except for hosting/deployment compatibility.
5. Revisit auth and websocket transport only if later product scope requires them.

**Cross-Component Dependencies:**

- Postgres schema and event normalization feed evaluation, replay, and dashboard reconstruction.
- MinIO artifact keys must stay joinable to episode and trace identity for deterministic debugging.
- OpenAPI regeneration drives frontend client compatibility and should move in lockstep with controller API changes.
- REST endpoints, structured events, and Temporal workflow state together define the observable runtime contract.
- The frontend depends on persisted backend traces and artifacts, not synthesized UI-only state.

### Deterministic Validation Decision Record

**Problem Statement:**
- Flaky schemas and unexpected derived-field drift must not reach runtime, persistence, or replay surfaces.

**Options Considered:**
- Boundary-only validation: rejected, because invalid derived values can still circulate internally after admission.
- Entry-only validation: rejected, because it misses corruption introduced during execution, serialization, or handoff.
- Entry + exit + persistence validation: chosen, because it closes the loop on deterministic data.

**Decision:**
- Any value derivable from schema, enums, numeric constraints, arithmetic, referential integrity, or other source fields is computed by deterministic code, not authored by the LLM.
- Derived values are recomputed and equality-checked on node entry and node exit.
- Unknown fields, malformed numbers, invalid enums, stale generated artifacts, and mismatched derived values hard-fail.
- Any mismatch is a bug, not tolerated variance.
- Validation is split by field class: machine-verifiable values are checked deterministically, schema-only values are checked for declared shape and bounds, and freeform or externally sourced values are validated only to the extent the schema and policy allow.
- Numeric validation uses canonical representations, not ambiguous binary-float comparison. Monetary and cost fields should be compared in exact minor units or an equivalent deterministic decimal form.
- Validation failures must emit stable machine-readable diagnostics including field path, derivation source, and expected versus actual values.

**Rationale:**
- The platform depends on reproducible evals, trace replay, and trustworthy episode state.
- Extra helper and schema work is cheaper than chasing silent corruption.

## Implementation Patterns & Consistency Rules

### Pattern Categories Defined

**Critical Conflict Points Identified:** 9 areas where AI agents could make different choices

### Naming Patterns

**Database Naming Conventions:**
- Tables use plural `snake_case` names, for example `episodes`, `episode_traces`, `review_decisions`
- Columns use `snake_case`
- Primary and foreign key fields end in `_id`, for example `episode_id`, `user_session_id`
- Indexes use `idx_<table>_<columns>`
- Timestamp fields use `*_at` in UTC, for example `created_at`, `updated_at`, `completed_at`

**API Naming Conventions:**
- Controller routes use plural resources, for example `/episodes/{episode_id}`
- Path and query parameters use `snake_case`
- Event names use `snake_case`
- OpenAPI-generated client contracts keep backend field names as-is at the boundary

**Code Naming Conventions:**
- Python backend code uses `snake_case` for modules, functions, and variables
- Python classes and Pydantic models use `PascalCase`
- Frontend React components use `PascalCase` file names, while local variables and functions use `camelCase`
- Generated code is consumed as generated; it is not hand-restyled into a different naming convention

### Structure Patterns

**Project Organization:**
- `controller/` owns HTTP APIs, orchestration, persistence, and policy enforcement
- `controller/agent/` owns LangGraph node logic, node-entry validation, handoff routing, and episode state management
- `controller/observability/` owns DB/Langfuse persistence, event plumbing, and trace reconstruction support
- `worker_light/` owns workspace execution, git, linting, and lightweight inspection
- `worker_light/runtime/` and `worker_light/utils/` own the worker session runtime and file-system/tool helpers
- `worker_heavy/` owns validation, simulation, preview rendering, and manufacturability analysis
- `worker_heavy/runtime/`, `worker_heavy/simulation/`, and `worker_heavy/workbenches/` own the heavy execution path and domain-specific validation logic
- `shared/` owns cross-service contracts, strict models, observability schemas, simulation types, COTS helpers, and reusable template fragments
- `shared/models/`, `shared/observability/`, `shared/simulation/`, and `shared/cots/` are the canonical shared contract subtrees
- `frontend/` is the secondary React/Vite dashboard and evidence surface
- `website/` is a separate static/marketing site mirror and does not participate in agent workflows
- `config/` is the policy source of truth for runtime behavior, permissions, render policy, and prompts
- `scripts/` is operational tooling for bootstrap, test, maintenance, and eval helpers
- `dataset/` owns eval seeds, materialized workspaces, and reproducibility inputs
- `tests/` and `dataset/evals/` stay integration/eval-first, not unit-test-first

**File Structure Patterns:**
- Shared reusable starter files live in `shared/agent_templates/common/`
- Worker-local mirrored starter content lives in `worker_light/agent_files/`
- Generated frontend API client files live under `frontend/src/api/generated/`
- Controller-owned generated schema/client assets are kept in the controller/frontend contract path, not duplicated ad hoc elsewhere
- Eval and seed materialization helpers live under `dataset/evals/` and `scripts/`, not in runtime service trees
- Runtime-only artifacts stay in worker session filesystems or object storage, not in ad hoc repo-local paths
- `docs/` documents the live repo shape, while `specs/` contains the architecture source contracts currently being migrated from

### Format Patterns

**API and Data Formats:**
- Boundary payloads use strict typed Pydantic models
- Unknown fields are rejected unless a schema explicitly allows an open-ended map
- JSON/YAML field names stay `snake_case`
- Timestamps are ISO-8601 UTC
- Large binaries stay in object storage; DB rows store metadata and pointers
- Reviewer outputs are strict-schema YAML pairs with stage-specific filenames

**Agent I/O Formats:**
- Agent inputs are role-scoped artifacts plus tool responses, not freeform nested dicts
- Agent outputs are explicit files, tool calls, persisted traces, or strict review artifacts
- Planner submission and reviewer output are terminal format contracts, not advisory text

### Deterministic Validation Rule

- See the `Deterministic Validation Decision Record` above for the canonical contract.
- Operationally, the same rule is enforced at boundary entry, boundary exit, and before persistence across controller APIs, worker tools, shared models, eval materialization, generated clients, review artifacts, and filesystem handoffs.

### Communication Patterns

**Event System Patterns:**
- Event names use `snake_case`
- Long-running or important operations emit start and end records
- `events.jsonl` is the worker-side batch transport format from worker to controller, not the canonical query store
- Controller normalizes worker events into Postgres and Langfuse for replay, evaluation, and debugging

**State and Trace Patterns:**
- Backend state is authoritative
- The frontend renders persisted traces, assets, and metadata only
- No synthetic reasoning rows, guessed tool rows, or UI-only placeholders
- Tool-call traces are emitted at call start and updated at call end
- Reasoning traces are persisted backend records and may stream incrementally; frontend must not synthesize them

### Agentic Infrastructure Patterns

**Agent Runtime Patterns:**
- Agent nodes are role-scoped state machines, not general-purpose chatbots
- Planner, coder, and reviewer stages have fixed handoff contracts and fixed artifact ownership
- Node entry and exit validation are fail-closed; a node only runs when required artifacts and manifests are present and schema-valid
- Any field derivable from other fields is recomputed and equality-checked at both node entry and node exit per the deterministic-validation decision record
- Costing, price, weight, derived caps, reviewer manifests, and other computed fields are never LLM-authored when a deterministic helper can generate them
- If a derived value differs from its deterministic source calculation, the node hard-fails and routes back
- Terminal actions are explicit: `submit_plan`, `submit_review`, and `finish`; no silent success
- Inputs are read-only unless the role explicitly owns the file or artifact
- `inspect_media(...)` is the only visual-evidence path and is required whenever render evidence exists for a role that must inspect it
- `shared/agent_templates/common/` is the shared bootstrap source; `skills/` is the runtime-mounted skill repository; `config/agents_config.yaml` governs permissions and visual policy
- Reasoning and tool traces are the observability substrate; the frontend must not synthesize them
- Worker-light and worker-heavy separation must be preserved so orchestration stays isolated from heavy execution
- `controller/agent/` owns the deterministic graph, while worker services only execute the tool effects the controller routes to them
- Shared agent contracts must stay file- and schema-driven so planner/coder/reviewer roles can be reproduced from persisted artifacts

### Process Patterns

**Error Handling Patterns:**
- Fail closed on missing artifacts, stale manifests, schema mismatches, and missing evidence
- Recompute derivable values on entry, normalize them on exit, and persist only the deterministic result per the deterministic-validation decision record
- No silent fallback from one backend or adapter to another inside a run
- Terminal reasons and failure classes must be explicit
- Durable retries belong to controller/Temporal orchestration, not ad hoc local loops

**Loading State Patterns:**
- Loading states reflect actual backend episode state, not speculative UI state
- Streaming updates should arrive incrementally as traces and events are persisted
- If evidence is required but missing, show a blocked or telemetry-missing state rather than a success-like state

### Enforcement Guidelines

**All AI Agents MUST:**
- Use the shared schema and naming conventions consistently
- Preserve `episode_id` and `user_session_id` in cross-service records
- Treat controller-first persisted traces as the only source of truth for the UI
- Keep REST + events as the current communication pattern and defer websockets to later phases
- Never author a value that can be deterministically derived, validated, or normalized by runtime helpers when the helper contract already exists

**Pattern Enforcement:**
- Verify patterns through integration tests and schema validation
- Record violations in review artifacts, logs, or trace events
- Update patterns only when a real implementation conflict appears

### Pattern Examples

**Good Examples:**
- `shared/models/schemas.py` defines a boundary model reused by controller and workers
- `frontend` reads `/api/episodes/{episode_id}` and renders persisted trace rows
- `website/` is updated independently of controller/worker runtime
- `events.jsonl` is produced by a worker and normalized by the controller

**Anti-Patterns:**
- frontend-synthesized reasoning rows
- duplicate schema definitions in controller and workers
- freeform dict handoffs across agent nodes
- making `website/` influence runtime contracts
- silent fallback between adapters or backends

## Project Structure & Boundaries

### Complete Project Directory Structure

```text
.
├── AGENTS.md
├── README.md
├── pyproject.toml
├── uv.lock
├── alembic.ini
├── docker-compose.yml
├── docker-compose.test.yaml
├── .env
├── .env.example
├── .gitignore
├── .dockerignore
├── .pre-commit-config.yaml
├── .python-version
├── controller_openapi.json
├── worker_openapi.json
├── events.jsonl
├── main.py
├── controller/
│   ├── api/
│   ├── agent/
│   ├── graph/
│   ├── activities/
│   ├── workflows/
│   ├── observability/
│   ├── persistence/
│   ├── clients/
│   ├── middleware/
│   ├── tools/
│   ├── utils/
│   ├── config/
│   ├── evals/
│   └── temporal_worker.py
├── worker_light/
│   ├── app.py
│   ├── api/
│   ├── runtime/
│   ├── tools/
│   ├── utils/
│   └── agent_files/
├── worker_heavy/
│   ├── app.py
│   ├── api/
│   ├── activities/
│   ├── runtime/
│   ├── simulation/
│   ├── utils/
│   └── workbenches/
├── shared/
│   ├── agent_templates/
│   ├── agents/
│   ├── backend/
│   ├── cli/
│   ├── config/
│   ├── cots/
│   ├── models/
│   ├── observability/
│   ├── ops/
│   ├── simulation/
│   ├── utils/
│   └── workers/
├── frontend/
│   ├── src/
│   ├── public/
│   ├── designs/
│   ├── verification/
│   ├── dist/
│   └── openapi.json
├── website/
├── config/
├── dataset/
│   ├── data/
│   └── evals/
├── scripts/
├── tests/
│   ├── integration/
│   ├── e2e/
│   ├── controller/
│   ├── worker_light/
│   ├── worker_heavy/
│   ├── electronics/
│   └── assets/
├── docs/
├── specs/
├── skills/
├── assets/
├── agent_runtime_utils/
├── evals/
├── logs/
├── renders/
├── test_output/
├── code-reviews/
└── .worktrees/
```

### Architectural Boundaries

**API Boundaries:**
- `controller/api/` is the public controller surface and owns the REST entrypoints for the runtime.
- `worker_light/api/` and `worker_heavy/api/` expose worker-local operational routes only; they do not own the system contract.
- `controller_openapi.json`, `worker_openapi.json`, and `frontend/openapi.json` are generated contract artifacts and must track source APIs exactly.
- REST plus structured events is the current communication model; websockets remain deferred.

**Component Boundaries:**
- `controller/agent/` and `controller/graph/` own LangGraph state, node validation, handoff routing, and steerability.
- `worker_light/` owns workspace execution, filesystem edits, lightweight inspection, and git/tool effects.
- `worker_heavy/` owns validation, simulation, rendering, manufacturability, and physics-backed checks.
- `shared/` owns cross-service schemas, enums, observability contracts, simulation types, COTS helpers, and reusable templates.
- `frontend/` is a secondary evidence surface that renders persisted backend state only.
- `website/` is separated from the runtime and remains outside agent workflows.

**Service Boundaries:**
- `controller/workflows/` and `controller/temporal_worker.py` own durable orchestration.
- `worker_heavy/temporal_worker.py` owns heavy simulation execution when Temporal dispatches it.
- `controller/observability/` and `shared/observability/` own trace ingest, persistence, replay support, and event normalization.
- `controller/evals/` and `dataset/evals/` own reproducibility and deterministic eval materialization.
- `scripts/` owns bootstrap, maintenance, and integration-test orchestration.

**Data Boundaries:**
- PostgreSQL is the canonical structured store and is accessed through controller persistence and migrations.
- MinIO/S3 holds render bundles, videos, simulation artifacts, and other large binaries.
- `events.jsonl` is worker-side batch transport; the controller ingests and normalizes it into relational rows at episode end.
- Worker session filesystems are isolated and hold runtime-only workspace state.
- Generated artifacts are committed or materialized through the contract path; they are not hand-authored ad hoc.

### Requirements to Structure Mapping

**Benchmark Generation and Planning**
- `controller/agent/benchmark_handover_validation.py`
- `controller/agent/graph.py`
- `controller/graph/agent.py`
- `controller/graph/steerability_node.py`
- `controller/workflows/execution.py`
- `shared/workers/benchmark_definition_template.py`
- `shared/workers/schema.py`
- `tests/e2e/test_benchmark_generation.py`
- `tests/integration/`

**Engineer Execution and Review**
- `controller/agent/node_entry_validation.py`
- `controller/agent/review_handover.py`
- `controller/workflows/heavy.py`
- `worker_light/runtime/executor.py`
- `worker_light/utils/filesystem.py`
- `worker_light/utils/git.py`
- `worker_heavy/simulation/`
- `worker_heavy/workbenches/`
- `worker_heavy/utils/validation.py`
- `tests/worker_light/`
- `tests/worker_heavy/`
- `tests/integration/test_full_workflow.py`

**Observability, Replay, and Debugging**
- `controller/observability/`
- `shared/observability/`
- `shared/models/observability.py`
- `shared/observability/events.py`
- `shared/observability/persistence.py`
- `dataset/evals/run_evals.py`
- `dataset/evals/materialize_seed_workspace.py`
- `events.jsonl`
- `logs/`
- `tests/integration/`

**Simulation and Manufacturability**
- `worker_heavy/simulation/`
- `worker_heavy/utils/cad.py`
- `worker_heavy/utils/dfm.py`
- `worker_heavy/utils/constraints.py`
- `worker_heavy/utils/preview.py`
- `worker_heavy/workbenches/`
- `tests/worker_heavy/test_physics_backends.py`
- `tests/worker_heavy/test_genesis_interaction.py`
- `tests/worker_heavy/test_simulation_builder_electronics.py`

**Frontend and Website**
- `frontend/src/`
- `frontend/public/`
- `frontend/designs/`
- `frontend/verification/`
- `frontend/openapi.json`
- `website/`

**Local Development and Tooling**
- `docker-compose.yml`
- `docker-compose.test.yaml`
- `scripts/env_up.sh`
- `scripts/env_down.sh`
- `scripts/run_integration_tests.sh`
- `config/agents_config.yaml`
- `config/prompts.yaml`
- `config/manufacturing_config.yaml`
- `config/reward_config.yaml`
- `config/lint_config.yaml`
- `config/generator_config.yaml`
- `.pre-commit-config.yaml`

### Generated and Runtime Surfaces

- `logs/`, `renders/`, `test_output/`, `code-reviews/`, and `.worktrees/` are runtime or scratch surfaces, not source roots.
- `controller_openapi.json`, `worker_openapi.json`, `frontend/openapi.json`, and `events.jsonl` are generated or derived artifacts that must stay in sync with source contracts.
- Hidden tool directories such as `.agents/`, `.codex/`, `.claude/`, `.gemini/`, `.kittify/`, `.jules/`, and `.vscode/` are local environment metadata and do not define the product architecture.
