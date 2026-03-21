---
stepsCompleted:
  - step-01-validate-prerequisites
  - step-02-design-epics
  - step-03-create-stories
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
  - specs/architecture/workbenches.md
  - specs/architecture/electronics-and-electromechanics.md
  - specs/architecture/fluids-and-deformables.md
  - specs/architecture/observability.md
  - specs/frontend-specs.md
  - specs/business-usecase.md
  - specs/dataset-generation.md
  - specs/integration-tests.md
  - specs/todos.md
---

# Problemologist-AI - Epic Breakdown

## Overview

This document is a regenerated planning artifact for Problemologist-AI. It keeps functional requirements, non-functional requirements, architecture/file-contract requirements, and UX requirements separated so the epic list stays aligned with current source specs instead of stale backlog notes.

## Requirements Inventory

### Functional Requirements

- FR1: The platform shall isolate each user session and episode in its own workspace, persist generated files and traces across turns, and support resumption after interruption.
- FR2: The platform shall route long-running validation and simulation to `worker-heavy` through Temporal with single-flight admission and deterministic busy responses.
- FR3: The platform shall classify validation and simulation results, including goal-hit, forbid-hit, out-of-bounds, timeout, instability, and breakage outcomes, and aggregate jittered runs when applicable.
- FR4: The platform shall persist incremental reasoning, tool-call, simulation, review, and lineage evidence with joinable session, episode, run, review, and seed IDs.
- FR5: The platform shall enforce strict schema validation, stage manifests, and fail-closed terminal states for planner/reviewer handoffs and evaluation transitions.
- FR6: The platform shall support benchmark generation, benchmark review, benchmark implementation, and benchmark refusal/handoff workflows.
- FR7: The platform shall support engineering planning, plan review, unified mechanical and electrical coding, refusal, and execution review workflows.
- FR8: The platform shall expose canonical agent utilities for validation, pricing, simulation, rendering, review submission, and docs lookup.
- FR9: The platform shall enforce workspace-relative filesystem permissions, read-only mounts, and denied writes to protected directories such as `.manifests/**`.
- FR10: The platform shall support manufacturable CAD solutions with COTS search, workbench/material pricing, drilling cost accounting, and joint/fastener validation.
- FR11: The platform shall provide a shared inspection workspace UI with session history, chat, CAD, code, electronics views, plan approval controls, and feedback.
- FR12: The platform shall support live steering via selected CAD and code context, plus interrupt controls during generation.
- FR13: The platform shall support electromechanical planning and implementation, including power, circuits, wires, power-gated actuation, and actuator control modes.
- FR14: The platform shall support fluid and deformable-material benchmarks with backend-appropriate validation and failure semantics.
- FR15: The platform shall persist dataset-ready bundles, clean invalid or polluted runs, and prioritize underrepresented seeds for reuse or training.
- FR16: The platform shall keep benchmark-owned fixtures, benchmark-owned electronics, and benchmark motion metadata as read-only environment inputs during downstream engineering stages.

### Non-Functional Requirements

- NFR1: Planner and reviewer machine-readable artifacts shall be strict-schema documents and shall reject unknown or extra fields recursively.
- NFR2: Required reasoning, tool-call, and review traces shall be emitted incrementally; missing required traces shall fail closed instead of synthesizing data.
- NFR3: When render images exist, reviewer approval shall require `inspect_media(...)`; file listings or textual descriptions do not satisfy the visual-inspection policy.
- NFR4: The heavy-worker service shall remain single-flight and shall not implement an internal multi-job queue.
- NFR5: Validation preview shall use MuJoCo by default and shall not be treated as proof of simulation-backend parity.
- NFR6: Heavy simulation and validation shall remain crash-contained behind isolated worker processes and durable Temporal execution boundaries.
- NFR7: Critical APIs shall remain schema-valid and compatible with HTTP-only integration verification against the running compose stack.
- NFR8: Observability shall capture required IDs, seeds, events, and error streams so runs remain queryable and attributable.
- NFR9: Terminal states shall always include explicit failure classification and deterministic reason codes.
- NFR10: Run provenance shall remain reproducible across seeds, runtime jitter, static variants, and backend selection.
- NFR11: The UI shall stream workflow updates in near real time and expose context usage and compaction telemetry.
- NFR12: File-system policy shall prevent writes to forbidden mounts and `.manifests/**`.
- NFR13: Known fallback or degraded behavior shall be explicit and machine-readable; silent success-like degradation is not allowed.
- NFR14: Artifact persistence shall remain traceable through database, object storage, and review files without losing session/episode linkage.
- NFR15: Skills and shared boilerplate artifacts shall remain runtime-mounted read-only inputs rather than workspace writes.

### Architecture and File Contract Requirements

- The benchmark planner handoff shall include `plan.md`, `todo.md`, `benchmark_definition.yaml`, and `benchmark_assembly_definition.yaml`, and planner submission shall persist `.manifests/benchmark_plan_review_manifest.json`.
- The engineering planner handoff shall include `plan.md`, `todo.md`, `benchmark_definition.yaml`, and `assembly_definition.yaml`, and planner submission shall persist `.manifests/engineering_plan_review_manifest.json`.
- Reviewers shall persist stage-specific YAML decision/comments pairs under `reviews/`, and routing shall depend only on the decision YAML.
- `benchmark_definition.yaml` shall carry objective zones, runtime randomization, and planner-authored estimate fields that derive `max_unit_cost` and `max_weight_g`.
- `benchmark_definition.yaml.moved_object.material_id` shall resolve to a known material in `manufacturing_config.yaml`.
- `benchmark_assembly_definition.yaml` shall always be a schema-valid full `AssemblyDefinition`, even when the benchmark uses a minimal fixture declaration.
- `benchmark_definition.yaml.benchmark_parts` shall represent benchmark-owned fixtures, including attachment policy and drill policy, and those fixtures shall remain read-only for engineering.
- `assembly_definition.yaml.environment_drill_operations` shall validate against benchmark-side drill policy and shall add benchmark drilling cost when present.
- `assembly_definition.yaml.electronics` shall hold the engineer-owned electrical design, while benchmark-owned electrical constraints remain in the benchmark definition.
- Physical wire routes shall be waypoint-based, gauge-aware, length-aware, and clearance-validated, with wire tear reducing power to the affected path.
- Static preview renders shall produce `renders/render_manifest.json` with per-image modality metadata and segmentation legends that distinguish repeated instances.
- Render modality emission shall be controlled by `config/agents_config.yaml`.
- `controller_openapi.json`, `worker_openapi.json`, and `frontend/openapi.json` shall remain generated contract artifacts, and API changes shall regenerate them through the repository hook flow rather than by manual sync.
- `events.jsonl` shall remain the worker-side batch transport artifact for episode observability; the controller shall ingest it at episode end and normalize it into canonical database rows, failing closed on truncation, duplication, or schema mismatch.
- The worker filesystem shall keep `/utils`, `/skills`, `/reviews`, and `/config` read-only while leaving the workspace root writable.
- Heavy operations shall use Temporal as the durable execution boundary, and direct `worker-heavy` HTTP endpoints shall remain integration-test and debug only.
- `inspect_media(...)` shall be the only agent-facing way to inspect render images or video frames.
- Controller-first communication shall remain the principle for frontend and asset access, with the controller brokering persisted artifacts and backend state for the UI.
- When identical starter artifacts recur, shared boilerplate shall be reused from `shared/agent_templates/common/` rather than duplicated in seed rows.
- Deferred backlog items in `specs/todos.md` are intentionally not promoted into the current epic scope unless they already affect a current contract.

### UX Design Requirements

- UX-DR1: The UI shall use a resizable three-column layout with session history, chat, and a right-side CAD/filesystem area for both benchmark and engineering workflows.
- UX-DR2: The benchmark and engineering views shall share the same shell structure so users move between flows without relearning the interface.
- UX-DR3: Chat shall render reasoning traces from persisted backend trace records only, with expand/collapse control and no synthetic placeholders.
- UX-DR4: Chat shall render tool-call rows from backend traces only, including readable file and directory labels derived from the trace payload.
- UX-DR5: Chat shall expose an interrupt or stop control while the agent is generating.
- UX-DR6: Chat shall show context usage and conversation compaction telemetry from backend metadata.
- UX-DR7: The CAD viewer shall support topology display, part hide/show, and simulation playback with time rewind and fast-forward.
- UX-DR8: The CAD viewer shall support selecting parts, faces, edges, and subassemblies and passing those selections into prompt context.
- UX-DR9: The code viewer shall provide a file tree, syntax highlighting, and line numbers, and line selection shall add code context.
- UX-DR10: The electronics view shall support a schematic view and a 3D wire-route view linked to the current episode.
- UX-DR11: The interface shall support light and dark themes with a clear user toggle.
- UX-DR12: Users shall be able to submit thumbs up/down feedback with an editable comment box and common topic tags.
- UX-DR13: Plan approval and disapproval controls shall be visible in the workflow UI after planning completes.
- UX-DR14: Context cards shall be shown for selected parts, code regions, and other prompt context items, with explicit remove controls.
- UX-DR15: Model and tool activity shall appear in the UI as soon as the backend receives it, rather than only after completion.

### FR Coverage Map

FR1: Epic 1 - Session isolation, durable artifact persistence, and resumability
FR2: Epic 1 - Heavy-worker routing and single-flight admission
FR3: Epic 1 - Result taxonomy and batched jittered validation
FR4: Epic 2 - Incremental traces and joinable lineage IDs
FR5: Epic 3 - Strict schema validation, manifests, and fail-closed gates
FR6: Epic 8 - Benchmark planner, reviewer, coder, and refusal workflow
FR7: Epic 4 - Engineering planner, reviewer, coder, and refusal workflow
FR8: Epic 4 - Canonical validation, pricing, simulation, rendering, review, and docs utilities
FR9: Epic 1 - Filesystem isolation and protected mounts
FR10: Epic 5 - Manufacturable CAD, COTS, pricing, and joint/fastener checks
FR11: Epic 7 - Shared inspection UI and feedback controls
FR12: Epic 6 - CAD and code context steering plus interrupt handling
FR13: Epic 9 and Epic 10 - Electromechanical power, wiring, and actuator control
FR14: Epic 11 and Epic 12 - Fluids, deformables, and failure semantics
FR15: Epic 13 - Dataset-ready bundles and corpus cleaning/prioritization
FR16: Epic 8 - Read-only benchmark fixtures and motion metadata during handoff

## Epic List

### Epic 1: Provide agent runtime and persistence
Users can execute agent actions in a session-scoped workspace with durable persistence, isolated execution, and fail-closed runtime gates.
**FRs covered:** FR1, FR2, FR3, FR9

### Epic 2: Capture observability, lineage, and review evidence
Users can reconstruct what happened, where, and why through complete traces, IDs, and evidence links.
**FRs covered:** FR4

### Epic 3: Build evaluation and verification infrastructure
Users can validate handoff artifacts, generated API contracts, and cross-contract checks, and keep seeded and integration verification fail-closed.
**FRs covered:** FR5

### Epic 4: Run engineering planner, reviewer, coder, refusal, and execution review workflows
Users can move engineering work through the correct handoff gates with the right files, manifests, and submission controls.
**FRs covered:** FR7, FR8

### Epic 5: Design manufacturable CAD solutions with validated materials and COTS
Users can build priced, manufacturable mechanical solutions from real materials, joints, workbenches, and catalog parts.
**FRs covered:** FR10

### Epic 6: Steer agents with precise context and prompt control
Users can point to exact parts, lines, and code context so the agent receives the right local evidence and prompt hints.
**FRs covered:** FR12

### Epic 7: Provide the interactive workspace and feedback UI
Users can monitor sessions, inspect CAD and code, interrupt runs, and rate outputs in one shared interface.
**FRs covered:** FR11

### Epic 8: Generate and certify benchmarks
Users can author benchmark problems, validate geometry and randomization, and certify solved benchmark packages for engineering intake.
**FRs covered:** FR6, FR16

### Epic 9: Solve electromechanical wiring and circuit problems
Users can specify power, circuits, wires, and physical wire routing, and reject invalid electrical designs before simulation.
**FRs covered:** FR13

### Epic 10: Solve powered electromechanical mechanisms
Users can plan and optimize powered mechanisms with electrical planning, specialist review, and unified implementation.
**FRs covered:** FR13

### Epic 11: Model fluids and fluid-electronics coupling
Users can define fluid tasks, run them on Genesis, and treat fluid exposure to electronics as a hard failure mode.
**FRs covered:** FR14

### Epic 12: Validate deformables, stress, and breakage
Users can reason about FEM-enabled materials, stress summaries, and breakage outcomes for structural tasks.
**FRs covered:** FR14

### Epic 13: Generate, clean, and recycle dataset-ready artifacts
Researchers and companies can produce training- and RL-ready data from completed runs and filter polluted or underrepresented data.
**FRs covered:** FR15

## Epic 1: Provide agent runtime and persistence
Users can execute agent actions in a session-scoped workspace with durable persistence, isolated execution, and fail-closed runtime gates.

### Story 1.1: Isolated session workspaces and durable artifacts
As a human software engineer, I want my controller-managed LLM agent to use a separate workspace and trace history for each episode, so that files, assets, and reasoning never leak across runs.

**Acceptance Criteria:**

**Given** two episodes are active in different sessions
**When** each episode writes and reads workspace files or session assets
**Then** each episode only sees its own files and artifacts

**Given** an episode is resumed after a pause or controller restart
**When** the agent continues the same episode ID
**Then** previously persisted artifacts and trace history are still available

### Story 1.2: Route heavy work off the controller
As a human software engineer, I want my `Engineering` or `Benchmark` LLM agent to run validation and simulation through the heavy-worker path, so that long-running work is isolated and durable, but my controller and light worker servers remain responsive.

**Acceptance Criteria:**

**Given** a heavy request is issued
**When** the controller dispatches it
**Then** the work executes on worker-heavy through Temporal rather than on the controller process

**Given** worker-heavy is already processing one job
**When** a second heavy request arrives
**Then** the response is a deterministic busy result and no internal queue is created

### Story 1.3: Classify validation and simulation outcomes
As a human software engineer, I want my `Engineering Coder` LLM agent to report distinct validation and simulation terminal states, so that I can trust robustness checks across jittered scenes.

**Acceptance Criteria:**

**Given** a solution is validated
**When** preview renders are generated
**Then** the static preview path uses the validation backend and does not mutate simulation backend selection

**Given** one admitted job runs multiple jittered scenes
**When** simulation completes
**Then** pass/fail statistics are aggregated and outcomes are classified as goal-hit, forbid-hit, out-of-bounds, timeout, instability, or breakage

### Story 1.4: Protect runtime mounts and generated manifests
As a human software engineer, I want the runtime filesystem policy to keep shared mounts read-only and reject agent writes to protected paths, so that generated artifacts and policy directories stay isolated from workspace edits.

**Acceptance Criteria:**

**Given** an agent tries to write to `/utils`, `/skills`, `/reviews`, `/config`, or `.manifests/**`
**When** the filesystem policy evaluates the request
**Then** the write is rejected and the workspace remains unchanged

**Given** the agent writes into the workspace root
**When** the path is allowed by role policy
**Then** the write succeeds without altering read-only mounted inputs

## Epic 2: Capture observability, lineage, and review evidence
Users can reconstruct what happened, where, and why through complete traces, IDs, and evidence links.

### Story 2.1: Stream reasoning and tool traces
As a human debugger or reviewer, I want reasoning and tool-call traces to be recorded incrementally, so that I can inspect what the agent is doing while it runs.

**Acceptance Criteria:**

**Given** an episode is running
**When** the agent reasons or calls tools
**Then** trace records are persisted incrementally with step index and source metadata

**Given** the runtime requires reasoning traces and none are produced
**When** the episode completes
**Then** the system fails closed or surfaces an explicit missing-telemetry state instead of synthesizing fake traces

### Story 2.2: Correlate runs, reviews, and assets
As a human evaluator, I want every run artifact to carry joinable lineage IDs, so that I can reconstruct the exact session, episode, and revision that produced it.

**Acceptance Criteria:**

**Given** a run is persisted
**When** I inspect trace, asset, review, or simulation metadata
**Then** I can join the artifact back to user_session_id, episode_id, simulation_run_id, cots_query_id, review_id, and seed lineage where applicable

**Given** media is inspected for review
**When** the media-inspection action is recorded
**Then** the inspection event is linked to the exact episode and revision that was viewed

### Story 2.3: Normalize worker event batches into canonical records
As a human evaluator, I want worker-side `events.jsonl` batches to be ingested into canonical database rows, so that replay, audit, and dataset extraction use the same event source of truth.

**Acceptance Criteria:**

**Given** a worker finishes an episode and writes `events.jsonl`
**When** the controller ingests the batch
**Then** the events are normalized into canonical Postgres rows, ingestion is idempotent, and the rows remain joinable to the episode and revision

**Given** `events.jsonl` is truncated, duplicated, or schema-invalid
**When** ingestion runs
**Then** ingestion fails closed and no success-like event state is persisted

## Epic 3: Build evaluation and verification infrastructure
Users can validate handoff artifacts, enforce strict schema and cross-contract checks, and keep seeded and integration verification fail-closed.

### Story 3.1: Validate planner and reviewer artifacts strictly
As a human software engineer, I want my planner/reviewer validation pipeline to reject unknown fields in every schema-backed handoff artifact, so that invalid structure never slips through as success.

**Acceptance Criteria:**

**Given** a planner or reviewer YAML/JSON artifact contains unknown fields
**When** schema validation runs
**Then** validation rejects the artifact recursively, including nested objects

**Given** a schema-backed artifact is malformed or missing required fields
**When** node-entry preflight checks it
**Then** execution stops before the node continues

### Story 3.2: Gate transitions with stage manifests
As a human software engineer, I want my workflow orchestrator to gate planner and reviewer entry with stage-specific manifests, so that stale or missing handoffs cannot proceed.

**Acceptance Criteria:**

**Given** a planner handoff reaches review
**When** the stage manifest is missing, stale, or schema-invalid
**Then** reviewer entry is blocked with a fail-closed error and the latest revision is not accepted

**Given** a valid latest-revision manifest exists
**When** the next reviewer stage starts
**Then** it consumes the correct stage-specific artifact pair and persists the correct reviewer-scoped outputs

### Story 3.3: Fail closed on evaluation and fallback paths
As a human software engineer, I want my evaluation pipeline to fail closed on cross-contract errors during seeded preflight and terminalization, so that degraded or fallback behavior never looks like clean success.

**Acceptance Criteria:**

**Given** a seeded workspace contains invalid present artifacts
**When** preflight runs
**Then** the run stops before node execution and emits explicit reason codes

**Given** a known fallback or degradation path is triggered
**When** the response is returned
**Then** it includes machine-readable degraded/reason data and a matching event instead of a silent success

**Given** an integration-mode handoff validation fails
**When** the episode terminalizes
**Then** it enters FAILED with structured terminal_reason and failure_class fields

### Story 3.4: Keep generated API contracts in sync
As a human software engineer, I want generated OpenAPI artifacts to stay synchronized with the controller and worker source contracts, so that client generation and integration tests never drift from the live API shape.

**Acceptance Criteria:**

**Given** controller, worker, or frontend API surfaces change
**When** the repository hook flow runs
**Then** `controller_openapi.json`, `worker_openapi.json`, and `frontend/openapi.json` regenerate from source contracts and stay in sync with the live API shape

**Given** the generated schema or client artifacts are stale or invalid
**When** integration verification runs
**Then** the drift is treated as a failure and the change cannot pass as clean success

## Epic 4: Run planner, reviewer, coder, and refusal workflows
Users can move benchmark and engineering work through the correct handoff gates with the right files, manifests, and submission controls.

### Story 4.1: Expose canonical planning and review tools
As a human software engineer, I want my `Planner`, `Coder`, and `Reviewer` LLM agents to use a stable `utils` tool surface, so that validation, pricing, simulation, rendering, and docs lookup use the same entrypoints everywhere.

**Acceptance Criteria:**

**Given** a role imports the canonical helpers
**When** it invokes validation, pricing, simulation, render, review, or docs utilities
**Then** those calls resolve through the top-level `utils` contract rather than ad hoc local paths

**Given** a role attempts to write a forbidden path
**When** the filesystem policy evaluates the request
**Then** the operation is denied and reviewer writes remain limited to the stage-specific YAML pair

### Story 4.2: Author and submit engineering planner handoffs
As a human software engineer, I want my `Engineering Planner` LLM agent to create the required handoff files and submit them through the planner gate, so that the next stage receives a validated plan.

**Acceptance Criteria:**

**Given** `plan.md`, `todo.md`, `benchmark_definition.yaml`, and `assembly_definition.yaml` are present and valid
**When** `submit_plan()` runs
**Then** the planner transitions to PLANNED and `.manifests/engineering_plan_review_manifest.json` is created

**Given** any required engineering planner file is missing or invalid
**When** `submit_plan()` runs
**Then** handoff fails closed and no success-like state is emitted

### Story 4.3: Review engineering plans and refusal evidence
As a human software engineer, I want my `Engineering Plan Reviewer` LLM agent to accept or refuse a plan with stage-specific YAML outputs, so that routing is deterministic and explainable.

**Acceptance Criteria:**

**Given** a valid engineering plan
**When** review passes
**Then** the decision and comments YAML files are written for the correct stage and round

**Given** a plan refusal is submitted with proof
**When** the reviewer confirms the refusal
**Then** routing returns to the planner and the confirm/reject decision is recorded

**Given** refusal output is missing evidence or has invalid schema
**When** the reviewer checks it
**Then** the refusal is rejected and cannot route

### Story 4.4: Refuse infeasible engineering plans
As a human software engineer, I want my `Engineering Coder` LLM agent to implement approved plans or refuse infeasible ones with evidence, so that only viable plans proceed to simulation.

**Acceptance Criteria:**

**Given** an approved plan is feasible
**When** coding is complete
**Then** the implementation can be submitted only after validation and simulation gates pass

**Given** a plan is impossible or internally inconsistent
**When** the coder refuses it
**Then** `plan_refusal.md` records role-specific reasons and proof, and the refusal stops further execution

**Given** the refusal is reviewed
**When** the reviewer confirms or rejects it
**Then** routing follows the reviewer decision deterministically

### Story 4.5: Review validated engineering executions
As an `Engineering Execution Reviewer`, I want to inspect the latest validated implementation and simulation evidence, so that only the current revision can pass final review.

**Acceptance Criteria:**

**Given** the latest revision has valid validation and simulation artifacts
**When** execution review starts
**Then** `.manifests/engineering_execution_review_manifest.json` is present and tied to the latest revision

**Given** the implementation is stale, over-actuated, or missing evidence
**When** execution review runs
**Then** the reviewer rejects it and persists the stage-specific decision/comments YAML pair

## Epic 5: Design manufacturable CAD solutions with validated materials and COTS
Users can build priced, manufacturable mechanical solutions from real materials, joints, workbenches, and catalog parts.

### Story 5.1: Search COTS parts with reproducible metadata
As a human software engineer, I want my `Benchmark Planner` or `Engineering Planner` LLM agent to search the COTS catalog and carry selected part metadata into planning artifacts, so that component choices are real and reproducible.

**Acceptance Criteria:**

**Given** a catalog query is issued
**When** COTS search runs
**Then** returned candidates include part identity, manufacturer, specs, price, source, and reproducibility metadata or a no-match rationale, and the search does not mutate workspace state beyond allowed journal logging

**Given** a COTS part is selected
**When** the plan is saved
**Then** the part ID and catalog metadata are persisted into the plan and cost artifacts

### Story 5.2: Price manufacturable parts by workbench and material
As a human software engineer, I want my `Engineering Planner` LLM agent to use the selected manufacturing method, material, and drill operations for pricing and manufacturability checks, so that budget estimates match build reality.

**Acceptance Criteria:**

**Given** a part uses a supported method and material
**When** `validate_and_price` runs
**Then** it returns cost and weight data for supported workbench methods (`CNC`, `injection molding`, or `3D printing`) or rejects the part if the combination is invalid

**Given** environment drilling is declared
**When** pricing runs
**Then** drilling cost is included and the handoff fails closed if the limits are exceeded

**Given** planner-owned max cost and weight are derived
**When** the plan is validated
**Then** they remain under the benchmark/customer caps

### Story 5.3: Map joints and fasteners to simulator constraints
As a human software engineer, I want my `Engineering Coder` LLM agent to map build123d joints and fasteners to simulator constraints, so that the solution is physically realizable instead of symbolic only.

**Acceptance Criteria:**

**Given** two parts are connected
**When** rigid-joint validation runs
**Then** hole sizes, fastener types, and connection geometry are compatible

**Given** a connection is underconstrained or uses unsupported DOF patterns
**When** validation runs
**Then** it is rejected before simulation

**Given** the assembly is exported
**When** simulator constraints are generated
**Then** they correspond to the authored joints without inventing invisible connections

## Epic 6: Steer agents with precise context and prompt control
Users can point to exact parts, lines, and code context so the agent receives the right local evidence and prompt hints.

### Story 6.1: Add CAD context cards from selected geometry
As a human engineer user, I want to click parts, faces, edges, and subassemblies into prompt context cards, so that I can steer the agent with precise geometry references.

**Acceptance Criteria:**

**Given** a model element is selected
**When** I add it to context
**Then** the UI shows a card with the correct semantic label and a removal control

**Given** multiple elements are selected with multi-select
**When** they are added to context
**Then** each selection is preserved independently in the context payload

### Story 6.2: Add code span context with line validation
As a human engineer user, I want to add exact file spans via `@path/file.py:start-end`, so that the agent sees the right source lines and invalid ranges fail visibly.

**Acceptance Criteria:**

**Given** a valid line span
**When** I add it to context
**Then** the agent receives the exact requested span

**Given** an invalid or out-of-range reference
**When** I add it to context
**Then** the UI shows a validation error instead of silently accepting it

### Story 6.3: Keep context payload assembly server-side
As a human engineer user, I want the frontend to send selected context as a structured payload only, so that prompt assembly stays on the backend and cards can be removed cleanly.

**Acceptance Criteria:**

**Given** multiple context cards exist
**When** I remove one
**Then** it disappears from the visible card stack and the payload

**Given** a prompt is sent
**When** the frontend transmits the request
**Then** it passes the selected context set without concatenating prompt text itself

## Epic 7: Provide the interactive workspace and feedback UI
Users can monitor sessions, inspect CAD and code, interrupt runs, and rate outputs in one shared interface.

### Story 7.1: Share the same workspace shell across flows
As a human engineer user, I want benchmark and engineering workspaces to share the same resizable three-column shell, so that I can move between flows without relearning the UI.

**Acceptance Criteria:**

**Given** either flow is opened
**When** the workspace renders
**Then** session history, chat, and right-side artifacts appear in the same shell layout and are accessed through the controller-proxied asset path

**Given** the panes are resized
**When** the browser viewport changes
**Then** the layout remains usable on desktop and mobile

### Story 7.2: Stream chat traces and interruption controls
As a human engineer user, I want reasoning and tool calls to stream into chat from backend traces, so that I can see what the agent is doing in real time.

**Acceptance Criteria:**

**Given** an agent run is active
**When** backend emits reasoning or tool events
**Then** the chat timeline updates without placeholder rows and shows backend-derived tool labels

**Given** an interrupt is sent
**When** generation stops
**Then** the UI shows the interrupted state and the agent no longer streams new content

**Given** context usage or compaction metadata exists
**When** I inspect the chat surface
**Then** the telemetry is shown from backend metadata

### Story 7.3: Inspect CAD and simulation playback
As a human engineer user, I want to inspect topology, hide or show parts, and scrub simulation playback, so that I can understand the mechanical solution visually.

**Acceptance Criteria:**

**Given** a completed episode exists
**When** I open the viewer
**Then** topology and part visibility controls are available

**Given** simulation artifacts exist
**When** I play or scrub them
**Then** time can move forward and backward without losing the episode context

**Given** benchmark objective boxes exist
**When** they are rendered
**Then** their semantic colors remain visible in the viewer

### Story 7.4: Inspect code and electronics views
As a human engineer user, I want to browse code and electronics artifacts with accurate syntax and schematic views, so that I can inspect implementation details in context.

**Acceptance Criteria:**

**Given** a code bundle exists
**When** I open the code viewer
**Then** the file tree, syntax highlighting, line numbers, and line selection are present

**Given** electronics artifacts exist
**When** I switch views
**Then** schematic and 3D wire-route views are linked to the same episode

### Story 7.5: Approve work and capture feedback
As a human engineer user, I want plan approval buttons, theme toggles, and feedback controls, so that I can accept or critique an agent output directly in the workspace.

**Acceptance Criteria:**

**Given** planning is complete
**When** I approve or reject the plan
**Then** the control is visible and records the action

**Given** feedback is submitted
**When** I enter a thumbs up or down with comments and topic tags
**Then** the feedback persists against the output

**Given** I switch themes
**When** the UI toggles
**Then** light and dark modes remain legible and consistent

## Epic 8: Generate and certify benchmarks
Users can author benchmark problems, validate geometry and randomization, and certify solved benchmark packages for engineering intake.

### Story 8.1: Author and submit benchmark planner handoffs
As a Benchmark Planner, I want to produce the required benchmark handoff package, so that the benchmark can be reviewed and implemented from a valid, versioned contract.

**Acceptance Criteria:**

**Given** the benchmark inputs are known
**When** the planner writes the handoff package
**Then** the required files exist, including a schema-valid `benchmark_assembly_definition.yaml` full `AssemblyDefinition`, and include estimate fields, objective zones, and mandatory material IDs

**Given** `submit_plan()` succeeds
**When** the benchmark handoff is accepted
**Then** `.manifests/benchmark_plan_review_manifest.json` is created and the benchmark plan reviewer is unblocked

### Story 8.2: Review benchmark geometry and moving fixture behavior
As a Benchmark Plan Reviewer, I want to inspect the benchmark setup and latest validation evidence, so that I can reject ambiguous or unsafe benchmark contracts before implementation.

**Acceptance Criteria:**

**Given** benchmark-owned fixtures, objective zones, and a latest-revision `.manifests/benchmark_plan_review_manifest.json` exist
**When** the reviewer checks the handoff
**Then** intersections, missing motion-visible facts, or unjustified DOFs are rejected and the stage-specific decision/comments YAML pair is written

**Given** benchmark-owned fixtures, motion metadata, or benchmark-owned electronics are present
**When** the reviewer inspects the plan
**Then** the benchmark-owned inputs remain read-only and are not treated as engineer-owned outputs

**Given** render images exist for the current revision
**When** review runs
**Then** the reviewer inspects the latest images with `inspect_media(...)` before approval

### Story 8.3: Implement benchmarks and generate preview artifacts
As a Benchmark Coder, I want to implement the approved benchmark and generate preview artifacts, so that the benchmark is ready for deterministic review and downstream engineering intake.

**Acceptance Criteria:**

**Given** an approved plan is feasible
**When** the benchmark is implemented
**Then** the geometry is valid and preview artifacts include RGB, depth, segmentation, and render manifest entries as configured

**Given** the approved plan is infeasible
**When** the benchmark coder refuses it
**Then** `plan_refusal.md` records role-specific reasons and proof, and no implementation proceeds

**Given** validation runs
**When** static preview is produced
**Then** the validation preview uses MuJoCo by default and does not mutate the simulation backend choice

### Story 8.4: Review benchmark execution and hand off to engineering
As a Benchmark Reviewer, I want to certify the latest implementation against current simulation evidence, so that downstream engineering always starts from a verified target.

**Acceptance Criteria:**

**Given** a benchmark with simulation or video evidence and a current `.manifests/benchmark_review_manifest.json`
**When** the reviewer checks it
**Then** the reviewer inspects the latest dynamic evidence and rejects stale or missing revision artifacts

**Given** the benchmark passes geometry and solvability checks
**When** review completes
**Then** the execution review decision/comments YAML pair is written and the handoff is accepted

## Epic 9: Solve electromechanical wiring and circuit problems
Users can specify power, circuits, wires, and physical wire routing, and reject invalid electrical designs before simulation.

### Story 9.1: Validate circuits before simulation
As a human engineer, I want my `Engineering Coder` LLM agent to run circuit validation before physics, so that invalid electrical designs fail before expensive simulation starts.

**Acceptance Criteria:**

**Given** a circuit has a short, open node, or overcurrent condition
**When** validation runs
**Then** the design is rejected before simulation with the appropriate explicit failure code (`FAILED_SHORT_CIRCUIT`, `FAILED_OPEN_CIRCUIT`, `FAILED_OVERCURRENT_SUPPLY`, or `FAILED_OVERCURRENT_WIRE`)

**Given** a circuit is valid
**When** validation completes
**Then** the PSU rating and wire current limits are respected

### Story 9.2: Gate motor power through the circuit
As a human engineer, I want my `Engineering Coder` LLM agent to make motor output depend on the electrical supply, so that powered and unpowered states behave realistically in simulation.

**Acceptance Criteria:**

**Given** a motor controller is valid but power is absent
**When** the simulation runs
**Then** the motor produces zero effective torque

**Given** the motor is powered within circuit limits
**When** the simulation runs
**Then** the motor produces the expected torque response

### Story 9.3: Route wires with gauge and tear behavior
As a human engineer, I want my `Engineering Coder` LLM agent to respect clearance, length, and gauge limits in wire routing, so that the design fails when wires are overstressed instead of pretending to work.

**Acceptance Criteria:**

**Given** a wire route violates gauge, clearance, or length constraints
**When** validation or simulation runs
**Then** the design is rejected or fails with a wire-specific reason

**Given** tension exceeds the rated threshold
**When** the simulation runs
**Then** the wire tears, power is removed from the affected path, and the failure is reported as `FAILED_WIRE_TORN`

## Epic 10: Solve powered electromechanical mechanisms
Users can plan and optimize powered mechanisms with electrical planning, specialist review, and unified implementation.

### Story 10.1: Plan powered mechanisms with electrical intent
As a human engineer, I want my `Electronics Planner` LLM agent to define power supplies, wiring constraints, and component choices for powered mechanisms, so that the downstream `Engineering Coder` LLM agent receives a complete electromechanical handoff.

**Acceptance Criteria:**

**Given** a powered mechanism is required
**When** the electronics planner writes the handoff
**Then** the plan includes power supply availability, wiring constraints, and explicit electrical requirements in `assembly_definition.yaml.electronics`

**Given** `submit_plan()` runs for electronics planning
**When** the handoff is accepted
**Then** the planner transitions to PLANNED and `.manifests/engineering_plan_review_manifest.json` is created for the next gate

### Story 10.2: Implement unified electromechanical solutions
As a human engineer, I want my `Engineering Coder` LLM agent to implement mechanics and electronics in one revision, so that powered assemblies stay coherent across wiring, motion, and geometry.

**Acceptance Criteria:**

**Given** an approved electromechanical plan
**When** the unified implementation is complete
**Then** one coder revision contains both mechanical and electrical changes

**Given** electronics-specific issues are found
**When** the `Electronics Reviewer` checks the revision
**Then** routing returns to the same coder rather than a separate implementation pass

**Given** the unified revision passes validation, simulation, and specialist review
**When** execution review runs
**Then** the reviewer can approve the same revision without extra implementation churn

### Story 10.3: Support controllable actuator motion modes
As a human engineer, I want my `Engineering Coder` LLM agent to support the approved actuator motion profiles, so that motors and powered fixtures can be driven with the intended behavior.

**Acceptance Criteria:**

**Given** a motor or actuator is configured
**When** the controller is applied
**Then** supported modes such as constant, sinusoidal, square, trapezoidal, or position-based control run without schema or runtime failure

**Given** the controller is malformed or unsupported
**When** the runtime validates it
**Then** the configuration fails closed before simulation proceeds

**Given** the actuator is commanded beyond its force range or sustained overload limit
**When** the simulation runs
**Then** the failure is classified as `motor_overload` instead of being silently clamped away

## Epic 11: Model fluids and fluid-electronics coupling
Users can define fluid tasks, run them on Genesis, and treat fluid exposure to electronics as a hard failure mode.

### Story 11.1: Author fluid benchmarks with backend selection
As a Benchmark Planner, I want to define fluid tasks and backend expectations explicitly, so that Genesis-backed fluid runs are planned and reviewed consistently.

**Acceptance Criteria:**

**Given** a fluid benchmark is authored
**When** the benchmark definition is saved
**Then** it includes the fluid objects, objectives, and the backend choice needed for the run

**Given** preview and final simulation use different backends
**When** the benchmark is reviewed
**Then** MuJoCo preview and Genesis validation remain separate contracts instead of being conflated

**Given** the benchmark uses a non-fluid backend
**When** the run is validated
**Then** fluid-specific objectives are not silently treated as supported

### Story 11.2: Evaluate fluid objectives and failure states
As a human engineer, I want my `Engineering Coder` LLM agent to score fluid containment and flow objectives in simulation, so that the run fails when the fluid behavior is wrong.

**Acceptance Criteria:**

**Given** a fluid containment objective exists
**When** the simulation completes
**Then** the result reports pass/fail based on the configured threshold fraction of particles in the zone and uses `FLUID_OBJECTIVE_FAILED` when it misses

**Given** a flow-rate objective exists
**When** the simulation completes
**Then** the measured rate is compared to the target and failures are reported explicitly as `FLUID_OBJECTIVE_FAILED` instead of being treated as a generic simulation miss

### Story 11.3: Treat fluid contact with powered electronics as a failure
As a human engineer, I want my `Engineering Execution Reviewer` LLM agent to fail the run when fluid contacts powered electrical components, so that electromechanical designs remain realistic and safe.

**Acceptance Criteria:**

**Given** fluids are enabled and powered electronics exist
**When** fluid contact reaches the electrical components
**Then** the simulation reports the electrical failure path and the benchmark fails

**Given** the fluid benchmark has no powered electronics
**When** the simulation runs
**Then** the fluid evaluation proceeds without inventing an electrical failure

## Epic 12: Validate deformables, stress, and breakage
Users can reason about FEM-enabled materials, stress summaries, and breakage outcomes for structural tasks.

### Story 12.1: Validate FEM-ready materials and meshes
As a human engineer, I want my `Engineering Coder` LLM agent to validate FEM-enabled parts for material properties and meshes, so that deformable simulation starts from valid inputs.

**Acceptance Criteria:**

**Given** `fem_enabled` is true
**When** a manufactured part lacks required FEM fields
**Then** validation rejects it before simulation

**Given** a deformable geometry is valid
**When** meshing runs
**Then** a tetrahedralized mesh is produced or repair fails closed with an explicit meshing error or `FAILED_ASSET_GENERATION`

### Story 12.2: Report stress summaries and breakage
As a human engineer, I want my `Engineering Execution Reviewer` LLM agent to expose stress outputs and breakage reasons in the simulation result, so that structural failures are explainable.

**Acceptance Criteria:**

**Given** a Genesis/FEM simulation completes
**When** I inspect the result
**Then** per-part stress summaries include max von Mises, safety factor, and utilization

**Given** a part exceeds ultimate stress
**When** the simulation runs
**Then** the run aborts or flags `PART_BREAKAGE` with the part label, stress value, and location

### Story 12.3: Surface deformable evidence for review
As a human engineer, I want my `Engineering Execution Reviewer` LLM agent to make deformable results and stress objectives visible in the evidence chain, so that the final decision reflects actual structural behavior.

**Acceptance Criteria:**

**Given** stress objectives are defined
**When** the simulation result is persisted
**Then** the pass/fail outcome is carried into the reviewer decision path

**Given** render or heatmap evidence exists for the current revision
**When** review runs
**Then** the reviewer inspects the evidence through `inspect_media(...)` before approval

## Epic 13: Generate, clean, and recycle dataset-ready artifacts
Researchers and companies can produce training- and RL-ready data from completed runs and filter polluted or underrepresented data.

### Story 13.1: Persist dataset-ready artifact bundles
As a researcher or company building LLMs for mechanical and electronics engineering, I want the dataset produced by this software to be directly usable for training, so that I do not need a custom conversion pipeline.

**Acceptance Criteria:**

**Given** a benchmark or engineer run completes
**When** artifacts are archived
**Then** inputs, outputs, traces, renders, `journal.md`, review evidence, and revision metadata are captured

**Given** the row is replayed later
**When** I inspect the stored metadata
**Then** the run can be restored without guessing missing files, including refusal artifacts when they exist

### Story 13.2: Exclude polluted and invalid runs
As a human dataset cleaner, I want to exclude integration tests, failed runs, and known-corrupted time windows, so that training data stays high quality.

**Acceptance Criteria:**

**Given** a run is an integration test or falls before `2026-03-03 00:00` Europe/Dublin, or otherwise lands in a known-corrupted time range
**When** dataset generation runs
**Then** the row is excluded

**Given** a run failed because of environment or LLM failure
**When** cleaning runs
**Then** it is not promoted into the complete-pass dataset

### Story 13.3: Prioritize underrepresented seeds
As a researcher or company building engineering-capable LLMs, I want the generator to prioritize underrepresented seeds and preserve lineage, so that I can use this software to create RL environments or augment my training data without losing provenance.

**Acceptance Criteria:**

**Given** a seed is already overrepresented
**When** the next batch is chosen
**Then** the generator prefers rarer seeds instead of iterating blindly

**Given** a row is created
**When** metadata is stored
**Then** it records the originating seed, generation kind, and parent lineage information

**Given** the exact same starter artifact recurs across rows
**When** the bundle is materialized
**Then** shared boilerplate is reused from `shared/agent_templates/common/` instead of being duplicated in the row bundle
