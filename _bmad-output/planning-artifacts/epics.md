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
  - specs/architecture/electronics-and-electromechanics.md
  - specs/architecture/fluids-and-deformables.md
  - specs/architecture/observability.md
  - specs/business-usecase.md
  - specs/dataset-generation.md
  - specs/frontend-specs.md
  - specs/integration-tests.md
  - specs/todos.md
  - docs/architecture.md
  - docs/project-overview.md
  - docs/spec-coverage.md
---

# Problemologist-AI - Epic Breakdown

## Overview

This document provides the complete epic and story breakdown for Problemologist-AI, decomposing the requirements from the architecture, business, frontend, and integration specs into implementable stories.

## Requirements Inventory

### Functional Requirements

- FR1: The platform shall support agent action execution and persistence.
  - Agent actions execute in a session-scoped runtime substrate.
  - Generated files, traces, and state persist across tool calls and later turns.
  - Later turns and handoffs resume from the persisted session state.
  - The benchmark and engineering workflow specifics that used to sit under this umbrella are covered by FR2-FR10.
- FR2: The platform shall support a benchmark planner role that can author `plan.md`, `todo.md`, `benchmark_definition.yaml`, and `benchmark_assembly_definition.yaml`, then submit the plan through `submit_plan()`.
- FR3: The platform shall support a benchmark plan reviewer that validates plan consistency, benchmark solvability, moving-fixture visibility, and persists decision/comments YAML plus the stage manifest.
- FR4: The platform shall support a benchmark coder that implements approved benchmarks and can refuse infeasible plans through `plan_refusal.md`.
- FR5: The platform shall support a benchmark reviewer that validates implemented benchmark geometry, render evidence, simulation evidence, and hands the benchmark off to engineering.
- FR6: The platform shall support an engineering planner that authors `plan.md`, `todo.md`, `benchmark_definition.yaml`, and `assembly_definition.yaml` for a solution proposal.
- FR7: The platform shall support an electronics planner that augments the engineering plan with electrical requirements, component choices, and wiring intent when powered mechanisms are needed.
- FR8: The platform shall support an engineering plan reviewer that validates the combined mechanical/electrical plan before coding, including budget realism and DOF minimization.
- FR9: The platform shall support a unified engineering coder that implements mechanical and electrical solution logic in one revision and can refuse invalid plans through `plan_refusal.md`.
- FR10: The platform shall support an electronics reviewer and an engineering execution reviewer that validate the unified implementation after validation and simulation succeed.
- FR11: The platform shall enforce reviewer-specific stage manifests and reviewer decision/comments YAML pairs for every review stage.
- FR12: The platform shall enforce read-only benchmark-owned fixtures, benchmark motion metadata, and benchmark drill/attachment permissions across handoff and coding stages.
- FR13: The platform shall allow COTS search through a read-only subagent with reproducible catalog results and no unintended workspace mutation.
- FR14: The platform shall expose agent-facing utilities for validation, simulation, review submission, pricing, rendering, and documentation lookup through the canonical `utils` package.
- FR15: The platform shall enforce workspace-relative filesystem permissions with read/write isolation, read-only mounts, and `.manifests/**` denial for agent roles.
- FR16: The platform shall keep controller, worker-light, worker-heavy, and controller-worker responsibilities split so heavy simulation and validation run outside the controller process.
- FR17: The platform shall provide single-flight heavy-worker admission with deterministic busy responses and readiness changes while jobs are active.
- FR18: The platform shall route heavy operations through Temporal workflows for durable execution and retry handling.
- FR19: The platform shall persist important files and generated assets to the worker filesystem and S3-backed storage with session-scoped isolation.
- FR20: The platform shall generate static validation preview renders as a 24-view package and persist render manifests, including RGB, depth, and segmentation siblings when enabled.
- FR21: The platform shall support MuJoCo-backed static validation preview by default, even when the simulation backend is Genesis, while keeping simulation backend selection explicit.
- FR22: The platform shall support simulation on the selected physics backend and keep validation preview distinct from simulation parity.
- FR23: The platform shall validate benchmark geometry against build, goal, and forbid zones, simulation bounds, and runtime randomization ranges before submission or review.
- FR24: The platform shall support benchmark-owned fixture metadata, including `benchmark_parts`, `fixed`, `cots_id`, `material_id`, attachment policy, and drill policy.
- FR25: The platform shall require move-object and objective metadata, including mandatory material IDs that resolve to known materials.
- FR26: The platform shall map build123d joints to simulator joints and constraints and validate realistic fastener-based rigid connections.
- FR27: The platform shall support motor and actuator controller functions including constant, sinusoidal, square, trapezoidal, and position-based control.
- FR28: The platform shall enforce manufacturability validation and pricing across supported workbenches and materials, and reject unsupported or invalid parts.
- FR29: The platform shall account for benchmark drilling costs when environment drill operations are declared.
- FR30: The platform shall evaluate simulation success and failure for goal hit, forbid hit, out-of-bounds, timeout, instability, overload, and breakage conditions.
- FR31: The platform shall support runtime randomization and batch verification over multiple jittered scenes in one admitted heavy-worker job.
- FR32: The platform shall support electromechanical benchmark and solution features including power supplies, circuits, wires, power-gated actuation, and wire-routing clearance and tear behavior.
- FR33: The platform shall reject invalid circuits, open circuits, short circuits, overcurrent conditions, and wire-routing violations before expensive simulation proceeds.
- FR34: The platform shall support fluid and deformable-material benchmarks, including Genesis backend selection, fluid definitions, fluid objectives, stress objectives, and breakage handling.
- FR35: The platform shall emit reasoning, tool-call, simulation, review, and lineage traces incrementally and make them available for frontend, review, and training-data use.
- FR36: The platform shall persist observability metadata including `user_session_id`, `episode_id`, simulation/review/trace IDs, seeds, and related lineage fields.
- FR37: The platform shall support dataset generation and cleaning by persisting all workflow inputs and outputs, prioritizing underrepresented seeds, and excluding invalid or polluted data.
- FR38: The platform shall provide a user-facing frontend that supports benchmark and engineering sessions, session history, chat, CAD viewer, code viewer, plan approval, and interrupt and steer actions.
- FR39: The platform shall surface user feedback collection on model outputs, including thumbs up/down, editable comments, and issue topics.
- FR40: The platform shall support evaluation gates and fail-closed terminal states so invalid artifacts or missing evidence never progress as success.

### NonFunctional Requirements

- NFR1: All planner and reviewer machine-readable outputs shall be strict-schema artifacts and shall reject unknown or extra fields recursively.
- NFR2: Reasoning and tool traces shall be emitted incrementally and the system shall fail closed when required traces are absent in modes that require them.
- NFR3: Visual inspection shall be required only when render images exist, and the dedicated `inspect_media(...)` tool shall be used; text-only file listing shall not count.
- NFR4: The heavy-worker service shall remain single-flight with deterministic busy responses and shall not implement an internal multi-job queue.
- NFR5: Validation preview shall remain MuJoCo-based by default and shall not be treated as Genesis parity evidence.
- NFR6: Heavy simulation and validation shall remain crash-contained behind isolated worker process boundaries rather than running LLM-generated code on the controller.
- NFR7: All critical APIs shall remain schema-valid and compatible with strict OpenAPI and schemathesis-style checks.
- NFR8: Observability shall capture all required IDs, seeds, events, and error streams so runs are queryable and attributable.
- NFR9: Episode terminal states shall always include explicit failure classification when the episode fails.
- NFR10: All run provenance relevant to evaluation shall remain reproducible across seeds, variants, and backend selection.
- NFR11: The UI shall stream updates in near real time and shall not wait for workflow completion to show traces or tool activity.
- NFR12: The system shall keep compatibility paths explicit and shall not hide fallback or degraded behavior as ordinary success.
- NFR13: The filesystem policy shall prevent agent writes to forbidden mounts and to `.manifests/**`.
- NFR14: Integration verification shall be HTTP-only against the running compose stack rather than unit-style internal invocation.
- NFR15: Artifact persistence shall remain traceable through DB, S3, and review files without losing session/episode linkage.

### Additional Requirements

- The benchmark planner handoff shall include a `plan.md`, `todo.md`, `benchmark_definition.yaml`, and `benchmark_assembly_definition.yaml` package, and planner submission shall persist `.manifests/benchmark_plan_review_manifest.json`.
- The engineering planner handoff shall include `plan.md`, `todo.md`, `benchmark_definition.yaml`, and `assembly_definition.yaml`, and planner submission shall persist `.manifests/engineering_plan_review_manifest.json`.
- Reviewer output shall always be persisted as stage-specific YAML decision/comments pairs under `reviews/`, and routing shall depend only on the decision YAML.
- `benchmark_definition.yaml` shall carry objective zones, randomization, and planner-authored estimate fields that derive `max_unit_cost` and `max_weight_g`.
- `benchmark_definition.yaml` shall include `moved_object.material_id`, and the value shall resolve to a known material in `manufacturing_config.yaml`.
- `benchmark_assembly_definition.yaml` shall always be a schema-valid full `AssemblyDefinition`, even when the benchmark uses a minimal fixture declaration.
- `benchmark_definition.yaml.benchmark_parts` shall represent benchmark-owned fixtures, including attachment policy and drill policy, and those fixtures shall remain read-only for engineering.
- `assembly_definition.yaml.environment_drill_operations` shall validate against benchmark-side drill policy and shall add benchmark drilling cost when present.
- `assembly_definition.yaml.electronics` shall hold the engineer-owned electrical design, while `benchmark_definition.yaml.electronics_requirements` shall hold benchmark-owned electrical constraints.
- Physical wire routes shall be waypoint-based, gauge-aware, length-aware, and clearance-validated, with wire tear reducing power to the affected path.
- Static preview renders shall produce a `renders/render_manifest.json` file with per-image modality metadata and segmentation legends that distinguish repeated instances.
- Render modality emission shall be controlled by `config/agents_config.yaml`, and the current policy shall keep RGB, depth, and segmentation enabled unless configuration says otherwise.
- The worker filesystem shall keep `/utils`, `/skills`, `/reviews`, and `/config` read-only while leaving the workspace root writable.
- Heavy operations shall use Temporal as the durable execution boundary, and direct `worker-heavy` HTTP endpoints shall remain integration-test/debug only.
- `inspect_media(...)` shall be the only agent-facing way to inspect render images or video frames, and review approval shall remain invalid without required inspection when renders exist.
- The project shall keep benchmark-side moving fixtures and benchmark-owned electronics as read-only environment behavior, not engineer-owned implementation targets.
- The dataset generation pipeline shall persist all useful workflow artifacts and metadata into dataset rows and shall prioritize underrepresented seeds instead of iterating blindly.
- Deferred backlog from `specs/todos.md`: integration mock responses shall eventually migrate from per-node fixtures to transcript-based scenarios with ordered tool/observation turns.
- Deferred backlog from `specs/todos.md`: agent tool transport shall eventually support a WebSocket protocol with strict correlation IDs while preserving HTTP compatibility during migration.
- Deferred backlog from `specs/todos.md`: fallback or degraded behavior shall be explicit and machine-readable rather than hidden behind ordinary success responses.
- Evaluation infrastructure shall own seeded preflight, cross-contract semantic checks, and HTTP-only integration verification; callers must not duplicate those validators locally.
- The system shall continue to treat skills as runtime-mounted artifacts, with skill and docs lookup available to agents through the documented tool surface.

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

FR1: Epic 1 - Execute agent actions in a session-scoped runtime and persist their state
FR2: Epic 7 - Author benchmark planner handoff artifacts and submit the benchmark plan
FR3: Epic 7 - Review benchmark plans for consistency, solvability, and motion visibility
FR4: Epic 7 - Implement approved benchmark plans and refuse infeasible ones
FR5: Epic 7 - Review implemented benchmarks and hand off solved benchmarks to engineering
FR6: Epic 3 - Author engineering planner handoff artifacts for a solution proposal
FR7: Epic 9 - Add electrical planning, component choices, and wiring intent for powered mechanisms
FR8: Epic 3 - Review the combined mechanical plan before coding
FR9: Epic 3 - Implement the unified mechanical solution and refuse invalid plans
FR10: Epic 9 - Review unified electromechanical implementations after validation and simulation
FR11: Epic 3 - Persist stage-specific review manifests and decision/comments YAML pairs
FR12: Epic 7 - Keep benchmark-owned fixtures, motion metadata, and drill permissions read-only for engineering
FR13: Epic 4 - Search the COTS catalog without mutating workspace state
FR14: Epic 3 - Expose canonical agent-facing utilities for validation, simulation, pricing, review, rendering, and docs lookup
FR15: Epic 1 - Enforce workspace-relative filesystem permissions and read-only mounts
FR16: Epic 1 - Keep controller, worker-light, worker-heavy, and controller-worker responsibilities split
FR17: Epic 1 - Enforce single-flight heavy-worker admission and deterministic busy responses
FR18: Epic 1 - Route heavy operations through Temporal for durable execution and retry handling
FR19: Epic 1 - Persist important files and generated assets with session-scoped isolation
FR20: Epic 7 - Generate the 24-view static validation preview package and render manifests
FR21: Epic 7 - Use MuJoCo-backed validation preview by default while keeping simulation backend selection explicit
FR22: Epic 1 - Keep validation preview distinct from actual simulation parity
FR23: Epic 7 - Validate benchmark geometry, zones, bounds, and runtime randomization before submission or review
FR24: Epic 7 - Support benchmark-owned fixture metadata and attachment/drill policy
FR25: Epic 7 - Require move-object and objective metadata, including mandatory material IDs
FR26: Epic 4 - Map build123d joints to simulator joints and validate realistic rigid connections
FR27: Epic 9 - Support motor and actuator controller functions for benchmark and solution flow
FR28: Epic 4 - Enforce manufacturability validation and pricing across supported workbenches
FR29: Epic 4 - Account for benchmark drilling costs when environment drilling is declared
FR30: Epic 1 - Classify simulation success and failure outcomes consistently
FR31: Epic 1 - Run batch verification over multiple jittered scenes in one admitted heavy-worker job
FR32: Epic 8 - Support power supplies, circuits, wires, and power-gated actuation
FR33: Epic 8 - Reject invalid circuits and wire-routing violations before simulation
FR34: Epic 10 - Support fluid and deformable-material benchmarks with Genesis behavior
FR35: Epic 2 - Emit reasoning, tool-call, simulation, review, and lineage traces incrementally
FR36: Epic 2 - Persist observability metadata, session IDs, episode IDs, and seed lineage
FR37: Epic 12 - Generate dataset-ready outputs and clean invalid or polluted data
FR38: Epic 6 - Provide the interactive benchmark/engineering workspace with chat, CAD, and code views
FR39: Epic 6 - Collect thumbs up/down feedback with comments and topic tags
FR40: Epic 3 - Enforce fail-closed terminal states so invalid artifacts never progress as success

## Epic List

### Epic 1: Provide agent runtime and persistence
Users can execute agent actions in a session-scoped workspace with durable persistence, isolated execution, and fail-closed runtime gates.
**FRs covered:** FR1, FR15, FR16, FR17, FR18, FR19, FR22, FR30, FR31

### Epic 2: Capture observability, lineage, and review evidence
Users can reconstruct what happened, where, and why through complete traces, IDs, metrics, and logs.
**FRs covered:** FR35, FR36

### Epic 3: Build evaluation and verification infrastructure
Users can validate handoff artifacts, enforce strict schema and cross-contract checks, and keep seeded and integration verification fail-closed.
**FRs covered:** FR40
**Relevant NFRs/additional requirements:** NFR1, NFR2, NFR3, NFR7, NFR9, NFR14, NFR15, seeded-eval preflight contract, integration-test HTTP-only boundaries

### Epic 4: Run planner, reviewer, coder, and refusal workflows
Users can move benchmark and engineering work through the correct handoff gates with the right files, manifests, and submission controls.
**FRs covered:** FR6, FR8, FR9, FR11, FR14

### Epic 5: Design manufacturable CAD solutions with validated materials and COTS
Users can build priced, manufacturable mechanical solutions from real materials, joints, workbenches, and catalog parts.
**FRs covered:** FR13, FR26, FR28, FR29

### Epic 6: Steer agents with precise context and prompt control
Users can point to exact parts, lines, and code context so the agent receives the right local evidence and prompt hints.
**Relevant UX/additional requirements:** UX-DR8, UX-DR14, exact-point selection, line-targeted steering, `@` mentions

### Epic 7: Provide the interactive workspace and feedback UI
Users can monitor sessions, inspect CAD and code, interrupt runs, and rate outputs in one shared interface.
**FRs covered:** FR38, FR39

### Epic 8: Generate and certify benchmarks
Users can author benchmark problems, validate geometry and randomization, and certify solved benchmark packages for engineering intake.
**FRs covered:** FR2, FR3, FR4, FR5, FR12, FR20, FR21, FR23, FR24, FR25

### Epic 9: Solve electromechanical wiring and circuit problems
Users can specify power, circuits, wires, and physical wire routing, and reject invalid electrical designs before simulation.
**FRs covered:** FR32, FR33

### Epic 10: Solve powered electromechanical mechanisms
Users can plan and optimize powered mechanisms with electrical planning, specialist review, and unified implementation.
**FRs covered:** FR7, FR10, FR27

### Epic 11: Model fluids and fluid-electronics coupling
Users can define fluid tasks, run them on Genesis, and treat fluid exposure to electronics as a hard failure mode.
**FRs covered:** FR34

### Epic 12: Validate deformables, stress, and breakage
Users can reason about FEM-enabled materials, stress summaries, and breakage outcomes for structural tasks.
**Relevant additional requirements:** `physics.fem_enabled`, stress objectives, breakage handling, stress summaries, stress heatmaps

### Epic 13: Generate, clean, and recycle dataset-ready artifacts
Researchers and companies can produce training- and RL-ready data from completed runs and filter polluted or underrepresented data.
**FRs covered:** FR37

## Epic 1: Provide agent runtime and persistence
Users can execute agent actions in a session-scoped workspace with durable persistence, isolated execution, and fail-closed runtime gates.

### Story 1.1: Isolated session workspaces and durable artifacts
As a human engineer, I want my controller-managed LLM agent to use a separate workspace and trace history for each episode, so that files, assets, and reasoning never leak across runs.

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
As a human engineer, I want my `Engineering Coder` LLM agent to report distinct validation and simulation terminal states, so that I can trust robustness checks across jittered scenes.

**Acceptance Criteria:**

**Given** a solution is validated
**When** preview renders are generated
**Then** the static preview path uses the validation backend and does not mutate simulation backend selection

**Given** one admitted job runs multiple jittered scenes
**When** simulation completes
**Then** pass/fail statistics are aggregated and outcomes are classified as goal-hit, forbid-hit, out-of-bounds, timeout, or instability

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
As a human engineer, I want my `Engineering Planner` LLM agent to create the required handoff files and submit them through the planner gate, so that the next stage receives a validated plan.

**Acceptance Criteria:**

**Given** `plan.md`, `todo.md`, `benchmark_definition.yaml`, and `assembly_definition.yaml` are present and valid
**When** `submit_plan()` runs
**Then** the planner transitions to PLANNED and `.manifests/engineering_plan_review_manifest.json` is created

**Given** any required engineering planner file is missing or invalid
**When** `submit_plan()` runs
**Then** handoff fails closed and no success-like state is emitted

### Story 4.3: Review engineering plans and refusal evidence
As a human engineer, I want my `Engineering Plan Reviewer` LLM agent to accept or refuse a plan with stage-specific YAML outputs, so that routing is deterministic and explainable.

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
As a human engineer, I want my `Engineering Coder` LLM agent to implement approved plans or refuse infeasible ones with evidence, so that only viable plans proceed to simulation.

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

## Epic 5: Design manufacturable CAD solutions with validated materials and COTS
Users can build priced, manufacturable mechanical solutions from real materials, joints, workbenches, and catalog parts.

### Story 5.1: Search COTS parts with reproducible metadata
As a human engineer, I want my `Benchmark Planner` or `Engineering Planner` LLM agent to search the COTS catalog and carry selected part metadata into planning artifacts, so that component choices are real and reproducible.

**Acceptance Criteria:**

**Given** a catalog query is issued
**When** COTS search runs
**Then** returned candidates include part identity, manufacturer, specs, price, and source or a no-match rationale, and the search does not mutate workspace state beyond allowed journal logging

**Given** a COTS part is selected
**When** the plan is saved
**Then** the part ID and catalog metadata are persisted into the plan and cost artifacts

### Story 5.2: Price manufacturable parts by workbench and material
As a human engineer, I want my `Engineering Planner` LLM agent to use the selected manufacturing method, material, and drill operations for pricing and manufacturability checks, so that budget estimates match build reality.

**Acceptance Criteria:**

**Given** a part uses a supported method and material
**When** `validate_and_price` runs
**Then** it returns cost and weight data or rejects the part if the combination is invalid

**Given** environment drilling is declared
**When** pricing runs
**Then** drilling cost is included and the handoff fails closed if the limits are exceeded

**Given** planner-owned max cost and weight are derived
**When** the plan is validated
**Then** they remain under the benchmark/customer caps

### Story 5.3: Map joints and fasteners to simulator constraints
As a human engineer, I want my `Engineering Coder` LLM agent to map build123d joints and fasteners to simulator constraints, so that the solution is physically realizable instead of symbolic only.

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
**Then** session history, chat, and right-side artifacts appear in the same shell layout

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
As a human mechanical engineer, I want the benchmark flow to output validated designs using proven scientific simulation methods, so that I can trust the designs it hands me for review.

**Acceptance Criteria:**

**Given** the benchmark inputs are known
**When** the planner writes the handoff package
**Then** the required files exist, including a schema-valid `benchmark_assembly_definition.yaml` full `AssemblyDefinition`, and include estimate fields, objective zones, and mandatory material IDs

**Given** `submit_plan()` succeeds
**When** the benchmark handoff is accepted
**Then** `.manifests/benchmark_plan_review_manifest.json` is created and the benchmark plan reviewer is unblocked

### Story 8.2: Review benchmark geometry and moving fixture behavior
As a human mechanical engineer, I want to inspect the benchmark setup and the simulation evidence used to validate my design, so that I can understand how the system reached its result.

**Acceptance Criteria:**

**Given** benchmark-owned fixtures and objective zones exist
**When** the reviewer checks the handoff
**Then** intersections, missing motion-visible facts, or unjustified DOFs are rejected

**Given** render images exist for the current revision
**When** review runs
**Then** the reviewer inspects the latest images with the dedicated media tool before approval

### Story 8.3: Implement benchmarks and generate preview artifacts
As a human mechanical engineer, I want to see how my product behaves in the simulation that represents the real environment, so that I can judge whether the design will work before fabrication.

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
As a human mechanical engineer, I want the benchmark reviewer to certify the latest implementation against current simulation evidence, so that downstream engineering always starts from a verified target.

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
**Then** the design is rejected before simulation

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
**Then** the wire tears, power is removed from the affected path, and the failure is reported

## Epic 10: Solve powered electromechanical mechanisms
Users can plan and optimize powered mechanisms with electrical planning, specialist review, and unified implementation.

### Story 10.1: Plan powered mechanisms with electrical intent
As a human engineer, I want my `Electronics Planner` LLM agent to define power supplies, wiring constraints, and component choices for powered mechanisms, so that the downstream `Engineering Coder` LLM agent receives a complete electromechanical handoff.

**Acceptance Criteria:**

**Given** a powered mechanism is required
**When** the electronics planner writes the handoff
**Then** the plan includes power supply availability, wiring constraints, and explicit electrical requirements

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
**When** the specialist reviewer checks the revision
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

## Epic 11: Model fluids and fluid-electronics coupling
Users can define fluid tasks, run them on Genesis, and treat fluid exposure to electronics as a hard failure mode.

### Story 11.1: Author fluid benchmarks with backend selection
As a human mechanical engineer, I want my `Benchmark Planner` LLM agent to define fluid tasks and backend expectations explicitly, so that Genesis-backed fluid runs are planned and reviewed consistently.

**Acceptance Criteria:**

**Given** a fluid benchmark is authored
**When** the benchmark definition is saved
**Then** it includes the fluid objects, objectives, and the backend choice needed for the run

**Given** the benchmark uses a non-fluid backend
**When** the run is validated
**Then** fluid-specific objectives are not silently treated as supported

### Story 11.2: Evaluate fluid objectives and failure states
As a human engineer, I want my `Engineering Coder` LLM agent to score fluid containment and flow objectives in simulation, so that the run fails when the fluid behavior is wrong.

**Acceptance Criteria:**

**Given** a fluid containment objective exists
**When** the simulation completes
**Then** the result reports pass/fail based on the configured threshold fraction of particles in the zone

**Given** a flow-rate objective exists
**When** the simulation completes
**Then** the measured rate is compared to the target and failures are reported explicitly

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
**Then** a tetrahedralized mesh is produced or repair fails closed with an explicit meshing error

### Story 12.2: Report stress summaries and breakage
As a human engineer, I want my `Engineering Execution Reviewer` LLM agent to expose stress outputs and breakage reasons in the simulation result, so that structural failures are explainable.

**Acceptance Criteria:**

**Given** a Genesis/FEM simulation completes
**When** I inspect the result
**Then** per-part stress summaries include max von Mises, safety factor, and utilization

**Given** a part exceeds ultimate stress
**When** the simulation runs
**Then** the run aborts or flags breakage with the part label, stress value, and location

### Story 12.3: Surface deformable evidence for review
As a human engineer, I want my `Engineering Execution Reviewer` LLM agent to make deformable results and stress objectives visible in the evidence chain, so that the final decision reflects actual structural behavior.

**Acceptance Criteria:**

**Given** stress objectives are defined
**When** the simulation result is persisted
**Then** the pass/fail outcome is carried into the reviewer decision path

**Given** render or heatmap evidence exists for the current revision
**When** review runs
**Then** the reviewer inspects the evidence through the dedicated media tool before approval

## Epic 13: Generate, clean, and recycle dataset-ready artifacts
Researchers and companies can produce training- and RL-ready data from completed runs and filter polluted or underrepresented data.

### Story 13.1: Persist dataset-ready artifact bundles
As a researcher or company building LLMs for mechanical and electronics engineering, I want the dataset produced by this software to be directly usable for training, so that I do not need a custom conversion pipeline.

**Acceptance Criteria:**

**Given** a benchmark or engineer run completes
**When** artifacts are archived
**Then** inputs, outputs, traces, renders, journals, and revision metadata are captured

**Given** the row is replayed later
**When** I inspect the stored metadata
**Then** the run can be restored without guessing missing files

### Story 13.2: Exclude polluted and invalid runs
As a human dataset cleaner, I want to exclude integration tests, failed runs, and known-corrupted time windows, so that training data stays high quality.

**Acceptance Criteria:**

**Given** a run is an integration test or falls in a known-corrupted time range
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
