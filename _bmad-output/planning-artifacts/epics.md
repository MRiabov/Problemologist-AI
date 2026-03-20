---
stepsCompleted:
  - step-01-validate-prerequisites
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
Users can produce training and evaluation data from completed runs and filter polluted or underrepresented data.
**FRs covered:** FR37

<!-- Repeat for each epic in epics_list (N = 1, 2, 3...) -->

## Epic {{N}}: {{epic_title_N}}

{{epic_goal_N}}

<!-- Repeat for each story (M = 1, 2, 3...) within epic N -->

### Story {{N}}.{{M}}: {{story_title_N_M}}

As a {{user_type}},
I want {{capability}},
So that {{value_benefit}}.

**Acceptance Criteria:**

<!-- for each AC on this story -->

**Given** {{precondition}}
**When** {{action}}
**Then** {{expected_outcome}}
**And** {{additional_criteria}}

<!-- End story repeat -->
