---
stepsCompleted:
  - step-01-init
  - step-02-discovery
  - step-02b-vision
  - step-02c-executive-summary
  - step-03-success
  - step-04-journeys
  - step-05-domain
  - step-06-innovation
  - step-07-project-type
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
  - _bmad-output/project-context.md
  - _bmad-output/planning-artifacts/epics.md
  - docs/index.md
  - docs/project-overview.md
  - docs/architecture.md
  - docs/backend-reference.md
  - docs/api-contracts.md
  - docs/data-models.md
  - docs/development-guide.md
  - docs/deployment-guide.md
  - docs/component-inventory.md
  - docs/spec-coverage.md
  - docs/source-tree-analysis.md
  - docs/project-scan-report.json
  - docs/project-parts.json
  - specs/business-usecase.md
  - specs/dataset-generation.md
  - specs/frontend-specs.md
  - specs/integration-tests.md
  - specs/todos.md
documentCounts:
  briefCount: 0
  researchCount: 0
  brainstormingCount: 0
  projectDocsCount: 35
classification:
  projectType: backend-first agentic engineering platform with a secondary React inspection dashboard
  domain: AI-assisted mechanical/electromechanical engineering, CAD, physics simulation, benchmark generation, and dataset tooling
  complexity: very high
  projectContext: brownfield
workflowType: 'prd'
---

# Product Requirements Document - Problemologist-AI

**Author:** Max
**Date:** 2026-03-20

## Executive Summary
Problemologist-AI is an open-source engineering platform for solving real engineering problems end to end, with scope very clearly bounded by the human engineer. It starts with mechanical engineering, extends into electromechanical systems, and can optionally cover fluids. The primary users are hardware engineers, product teams, and builders who need verified designs; the secondary users are LLM researchers and companies that need high-signal engineering benchmarks, solution traces, and training data. The platform addresses two gaps at once: current LLMs are not yet reliable enough for unconstrained engineering autonomy, and the field lacks open, high-quality datasets that reflect real design, validation, and review workflows.

Problemologist-AI combines solution generation with an evidence-preserving dataset loop. The solver workflow takes a bounded problem statement, decomposes it, generates candidate concepts, validates them against physics, manufacturability, cost, weight, and review constraints, and iterates until the artifact is acceptable. In parallel, the system preserves the benchmarks, reasoning traces, solved examples, reviewer feedback, and optimization notes needed to train and evaluate future engineering-capable models. The result is both a practical engineering tool and a reusable research infrastructure.

### What Makes This Special
Problemologist-AI does not treat dataset generation as a side effect. It treats solving and data production as a single closed loop. The product is differentiated by the fact that every engineering attempt can become reusable training signal: the benchmark, the solution, the failure modes, the review comments, and the optimized but unused alternatives remain available as open-source artifacts.

The core insight is that reliable engineering assistance depends on strict feedback loops, not just bigger models. Real engineering output requires physics-backed validation, manufacturability checks, and reviewer-gated iteration. Those same gates create the dataset that makes future models better. That dual use is the moat: solve the task now, and turn the work into training material for the next system.

This is not a generic agent, not a benchmark-only project, and not a design sandbox. It is a strict engineering system with open-source outputs and a dataset engine attached to it.

## Project Classification
- Project Type: backend-first agentic engineering platform with a secondary React inspection dashboard
- Domain: AI-assisted mechanical/electromechanical engineering, CAD, physics simulation, benchmark generation, and dataset tooling
- Complexity: very high
- Project Context: brownfield

## Success Criteria

### User Success
- A human engineer can define a bounded mechanical or electromechanical problem, review the plan, and receive a verified design package without manual artifact repair.
- The user can tell why a candidate solution passed or failed from the trace, review, and simulation evidence alone.
- The workflow reduces manual iteration needed to reach a reviewable solution.

### Business Success
- The platform reliably solves bounded engineering problems that are useful in real workflows, starting with mechanical tasks and extending to electromechanical tasks.
- Every completed run produces both a usable solution and a reusable benchmark/dataset artifact bundle.
- Researchers and companies can reuse traces, benchmarks, solved examples, and review outcomes without reconstruction.

### Technical Success
- Success is fail-closed: missing artifacts, invalid schemas, stale manifests, or missing review evidence never pass as clean success.
- Metrics are derived from persisted artifacts and traces, not synthetic fixtures or fallback states.
- When render evidence exists, reviewers inspect actual media before approval.
- Runs remain reproducible across seeds and runtime jitter where robustness is expected.

### Measurable Outcomes
- Fast evals: markdown validity reaches 95% on the first prompt, code validity reaches 90% on average tool calls, build123d code validity reaches 95%, and output YAML validity reaches 95%.
- Benchmark generation: benchmark planner passes reviewer scrutiny in 80% of cases; benchmark coder reaches 95% physical validity after 30 turns and three reviewer submissions.
- Engineering solving: the engineer reaches the goal within 30 tool calls and three simulation attempts, with first/second/third submission pass rates of 70%, 85%, and 95%.
- Robustness: if one simulation passes, at least 70% of runtime-jitter variations also pass.
- Manufacturability: manufacturability checks pass in 70% of validation/pricing calls and 90% of expected simulation/submission calls, improving to 95% and 97% on second and third attempts.
- Review quality: reviewers reject inconsistent, infeasible, ambiguous, or over-actuated solutions at the architecture thresholds, and canonical checklist keys/values match seeded ground truth in at least 95% of cases.
- Dataset readiness: completed episodes include all mandatory artifacts, traces, validation markers, and review evidence required for training readiness.

## Product Scope

### MVP - Minimum Viable Product
- Bounded mechanical problem solving.
- Benchmark generation, planning, review gates, simulation validation, and trace persistence.
- Open-source export of benchmarks, solved examples, and execution traces.
- Secondary inspection UI for reviewing outputs and evidence.
- Debuggability and observability are Phase 1 requirements: runtime behavior must align with real application features and documentation, feedback loops must be fast, fallback paths must be explicit and logged, terminal reasons must be deterministic, and logs/traces must make failures reproducible.

### Growth Features (Post-MVP)
- Electromechanical planning and validation.
- Jitter-randomization robustness.
- Better dataset packaging, metrics, and export paths.
- Broader benchmark coverage and richer review tooling.

### Vision (Future)
- Fluids and broader multiphysics.
- Better generalization across benchmark families.
- A reusable open-source infrastructure layer for engineering benchmark creation, solution generation, and model training.

## User Journeys

### Journey 1: Mechanical Engineer Receives a Design They Can Trust
A human mechanical engineer has a bounded physical problem: a part, mechanism, fixture, or machine that needs to work in the real world. They define the problem in terms of load, size, cost, material limits, and expected behavior. The system returns a validated design package instead of a speculative sketch.

The engineer reviews the plan, CAD, simulation evidence, and manufacturability checks as a single engineering artifact. If the design passes, the engineer can move forward without manual cleanup. If it fails, the failure is specific enough to revise the brief and try again.

This journey reveals requirements for validated design output, traceable reasoning, manufacturability checks, and clear pass/fail explanation.

### Journey 2: Mechanical Engineer Inspects Validation and Simulation
The same engineer does not trust a pass blindly. They want to inspect the validation setup, see how the benchmark was defined, and understand how the simulation behaved in the modeled environment.

This is the trust-building moment. The engineer checks the objective zones, constraints, randomization, and the actual simulation behavior to decide whether the product's answer is defensible. If the run exposes a flaw, they tighten the brief and rerun. If it holds up, they can sign off on the design with confidence.

This journey reveals requirements for inspectable validation setup, simulation playback or evidence, real-environment behavior, and a revision loop when the simulation exposes a problem.

### Journey 3: Researcher or Company Reuses the Dataset
A researcher or company building engineering-capable LLMs wants the data this software produces to be directly useful for training and evaluation. They are not looking for a one-off design result. They want complete, structured episodes with traces, decisions, solutions, and lineage metadata that can flow into a training or RL pipeline.

They export solved episodes, filter by benchmark family or seed coverage, and check that the schema is stable enough to automate ingestion. If any episode is missing artifacts or has polluted lineage, the data is not useful to them.

This journey reveals requirements for dataset export, schema stability, lineage metadata, reproducibility, and RL-ready packaging.

### Journey 4: Dataset Operator or Curator Keeps the Corpus Usable
A human dataset operator or curator watches the generated corpus over time. They review coverage across problem families, remove polluted or incomplete data, and prioritize underrepresented seeds so the dataset stays balanced and useful.

Their work is not about solving engineering tasks. It is about keeping the open-source corpus trustworthy and training-ready. They need lineaged evidence, quality checks, and cleanup controls strong enough to separate a genuinely useful episode from a superficially successful but unusable one.

This journey reveals requirements for dataset cleaning, coverage management, seed prioritization, failure classification, and operational observability.

### Journey Requirements Summary
- The primary human user is a mechanical engineer, not a benchmark designer persona.
- Benchmark setup is part of validation evidence, not a separate human role.
- The product must support trust through inspectable evidence, not just output generation.
- Dataset consumers need structured, reproducible, training-ready exports.
- Dataset operators need cleaning, coverage, and lineage controls.
- Internal agent roles are implementation details and should not be written as user journeys.

## Domain-Specific Requirements

### Compliance & Regulatory
- The product is not a certification, compliance, or sign-off system.
- Human engineer judgment remains the authority for real deployments.
- The system must preserve provenance and evidence so decisions can be audited, but it must not imply external certification-grade guarantees.

### Technical Constraints
- The primary domain is bounded mechanical engineering; electromechanical support is required when power, wiring, motors, or circuits are part of the task.
- Fluids and deformables are optional extensions and must use the correct backend and evidence path when enabled.
- Supported workbenches are finite and explicit: `CNC`, `injection molding`, and `3D printing`.
- Supported mechanisms and capabilities must stay explicit and finite; unsupported behaviors should fail closed.
- The human engineer must be able to choose simulation fidelity per task: rigid-body simulation for fast iteration, or FEM when slower but more precise structural response is required.
- Final solutions must be manufacturable at the requested production volume, not just mechanically plausible in simulation.
- Cost validation must be volume-aware: setup cost and variable cost both matter, so the preferred manufacturing method can change with quantity.
- The system must enforce cost, weight, size, and form-factor constraints together rather than treating them independently.
- At low quantities, a method with higher per-part cost but lower setup cost may be correct; at high quantities, a higher-tooling method with lower variable cost may be correct.
- Example rule: a small batch can favor CNC, while large batches can favor injection molding if the geometry and requirements fit.
- Electromechanical tasks require circuit validity before expensive physics simulation proceeds.
- Benchmark-owned fixtures and benchmark-owned electronics are read-only context unless the benchmark explicitly declares engineer interaction through its verification setup.
- Moving benchmarks require dynamic evidence for approval; static validation preview is context, not proof of motion behavior.
- Benchmark-owned fixtures may be drilled when the benchmark verification setup explicitly allows it, and those drilling operations must be priced; this is the mechanism for attaching or constraining parts to an environment, such as fixing a machine to the floor or wall.
- Fluids, deformables, and stress-aware tasks require Genesis for final validation, while MuJoCo remains the fast preview path.
- Benchmark-owned fixtures, electronics, and motion context are read-only environment inputs, not engineer-owned outputs.

### Integration Requirements
- Controller-first orchestration remains the product path for planning, validation, review, and retrieval.
- Heavy validation and simulation stay behind durable workflows.
- Observability must preserve session, episode, simulation, review, and seed lineage.
- The frontend must expose validation evidence, simulation playback, and benchmark setup inspection.
- Dataset generation must support reproducible export, seed prioritization, and filtering of polluted or incomplete episodes.

### Risk Mitigations
- Fail closed on missing manifests, invalid schemas, stale evidence, unsupported components, or invalid circuits.
- Keep validation preview distinct from final simulation proof.
- Require visual evidence inspection whenever renders exist.
- Exclude integration-only runs and known-corrupted windows from training data.
- Do not use single-unit price as the only cost signal when target quantity changes the economics of the design.
- Do not accept a design that passes simulation but cannot be manufactured within the requested volume, weight, or size constraints.
- Avoid exporting to high-fidelity simulation paths unless they are needed to validate the physical phenomenon being tested; cheaper preview paths speed up iteration, lower business cost, and make debugging faster.
- If a behavior is not reflected in the application and its documentation, it is not stable enough to serve as a debugging target in Phase 1.
- Do not describe the product as a general-purpose engineering automation system; it is bounded, evidence-backed, and human-scoped.

## Innovation & Novel Patterns

Problemologist-AI is a human-bounded engineering copilot for mechanical engineering, with electromechanical support and later fluids, that works more like `Cursor` or `Claude Code` for physical engineering than like a traditional CAD or simulation tool. The novelty is the closed loop: it solves a bounded engineering task, strictly validates that the proposed solution actually works, and preserves the full episode as reusable benchmark and training data.

### What Is Novel
- It targets a large class of engineering work that current AI cannot reliably handle yet, but only within a human-defined scope.
- It couples mechanical validation, manufacturability, cost/volume economics, and evidence inspection into one workflow.
- It extends into electronics only when those constraints are part of the same problem, rather than treating electronics as a separate toolchain.
- It produces training-ready mechanical engineering data and benchmark episodes as a first-class output.
- It keeps preview and proof separate, so fast iteration is not confused with final validation.

### Market Context
- Existing AI coding tools help with software workflows, but they do not solve physical engineering tasks.
- Existing CAD and simulation tools validate designs, but they do not generate reusable ML training episodes from the validation process.
- Existing electronics datasets are limited and usually not coupled to mechanical behavior.
- The gap is not another generic assistant. It is a system that can solve bounded engineering tasks and generate high-signal data from the verified solution path.

### Validation Approach
- Start with bounded mechanical tasks where success is explicit.
- Require simulation-backed evidence, manufacturability checks, and human review before acceptance.
- Extend to electromechanical tasks only when circuit validity and mechanical validation both hold.
- Measure solve rate, reproducibility, artifact completeness, and downstream dataset usefulness.

### Risks and Boundaries
- Avoid framing the product as general engineering autonomy.
- Keep human-scoped boundaries explicit.
- Fail closed on missing evidence, invalid schemas, unsupported mechanisms, or incomplete episodes.
- Treat dataset generation as a byproduct of verified work, not as a substitute for verified work.

## Project-Type Specific Requirements

### Project-Type Overview
Problemologist-AI is a simulation-backed RL environment for engineering tasks and benchmark generation. The core product is a family of sequential, role-specific environments where each agent observes its own handoff artifacts, takes CodeAct-style read/write/tool actions, receives role-specific rewards, and terminates through strict fail-closed gates. The environment exists to produce verified engineering solutions and reusable training data.

### Environment Contract
- Episodes are bounded units of work with explicit state and persisted lineage.
- Lineage means the reconstructable chain that links a run to its seed task, parent run, agent role, revision history, artifacts, reviews, and outputs, so the episode can be replayed and analyzed later.
- Each agent role has its own observation set, allowed CodeAct-style actions, and reward signal.
- In the current architecture, roles are executed sequentially and their outputs flow into the next role’s environment; the exact count of roles is implementation detail and may evolve.
- Terminal states are driven by validation and review gates, not free-form conversation outcomes.
- The top-level workflows are benchmark generation and engineering solving, but each workflow is decomposed into role-specific environments rather than treated as one monolithic loop.

### Technical Architecture Considerations

#### Observation and action space
- The environment must expose the same artifact surfaces the agents use in the workflow: plan docs, YAML handoffs, renders, simulation outputs, review manifests, journals, and code.
- Agent actions are restricted to CodeAct-style reading, writing, tool invocation, and handoff submission within the files and tools the role is allowed to touch.
- The tooling must be reproducible and inspectable so episodes can be replayed and analyzed later.

#### Reward and terminal gates
- Reward should align with the PRD’s own success criteria, especially the user, business, technical, and measurable outcomes already defined earlier in this document.
- Reward is role-specific, not one global scalar. Planners, coders, reviewers, and benchmark-generation roles each optimize different quality gates.
- Success means a verified, manufacturable, simulation-passing solution within benchmark constraints.
- Failure means invalid schema, missing artifacts, unsupported mechanisms, invalid circuits, or simulation failure.

#### State persistence and lineage
- Every episode must persist traces, seed lineage, artifacts, and review decisions.
- Lineage should include parent/child episode links, source seed, role outputs, revision numbers, and terminal reason.
- Dataset rows must be reconstructable from persisted artifacts, not only from prompts or transcripts.
- Integration-test runs and corrupted historical windows must be excluded from training data.

#### Simulation and validation interface
- Fast preview and heavy physics simulation are separate environment steps.
- Validation is part of the environment feedback loop, not a cosmetic UI check.
- Electromechanical and fluids-specific validation paths remain gated by the corresponding architecture contracts already established elsewhere in this document.

#### Workflow roles
- Planner, coder, and reviewer roles are policy layers acting inside the environment.
- The environment must preserve a clear distinction between benchmark authoring episodes and engineering solving episodes.
- The React UI is an inspection surface for environment state and evidence, not the source of truth for workflow state.

### Implementation Considerations
- Keep auth and multitenancy out of scope until the environment loop is stable.
- Optimize for episode fidelity, replayability, and training usefulness rather than generic platform features.
- Make the environment friendly to batch generation and seed variation so dataset coverage can grow.
- Keep terminalization, review gates, and artifact persistence fail-closed.
- The environment should support a small, role-decomposed agent chain rather than a single generic solver.

### Boundary Statement
- This is not a SaaS backend first and not a generic API product.
- It is a simulation-backed RL environment for engineering tasks and benchmark generation.
- The primary deliverable is a verified episode and its reusable dataset artifacts.
