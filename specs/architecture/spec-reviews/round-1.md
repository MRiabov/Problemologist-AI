# Architecture Spec Review - Round 1

## Scope summary

- This document records the first structured review of `specs/architecture/**` for MVP and paper-readiness.
- It focuses on benchmark publishability, evaluation hygiene, reproducibility, artifact contracts, and architecture scope clarity.
- It is a review document, not a replacement for the architecture specs themselves.
- Closure for the findings below requires edits in the source architecture documents, not only agreement in this file.

## Review outcome

The current architecture is much stronger on internal agent workflow plumbing than on benchmark-release rigor.

The current docs already specify many important internal contracts well:

- role splits between planner, coder, and reviewer,
- reviewer manifests and fail-closed handoff gates,
- worker/controller split,
- validation-preview versus simulation backend split,
- filesystem ownership and artifact immutability rules,
- observability IDs and review-evidence tracking.

The main missing layer is the benchmark-as-product contract. The docs explain how agents should operate, but they do not yet fully define how a benchmark release becomes:

- frozen,
- held out,
- solvability-certified,
- reproducible,
- sliceable for paper analysis,
- and comparable across models on a leaderboard.

That gap is large enough to matter for both MVP credibility and any paper or public benchmark release.

## Priority order for MVP and paper

The three highest-priority gaps are:

1. Benchmark protocol
2. Train/dev/test split and contamination policy
3. Reference solutions and difficulty calibration

The next priority group is:

4. Supported-capabilities contract
5. Reproducibility bundle
6. Environment attachment and drillability contract

The final priority group is:

7. Benchmark taxonomy metadata
8. Electronics scope decision

## Findings

### 1. Missing paper-facing benchmark protocol

#### Why this matters

`evals-and-gates.md` is an internal agent-quality document. It does not yet define the external benchmark contract that makes model comparisons fair and reproducible.

Without a benchmark protocol, it is unclear:

- what exact prompt package is frozen for evaluation,
- which tools are allowed,
- what turn, time, token, and submission budgets apply,
- how many seeds or repeats are required,
- what the primary metric is,
- how secondary metrics are reported,
- how model runs are aggregated into one official score.

That ambiguity is acceptable during internal prototyping, but it is a blocker for a paper, a leaderboard, or any claim of cross-model comparability.

#### Current evidence in the architecture

- `specs/architecture/evals-and-gates.md` defines internal fast, medium, and slow evals for agents.
- The architecture does not currently define a frozen benchmark execution contract for external reporting.
- Budget-related concepts appear in scattered places (`OUT_OF_TURN_BUDGET`, `OUT_OF_TOKEN_BUDGET`, tool-call limits), but not as a single benchmark protocol.

#### Required spec change

Add a dedicated architecture document such as `specs/architecture/benchmark-protocol.md`.

That document should define:

1. The official evaluation unit.
   - One benchmark instance
   - One benchmark family
   - One family across multiple runtime seeds
2. The frozen input package.
   - System prompt hash
   - User/task prompt template
   - Allowed context artifacts
   - Prompt/config version identifiers
3. The allowed tool surface.
   - Which tools are allowed in benchmark evaluation
   - Whether internet access is allowed
   - Whether skill creation or prompt editing is allowed during an official run
4. Resource budgets.
   - Turn limit
   - Token limit
   - Wall-clock limit
   - Tool-call limit
   - Validation and simulation retry budgets
5. Official metrics.
   - Primary metric
   - Secondary metrics
   - Cost/weight compliance reporting
   - Robustness reporting across seeds
6. Aggregation rules.
   - Mean versus median
   - Family-level weighting
   - Seed-level aggregation
   - Whether failures count as zero or are excluded only under explicit infra-failure rules
7. Reporting rules.
   - What a leaderboard entry must include
   - What a paper result table must include
   - What metadata is mandatory for reproducibility

#### Closure criteria

- A single benchmark protocol document exists and is linked from `specs/desired_architecture.md`.
- The protocol defines frozen prompts, allowed tools, budgets, seed policy, and official metrics.
- `evals-and-gates.md` cross-references the benchmark protocol instead of implicitly serving as one.
- `observability.md` is updated so every official benchmark run records the protocol version and budget counters required by that document.

### 2. No train/dev/test split and contamination policy

#### Why this matters

The architecture already includes:

- online skill learning,
- prompt evolution and debugging,
- randomized benchmark families,
- persistent traces and journals,
- and dataset generation from successful runs.

That makes contamination risk unavoidable unless the split policy is explicit and enforced. Right now there is no architecture-level rule for what data may influence prompts, skills, or tuning after a benchmark is assigned to a dev or test partition.

Without a contamination policy, test performance becomes difficult to trust.

#### Current evidence in the architecture

- `observability.md` defines lineage fields such as `seed_id`, `seed_dataset`, `generation_kind`, and `parent_seed_id`.
- The docs do not define held-out benchmark partitions or family-level split rules.
- The docs do not define quarantine rules for traces, reviews, journals, or learned skills derived from benchmark runs.

#### Required spec change

Add a dedicated split and contamination section, either inside `benchmark-protocol.md` or in a separate `benchmark-splits-and-contamination.md`.

The architecture should define:

1. Split objects.
   - `benchmark_id`
   - `benchmark_family_id` or `family_id`
   - `benchmark_release_id`
   - `variant_id`
2. Family-level split assignment.
   - Split at the family/template level, not only per rendered instance
   - All close variants of a family stay in the same split
3. Dedupe and similarity controls.
   - Geometry fingerprint
   - Task fingerprint
   - Constraint fingerprint
   - Randomization-template fingerprint
4. Quarantine rules.
   - Test-split traces do not feed skill learning
   - Test-split failures do not feed prompt editing
   - Test-split artifacts are not used as seed material for future benchmark generation
   - Test-split review outcomes are visible for reporting, not for training or optimization
5. Allowed use by split.
   - Training split may feed skill and prompt improvement
   - Dev split may drive iteration and ablations
   - Test split is frozen and evaluation-only
6. Human-debugging policy.
   - Whether engineers may inspect test failures
   - Whether such inspection invalidates the held-out status
   - Whether a new test release must be cut after contamination

#### Closure criteria

- Every benchmark item and family has stable IDs and fingerprints.
- Split assignment is defined at family level.
- The architecture explicitly prohibits test-split leakage into skills, prompts, benchmark seeds, and learned artifacts.
- `observability.md` is updated to persist split membership and quarantine state per run.
- Official evaluation runs are invalid unless they target a frozen split release.

### 3. No reference-solution and solvability certification flow

#### Why this matters

The current specs require benchmark reviewers to judge solvability and reject impossible environments. That is useful, but it is not enough for a publishable benchmark.

A benchmark should not be released only because a reviewer believes it is solvable. It should be released because at least one frozen solution bundle has actually solved it under the benchmark protocol.

This is especially important because the project goal is not only to generate interesting tasks, but to produce a benchmark and dataset for evaluating engineering agents.

#### Current evidence in the architecture

- The benchmark reviewer is expected to check feasibility and solvability.
- There is no release-gate artifact that stores a certified solution for each released benchmark.
- Difficulty is discussed informally, but no concrete difficulty-tier contract is defined.

#### Required spec change

Add a solvability certification flow and a stored `reference_solution_bundle` contract.

The architecture should define:

1. Release gate.
   - A benchmark is not publishable until at least one frozen reference solution passes the official protocol.
2. Reference solution bundle contents.
   - Approved planner artifacts
   - Approved implementation artifacts
   - Validation outputs
   - Simulation outputs
   - Render evidence
   - Seed coverage used for certification
   - Solver provenance
   - Protocol version
3. Solver provenance.
   - Human-authored
   - Internal baseline model
   - External baseline model
   - Date and revision metadata
4. Difficulty tier.
   - Simple, medium, hard, or equivalent
   - Defined from actual baseline/reference performance, not only author intuition
5. Optional baseline table.
   - Store one or more baseline runs so difficulty and future regression analysis are measurable

#### Closure criteria

- Each released benchmark has at least one immutable `reference_solution_bundle`.
- The release flow rejects benchmarks that lack a certified reference solution.
- Difficulty tier is defined from a documented calibration rule.
- `observability.md` and release metadata capture the reference solution identifier used for certification.

### 4. Supported-capabilities contract is unfinished

#### Why this matters

Several reviewer requirements depend on rejecting unsupported components, mechanisms, or operations. That only works if there is a source-of-truth list of what the system actually supports.

Right now the architecture clearly expects such a contract, but the explicit capability source is still a placeholder.

#### Current evidence in the architecture

- `specs/architecture/agents/artifacts-and-filesystem.md` contains an unfinished note about a `capabilies.md` document.
- Reviewer specs require rejection of unsupported or invented mechanisms.
- `simulation-and-dod.md` lists some allowed components and joints, but not as a complete versioned capability manifest.

#### Required spec change

Define a canonical capability contract, ideally as both:

- a human-readable architecture document, and
- a machine-readable manifest version referenced by prompts, validators, and review gates.

The capability contract should define:

1. Allowed component families.
   - Environment fixtures
   - Manufactured parts
   - Fasteners
   - Motors
   - Bearings
   - Any currently supported COTS categories
2. Allowed mechanism primitives.
   - Weld/rigid
   - Revolute
   - Prismatic
   - Explicitly unsupported joint types
3. Allowed control modes.
   - Open-loop control types
   - Parameterized controllers
   - Explicitly unsupported control logic
4. Allowed materials and workbenches.
5. Allowed environment-modification operations.
   - Drilling
   - Fastening
   - Clamp-on mounting
   - Adhesive mounting
   - Or a narrower MVP subset if preferred
6. Explicit non-goals for MVP.
   - Electronics, belts, gears, fluids, deformables, conveyor systems, and other unsupported mechanisms if they remain out of scope

#### Closure criteria

- A canonical capability document exists and is linked from the architecture index.
- Reviewer rejection rules reference that manifest, not only generic prompt wording.
- The manifest is versioned and recorded in run metadata for reproducibility.
- Unsupported mechanisms are enumerated explicitly rather than left implicit.

### 5. Environment attachment and drillability contract is still underspecified

#### Why this matters

The engineering planner needs a concrete environment-mounting contract. Without it, attachment planning remains prompt-dependent and ambiguous.

The current docs already recognize the issue. They say benchmark fixtures may be drillable or non-drillable and mention a future YAML handoff, but the actual schema and handoff path are not yet defined.

That creates ambiguity in:

- plan feasibility,
- review correctness,
- validator behavior,
- and future reproducibility.

#### Current evidence in the architecture

- `simulation-and-dod.md` says the Engineering Planner should receive a YAML file describing what can be drilled and what cannot.
- The same section explicitly notes uncertainty about how to pass it.
- No concrete artifact name or schema is currently specified in handover contracts.

#### Required spec change

Define a deterministic environment attachment artifact, for example `environment_mounts.yaml`.

That artifact should include, at minimum:

1. Stable environment object and surface identifiers.
2. Allowed attachment operations per surface.
   - Drill
   - Bolt-through
   - Clamp
   - Adhesive
   - No-attachment
3. Drillability constraints.
   - Allowed hole count
   - Allowed diameter range
   - Allowed depth
   - Edge clearance
   - Surface thickness constraints
4. Orientation and normal data.
5. Forbidden faces and keep-out regions.
6. Benchmark-level rationale where needed.
   - Example: "do not attach to motor housing"
7. Handoff rules.
   - Which stage creates the file
   - Which roles may read it
   - Which validator enforces it

#### Closure criteria

- A concrete environment-mounting artifact is named and schema-defined.
- Benchmark-to-engineer handoff explicitly includes it where applicable.
- Reviewers and validators consume the same artifact instead of prompt-only heuristics.
- `simulation-and-dod.md` no longer contains uncertainty about how the information is passed.

### 6. Benchmark taxonomy metadata is missing

#### Why this matters

`observability.md` calls for diversity and coverage metrics across:

- physics principles,
- DOFs,
- moving parts,
- templates,
- and other benchmark characteristics.

Those metrics should not rely only on post-hoc inference from traces. The architecture should define a required benchmark metadata artifact so dataset slicing and paper tables are deterministic.

#### Current evidence in the architecture

- Diversity metrics are listed in `observability.md`.
- There is no mandatory benchmark metadata artifact that stores those labels at benchmark release time.

#### Required spec change

Add a required artifact such as `benchmark_metadata.yaml`.

It should carry release-level labels such as:

1. `benchmark_id`
2. `family_id`
3. `release_id`
4. Benchmark title and one-line summary
5. Physics-principle labels
6. Actuation labels
7. Moving-part count
8. DOF summary
9. Environment template/category
10. Manufacturing regime and target quantity regime
11. Difficulty tier
12. Randomization regime
13. Reference solution identifier

This artifact should be created before release and treated as part of the benchmark package, not inferred later from logs.

#### Closure criteria

- A benchmark metadata artifact is defined and required for released benchmarks.
- Diversity metrics in `observability.md` reference that artifact as a canonical source.
- Paper tables and dataset slicing can be produced from benchmark metadata plus run manifests without manual relabeling.

### 7. Reproducibility bundle is incomplete

#### Why this matters

The docs already specify many pieces of reproducibility:

- lineage fields,
- catalog snapshot metadata,
- reviewer manifests,
- skill revisions,
- and stable artifact names.

What is still missing is a single immutable execution or release bundle that makes a benchmark result replayable end to end.

Without that bundle, a future reader may know many local details but still not be able to reproduce the exact run.

#### Current evidence in the architecture

- `observability.md` records IDs and lineage.
- `roles.md` requires COTS snapshot metadata.
- Reviewer manifests exist.
- There is no single architecture contract for an immutable run or release manifest that binds all of those together.

#### Required spec change

Define an immutable manifest bundle for both:

1. A benchmark release
2. An official evaluation run

This may be one document with two schemas, or two linked manifests.

The reproducibility contract should include:

1. Controller commit and worker commit
2. Worker image digests
3. Backend versions
   - MuJoCo
   - Genesis
   - build123d
   - catalog snapshot versions
4. Protocol version
5. Capability manifest version
6. Skill revision identifiers
7. Prompt/config hashes
8. Benchmark revision identifiers
9. Artifact hashes and URIs
   - CAD outputs
   - MJCF
   - renders
   - validation results
   - simulation results
10. Seed values and seed-policy identifiers
11. Final outcome and metric summary

#### Closure criteria

- Official runs emit an immutable run manifest.
- Released benchmark families emit an immutable release manifest.
- Accepted paper or leaderboard results must cite those manifests.
- Replay instructions can be derived from the manifest bundle without relying on undocumented runtime state.

### 8. Electronics scope is ambiguous

#### Why this matters

The architecture names electronics roles in multiple places:

- `Electronics Planner`
- `Electronics Engineer`
- `Electronics Reviewer`

But the rest of the architecture does not define electronics artifacts, validation rules, supported components, or benchmark categories at the same level of precision as the mechanical path.

This is currently a scope ambiguity rather than a minor documentation gap.

#### Current evidence in the architecture

- `agents/overview.md`, `agents/roles.md`, `agents/handover-contracts.md`, and `agents/artifacts-and-filesystem.md` all mention electronics roles.
- The architecture index does not point to a dedicated electronics contract.
- The benchmark and simulation docs remain overwhelmingly mechanical.

#### Required spec change

Make an explicit decision:

1. Either electronics are in scope for MVP and paper.
   - Add dedicated architecture docs and handoff contracts
   - Define artifact schemas, review gates, and capability limits
   - Integrate the electronics contract into the architecture index
2. Or electronics are out of scope for MVP and paper.
   - Mark them as future work
   - Remove or clearly demote them from the active benchmark protocol and acceptance criteria

For MVP, the cleaner choice is likely to mark electronics out of scope unless there is already a near-term electromechanical architecture with concrete artifacts and validators.

#### Closure criteria

- The architecture explicitly states whether electronics are in scope for MVP and the initial paper release.
- If in scope, the architecture index links to the electronics source-of-truth docs and the handoff contracts become concrete.
- If out of scope, the existing role mentions are clearly marked as future work rather than active release surface area.

## Recommended follow-up document additions

The minimal set of new source documents likely needed after this review is:

1. `specs/architecture/benchmark-protocol.md`
2. `specs/architecture/benchmark-splits-and-contamination.md` or an equivalent section inside the protocol doc
3. `specs/architecture/supported-capabilities.md`

The minimal set of new required runtime or release artifacts likely needed is:

1. `reference_solution_bundle`
2. `environment_mounts.yaml`
3. `benchmark_metadata.yaml`
4. `run_manifest.json` or `run_manifest.yaml`
5. `benchmark_release_manifest.json` or `benchmark_release_manifest.yaml`

## MVP recommendation

For MVP and a first paper-quality release, the architecture should explicitly optimize for:

- mechanical benchmarks only,
- frozen benchmark protocol,
- family-level held-out splits,
- certified reference solutions,
- versioned capability limits,
- and replayable run manifests.

That is a smaller scope than the full long-term vision, but it is coherent, defensible, and much easier to evaluate rigorously.

## Review status

This review is open until the source architecture docs absorb the missing contracts above.
