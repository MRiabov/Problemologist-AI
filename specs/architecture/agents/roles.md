# Agent roles

<!--FIXME: in this document, incorrect heading levels due to automatic adjustment a while ago.-->

## Scope summary

- Primary focus: role-level behavior for benchmark generation and engineering execution.
- Defines Planner/Coder/Reviewer responsibilities, expected artifacts, and quality expectations.
- Includes concrete examples of `plan.md` and `benchmark_definition.yaml` style outputs.
- Use alongside handover contracts when implementing agent transitions.

## Benchmark generator agent (or graph)

## Agent purpose

The agent is to generate problems for an engineer to solve. This is important, as to train or fine-tune a model with reinforcement learning or prompt optimization we need challenges to scale, data to improve against.

## Agent subagents

1. Planner - compose a description of how the benchmark behaves, what the learning goal is, and design such a challenge (such a puzzle) that would teach the agent something; e.g., how gravity works, friction, dynamic objects, motors, etc.
2. A Benchmark Plan Reviewer that reviews the planner handoff before implementation starts.
3. A Benchmark Coder that implements the benchmark from the approved plan
4. A reviewer that reviews the implemented environment for
   - Feasibility of solution
   - Lack of violation of environment constraints (no significant, etc.)
   - Proper randomization.
   - No excessive or unjustified benchmark-side degrees of freedom.

## Output requirements

The benchmark generator writes code for the and gets internal reviews by the reviewer inside of the generator.

The benchmarks are randomized to ensure data distribution.

The benchmarks are consisting of CAD models which are converted into XML.

- The execution runs in isolated containers to prevent accidental harmful code execution in the main system.
- The benchmarks are verified for geometric validity and fast preview generation before they are handed to engineering. They are not solved by an engineer yet, just benchmarks are created and verified for validity.
  - Validity means no intersections and other problems; it also means that the code will compile in the first place.
  - MJCF is verified for correctness by XML schema and by backend-parity coverage in dedicated simulation tests.
  - The fast validation-preview path uses the renderer worker's selected preview backend for static handoff renders by default; it is not itself a Genesis-runtime proof path.
- MJCF is created programmatically, not by a LLM.
- Authored top-level part labels must be unique and must not be `environment` or start with `zone_`; those names are reserved for the scene root and simulator-generated objective bodies and collide with MJCF mesh/body naming if reused.

<!-- I will need to experiment, but I don't think the LLM should be able to edit it.  ->

## Benchmarks

The environments are made with static objects, dynamic objects, and motors. <!-- note - not sure if I handle dynamic objects atm. If I do, how are they specified in CAD? -->

Benchmark-owned motors and electronics are treated as fixture behavior, not as engineer-owned electrical design. They are validation context, so they do not become part of engineer pricing or manufacturability targets. Strict COTS identification still applies when they come from the catalog.

For benchmark-owned powered fixtures, we do not require benchmark-side wiring realism in MVP. Benchmark fixtures may be implicitly powered when the benchmark contract says so, and they may be fixed, partially constrained, motorized, or fully free when the benchmark contract explicitly says so.

Problems with motors and moving parts are verified more consistently because they are more prone to error.

<!-- note: it would be useful to persist an "expected solution" from the planner during task generator. It'll help guide exploration and maybe improve LLM optimization (prompt reflection/RL) with more data. -->

## Subagents output requirements

## Sample output

(input: Test benchmark: drop a ball into a funnel)

```markdown
`plan.md` (Benchmark Planner excerpt)

1. **Learning objective**
   - Test spatial redirection with gravity: the engineer must route a falling ball into a goal while avoiding a forbidden vertical-drop path.

2. **Environment geometry (with static randomization)**
   - `support_wall`: center `[0, -35, 40]`, size `[120, 6, 80]`, static scale range `[0.9, 1.15]`.
   - `funnel_body`: center `[32, 10, 18]`, top radius `20`, bottom radius `7`, height `26`, static scale range `[0.9, 1.1]`.
   - `goal_bin`: AABB min `[26, 4, 0]`, max `[42, 20, 10]`.
   - No moving parts in this benchmark (all `benchmark_assembly.parts[*].dofs: []`).

3. **Input objective (moved object)**
   - Shape: `sphere`.
   - Static randomization: radius in `[5, 7]`.
   - Nominal spawn position: `[0, 0, 70]`.
   - Runtime jitter: `[2, 2, 1]` (must be solved robustly across seeds).

4. **Objective locations**
   - `build_zone`: min `[-40, -30, 0]`, max `[40, 30, 70]`.
   - `forbid_zones`:
     - `vertical_drop_trap`: min `[-8, -8, 0]`, max `[8, 8, 20]`.
   - `goal_zone`: min `[28, 6, 1]`, max `[40, 18, 9]`.

5. **Simulation bounds**
   - min `[-60, -60, 0]`, max `[60, 60, 90]`.

6. **Constraints handed to engineering**
   - Planner authors realistic estimate fields in `benchmark_definition.yaml`: `constraints.estimated_solution_cost_usd: 30.0` and `constraints.estimated_solution_weight_g: 733.3`.
   - Runtime derives benchmark/customer caps during `submit_plan()`: `max_unit_cost = 1.5 * estimated_solution_cost_usd = 45.0 USD`, `max_weight_g = 1.5 * estimated_solution_weight_g = 1100.0 g`.

7. **Success criteria**
   - Success if the moved object's center enters `goal_zone` without touching any forbid zone.
   - Fail if object exits `simulation_bounds`.

8. **Planner artifacts**
   - Write `todo.md` implementation checklist.
   - Write draft `benchmark_definition.yaml` matching this geometry/constraint data.
   - Write draft `benchmark_assembly_definition.yaml` with per-part DOFs/control in `benchmark_assembly.parts` (benchmark-owned handoff artifact copied into engineer intake as read-only context).
   - Write `benchmark_plan_evidence_script.py` as the build123d sketch/evidence companion for the benchmark plan.
   - Write `benchmark_plan_technical_drawing_script.py` as the technical-drawing companion for that same benchmark plan.
   - Call `submit_plan()` to explicitly submit the planner handoff; completion is accepted only when `submit_plan()` returns `ok=true`.
```

The benchmark planner self-validates that `benchmark_plan_evidence_script.py` and `benchmark_plan_technical_drawing_script.py` preserve the same labels, repeated quantities, and COTS identities as the benchmark inventory before `submit_plan()`.
The benchmark planner also ensures every benchmark inventory label and selected COTS `part_id` appears at least once in `plan.md` as an exact identifier mention; backticks are preferred for the first mention.
Benchmark `plan.md` uses exact heading matching for the active benchmark-family schema; substring-only heading matches are not accepted.

```yaml
# benchmark_definition.yaml (draft from Benchmark Planner)
objectives:
  goal_zone:
    min: [28, 6, 1]
    max: [40, 18, 9]
  forbid_zones:
    - name: "vertical_drop_trap"
      min: [-8, -8, 0]
      max: [8, 8, 20]
  build_zone:
    min: [-40, -30, 0]
    max: [40, 30, 70]

simulation_bounds:
  min: [-60, -60, 0]
  max: [60, 60, 90]

moved_object:
  label: "projectile_ball"
  shape: "sphere"
  material_id: "abs"
  static_randomization:
    radius: [5, 7]
  start_position: [0, 0, 70]
  runtime_jitter: [2, 2, 1]

constraints:
  estimated_solution_cost_usd: 30.0
  estimated_solution_weight_g: 733.3

randomization:
  static_variation_id: "drop_ball_funnel_v2"
  runtime_jitter_enabled: true
```

### Benchmark reviewer split

The benchmark loop has two reviewer stages with different responsibilities.

`Benchmark Plan Reviewer` responsibilities:

01. Reject plans that mention benchmark objects, moving parts, joints, or objective markers that are not declared consistently across `plan.md`, `benchmark_definition.yaml`, and benchmark-owned `benchmark_assembly_definition.yaml`.
02. Validate plan consistency across `plan.md`, `todo.md`, `benchmark_definition.yaml`, and `benchmark_assembly_definition.yaml`, including label/quantity/COTS-identity exactness in `benchmark_plan_evidence_script.py` and `benchmark_plan_technical_drawing_script.py`, a structural build123d `TechnicalDrawing` import-and-call check, plus exact identifier mention coverage in `plan.md`. COTS-bearing rows must preserve the authored label/COTS-ID pairing; independent label counts and COTS-ID counts are not enough if the pairings are swapped across rows.
03. Reject plans that rely on free-form XYZ placement as the primary positioning mechanism; require selector-driven placement, explicit mates/joints, or clearly bounded absolute anchors.
04. Validate feasibility of the planned benchmark geometry before implementation starts, including objective clearance, randomization sanity, and that the moved object/runtime jitter contract stays inside benchmark bounds.
05. Validate non-ambiguity and completeness of planner handoff artifacts.
06. Reject unsupported benchmark-side mechanisms or metadata outside current benchmark contracts/tooling.
07. Reject benchmark-side motion that is underspecified for engineering intake. If a benchmark fixture moves, the planner handoff must declare reviewer-visible motion facts such as actuator type, axis/path, motion range or target state, and whether the engineer may rely on that motion. Benchmark fixtures are validation context, not manufactured solution parts, so the reviewer checks reviewability and solvability rather than engineer manufacturability.
08. Benchmark fixtures may be fixed, partially constrained, motorized, or fully free when the benchmark contract explicitly declares that behavior; the reviewer validates the declared motion against evidence rather than minimizing DOFs.
09. Reject impossible, contradictory, or unstable benchmark-side motion. Benchmark fixtures may be less physically constrained than engineering solutions, but they still must not rely on teleporting geometry, free-floating actuators, or unreviewable joint setups.
10. Reject planner handoff when `moved_object.material_id` is missing, empty, or not a known material from `manufacturing_config.yaml`, or when `benchmark_assembly_definition.yaml` is not a schema-valid full `AssemblyDefinition` artifact even if the planner only intends a minimal benchmark-side fixture declaration.
11. This stage is review-only. `Benchmark Plan Reviewer` inspects planner artifacts and evidence but does not rewrite planner-owned files, and when render images exist for the current revision, visual inspection through `inspect_media(...)` is mandatory before approval under the role policy in `config/agents_config.yaml`.

`Benchmark Reviewer` responsibilities:

1. Verify the implemented benchmark follows the approved plan or has justified, reviewable deviations, including the approved benchmark inventory labels, repeated quantities, and COTS identities. Reject pair-swapped COTS rows even when the separate label and COTS-ID counts happen to match.
2. Verify the implemented environment is geometrically valid and simulation-valid for the latest revision.
3. Verify the benchmark remains solvable, properly randomized, and consistent with the declared motion contract and observed simulation behavior after implementation.
4. Reject implementations that rely on free-form XYZ placement instead of selector-driven placement, explicit mates/joints, or the few absolute anchors already fixed by the approved plan.
5. For benchmarks with powered fixtures or moving benchmark-owned parts, require dynamic simulation evidence for the latest revision rather than relying on preview images alone.
6. Execute only after successful validation/simulation handoff artifacts are present for the latest revision, including `.manifests/benchmark_review_manifest.json`.
7. When simulation video exists for the current revision, inspect that video through `inspect_media(...)` before approval. Static renders remain mandatory context, but they do not replace dynamic evidence for moving benchmarks.
8. When render images exist for the current revision, visual inspection through `inspect_media(...)` is mandatory before approval under the role policy in `config/agents_config.yaml`.
9. Reviewer completion uses `write_file` for the stage-owned review artifacts and then `bash scripts/submit_review.sh` as the explicit local submission gate.

Benchmark-side reviewer manifest naming:

- Plan reviewer stage: `.manifests/benchmark_plan_review_manifest.json`
- Execution reviewer stage: `.manifests/benchmark_review_manifest.json`

Benchmark-side reviewer persistence naming:

- Plan reviewer writes:
  - `reviews/benchmark-plan-review-decision-round-<n>.yaml`
  - `reviews/benchmark-plan-review-comments-round-<n>.yaml`
- Execution reviewer writes:
  - `reviews/benchmark-execution-review-decision-round-<n>.yaml`
  - `reviews/benchmark-execution-review-comments-round-<n>.yaml`

## Engineer (problem solver)

## Purpose

There are real-life engineering problems that LLM AI agents can help overcome. This agent is an engineer that should be capable of solving problems, given their visuals/CAD designs, and produce solutions as to how to solve a given problem; constrained on physics. The agent should operate in the most closely real-life environment. The agent shouldn't even understand it works in a "fake" environment.

## Description

Writes CAD code to solve the problems (the benchmarks as above).

Is constrained by cost and manufacturability. Also constraints such as weight and where the agent can actually build their solution.

Has access to all realistic CAD tools.

The agent can preview their tools and check for whether the designs are manufacturable without being "punished".

It is OK if agents are running for 10 or more minutes (in production).

Again, the execution runs in isolated containers to prevent accidental harmful code execution in the main system.

## Engineer agent details

Engineer has explicit node roles:

1. `Engineering Planner` (mechanical plan author)
2. `Electronics Planner` (electrical planning companion stage)
3. `Engineering Plan Reviewer` (plan-quality reviewer before coding)
4. `Engineering Coder` (unified mechanical + electrical implementation owner)
5. `Electronics Reviewer` (electrical/electromechanical specialist review when required)
6. `Engineering Execution Reviewer` (final Execution Reviewer after validated/simulated implementation handoff)

The architect will create and persist a TODO list. The engineer must implement. The agent will have easy access to the TODO list.

- The engineer, after *proving* the TODO list is impossible, can refuse the plan.

<!-- Standard best practices will be used. No point in making anything weird in this application. -->

### Planner workflow

The Engineering Planner workflow is:

1. **Intake and mandatory context read**

   - Read `benchmark_definition.yaml` as present from the benchmark generator (goal/forbid/build zones, runtime jitter, planner-authored benchmark estimates, and runtime-derived benchmark/customer caps `max_unit_cost`/`max_weight_g`).
   - Read `benchmark_assembly_definition.yaml` as required benchmark-owned read-only handoff context copied into the engineer workspace. Use it to understand benchmark-owned fixtures, motion, and which benchmark-owned components explicitly allow engineer interaction, but fail closed if the file is missing and do not treat it as an engineer-owned costing artifact.
   - Read benchmark visuals from `renders/benchmark_renders/`, the planner's own stage bundle under `renders/engineer_plan_renders/` once it exists, and environment geometry metadata.
   - Read required skill and config inputs (CAD drafting skill, manufacturing knowledge when cost/quantity matters, manufacturing config + catalog).

2. **Plan the mechanism and budgets**

   - Propose a physically feasible mechanism that fits build-zone constraints and runtime jitter, and fit
   - Set planner-owned `max_unit_cost` and `max_weight_g` **under** benchmark/customer caps.
   - Minimize motion complexity: use the smallest DOF set needed to satisfy the objective; avoid unnecessary moving axes.
   - Select candidate COTS parts (motors/fasteners/bearings/gears) via the COTS Search subagent and carry part IDs + catalog prices into the plan.

3. **Calculate the costs per part**:

We want to estimate a rough, but detailed prices and architecture of the solution.

Create a file like `assembly_definition.yaml` containing:

For each part:
1\. A name of the part
2\. a description of the part,
3\. costing information each part in the simulation, e.g. for CNC, blank size, and final volume of the part (and calculate how much to be removed). The inputs are auto-validated, per manufacturing method

Then: create an assembly structure, like:

```yaml
final_assembly:
  - subassembly_1: 
      parts:
        - part_1:
            dofs: ["dof_2", "dof_1"]
        - part_2:
            dofs: []
        - motor_a:
            dofs: ["rotate_z"]
            control:
              mode: sinusoidal
              speed: 1.0
      joints: 
      - joint_1: 
          parts:
            - part_1
            - part_2
          type: fastener_joint 
  - subassembly_2:
      parts:
        - part_4:
            dofs: []
        - part_1: # part 1 is inserted twice. Hence it'll be estimated as necessary to manufacture it twice, hence unit costs will drop (as per manufacturing method config)
            dofs: ["dof_2", "dof_1"]
      joints:
        - joint_2:
            parts:  #note: maybe we want to inline it.
              - part_4
              - part_1
            type: fastener_joint
  - part_5 
```

Then: Run a script like `validate_costing_and_price.py` that would automatically validate the YAML file for consistency and output pricing. The model can thus use a stricter constraint.

Notably, if the plan is higher than the max_unit_cost, it can't proceed and needs to adapt the plan.

1. **Write required planner artifacts**

- Create `plan.md` using the strict engineering structure:
  `## 1. Solution Overview`, `## 2. Parts List`, `## 3. Assembly Strategy`, `## 4. Assumption Register`, `## 5. Detailed Calculations`, `## 6. Critical Constraints / Operating Envelope`, `## 7. Cost & Weight Budget`, `## 8. Risk Assessment`.
- In `plan.md`, include manufacturing method/material choices, assumption sources, the detailed-calculation index table and matching `CALC-*` proof subsections from the handover contract, operating limits, assembly strategy (including rigid-connection fastener strategy), and risk mitigations.
- Create `todo.md` as an implementation checklist for the Engineering Coder (initially `- [ ]` items).
- Create `assembly_definition.yaml` with per-part costing fields (method-specific) and a `final_assembly` structure for reuse/quantity accounting.
- Write `solution_plan_evidence_script.py` as the build123d sketch/evidence companion for the drafted solution geometry.
- Write `solution_plan_technical_drawing_script.py` as the technical-drawing companion for that same solution geometry.
- Call `submit_plan()` to explicitly submit the planner handoff; completion is accepted only when `submit_plan()` returns `ok=true`.

At this point, the planner can handoff the documents to the Engineering Coder. Before handoff, the planner runs a standalone script from `.agents/skills/manufacturing-knowledge/scripts/validate_costing_and_price.py` to validate `assembly_definition.yaml` and compute assembly totals (including geometry-driven fields such as part volume, blank/stock size, stock volume, and removed volume for CNC). If the estimated cost is above `max_unit_cost`, the planner cannot proceed and must adapt the plan. The planner's documents are autovalidated; if validation fails, handoff (submission) is refused until fixed. (the validation is currently implemented as Pydantic validation.)

The Engineering Planner also self-validates that `solution_plan_evidence_script.py` and `solution_plan_technical_drawing_script.py` preserve the same labels, repeated quantities, and COTS identities as `assembly_definition.yaml`, and that `solution_plan_evidence_script.py` passes the 3D self-intersection and overlap gate before `submit_plan()`. Binding numeric claims should have a trace from `Assumption Register` through `Detailed Calculations` into `Critical Constraints / Operating Envelope`, with the calculation table rows and `CALC-*` subsections remaining one-to-one.
The Engineering Planner also ensures every planner-declared inventory label and selected COTS `part_id` appears at least once in `plan.md` as an exact identifier mention; backticks are preferred for the first mention. For COTS-bearing rows, the authored label and COTS identity are a single identity-bearing pair, so pair-swaps across rows are invalid even if the separate counts still look right.

### Unified implementation ownership

Implementation is not split into a mechanical coder followed by a separate electronics implementer.

The architecture rule is:

1. `Engineering Planner` owns the mechanical planning pass.
2. `Electronics Planner` adds electrical requirements, component choices, and wiring intent when the benchmark declares explicit electronics.
3. `Engineering Plan Reviewer` approves or rejects the combined planner handoff.
4. `Engineering Coder` then implements the whole approved solution in one workspace revision, including geometry, controller behavior, electronics definitions, and any wire-routing logic required by the approved plan.
   - The implementation must preserve the approved planner inventory: manufactured-part labels, repeated quantities, and COTS identities must match the planner-declared multiset.
   - The coder may refactor internal geometry construction, but missing, extra, or relabeled inventory items are plan violations, not acceptable implementation freedom.
   - If drafting mode is enabled for the approved plan, the technical-drawing companion is part of the binding plan package and must remain consistent with the implemented inventory.
5. `Electronics Reviewer` validates the electromechanical implementation when electronics are present.

We choose this because electromechanical implementation is often co-dependent:

1. wire-routing constraints can require geometry changes,
2. PSU or connector packaging can require mounting changes,
3. moving-part clearance can require both wiring and mechanical edits,
4. a late serialized electrical-only coding stage creates unnecessary handoff loops and stale assumptions.

The implementation split is therefore planner-specialized but coder-unified.

Reviewer completion in the engineer graph uses `write_file` for the stage-owned review artifacts and `bash scripts/submit_review.sh` as the local submission gate.

### Engineering reviewer split

The engineering loop has two reviewer stages with different responsibilities.

`Engineering Plan Reviewer` responsibilities:

01. Reject plans that propose unsupported components/mechanisms outside the current allowed system/tooling/contracts.
02. Validate plan consistency across `plan.md`, `todo.md`, `benchmark_definition.yaml`, and `assembly_definition.yaml`, including label/quantity/COTS-identity exactness in `solution_plan_evidence_script.py` and `solution_plan_technical_drawing_script.py`, a structural build123d `TechnicalDrawing` import-and-call check, plus exact identifier mention coverage in `plan.md`.
03. Validate feasibility (physics realism, build-zone fit, planner budgets under benchmark caps, and the presence of a source-backed motion forecast, assumptions, calculations, and operating-envelope limits for any binding numeric claims).
04. Reject plans that rely on free-form XYZ placement as the primary positioning method; require selector-driven placement, explicit mates/joints, or clearly bounded absolute anchors.
05. Validate non-ambiguity and completeness of planner handoff artifacts.
06. Re-run pricing/weight validation (`.agents/skills/manufacturing-knowledge/scripts/validate_and_price.py` or equivalent tool-wrapped validator) against the planner handoff and reject mismatches/failures.
07. Reject plans with excessive DOFs; each non-empty `final_assembly.parts[*].dofs` entry must be necessary for the mechanism and justified in planner artifacts, typically in `Assembly Strategy`, `Detailed Calculations`, `Critical Constraints / Operating Envelope`, or `Risk Assessment`.
08. Deterministic DOF suspicion rule: any part with `len(dofs) > 3` is treated as suspicious over-actuation and is rejected unless explicit mechanism-level justification is present and reviewer accepts that evidence.
09. Future work (non-blocking for now): recommend cost/weight optimizations and flag unrealistic or overdesigned targets.
10. When render images exist for the current revision, visual inspection through `inspect_media(...)` is mandatory before approval under the role policy in `config/agents_config.yaml`.

`Engineering Execution Reviewer` responsibilities:

1. Verify implementation follows the approved plan at the inventory level, including labels, repeated quantities, COTS parts/identities, and any drafting package required by the approved plan. Justified, reviewable deviations are allowed only when they do not change the approved inventory contract. COTS pair-swaps are failures even when the label and COTS-ID multisets match separately.
2. Verify robustness and non-flakiness of the successful solution, using simulation evidence across runtime randomization.
3. Execute only after successful validation/simulation handoff artifacts are present (`validation_results.json`, `simulation_result.json`, and the coder-written `.manifests/engineering_execution_handoff_manifest.json` for the latest revision). This stage is post-success auditing, not initial simulation pass/fail gating.
4. Reject execution-time evidence of free-form XYZ placement that was not grounded in the approved datum/joint chain, even when the layout happens to pass on one run.
5. Flag execution-time evidence of over-actuated designs (unnecessary moving parts/axes) as a robustness risk, even when single-run success exists.
6. Optional code-quality review is secondary and non-blocking unless it reveals concrete safety/correctness risk.
7. When render images exist for the current revision, visual inspection through `inspect_media(...)` is mandatory before approval under the role policy in `config/agents_config.yaml`.

Engineer-side visual-inspection policy:

1. `Engineering Planner`, `Engineering Coder`, `Engineering Plan Reviewer`, and `Engineering Execution Reviewer` are all policy-configured visual-inspection roles.
2. The requirement is conditional on render-image availability in `renders/**`.
3. The current policy minimum is one distinct image per required node, but the architecture treats this as config-owned rather than hardcoded.
4. Engineering Coder must be able to inspect benchmark evidence under `renders/benchmark_renders/` and engineer-plan evidence under `renders/engineer_plan_renders/` when those bundles exist for the current revision. Engineering Execution Reviewer must be able to inspect benchmark evidence under `renders/benchmark_renders/` and final solution submission evidence under `renders/final_solution_submission_renders/` when those bundles exist. Engineering Plan Reviewer must be able to inspect benchmark evidence, engineer-plan evidence, final solution submission evidence, and scratch evidence under `renders/current-episode/` when those artifacts exist for the current revision.

Reviewer manifest naming in engineering:

- Plan reviewer stage: `.manifests/engineering_plan_review_manifest.json`
- Execution reviewer stage: `.manifests/engineering_execution_handoff_manifest.json`

Note: the execution-review manifest is produced by coder submission and then
consumed by the execution reviewer. The reviewer does not invent a separate
file with a different name.

Reviewer persistence naming in engineering:

- Plan reviewer writes:
  - `reviews/engineering-plan-review-decision-round-<n>.yaml`
  - `reviews/engineering-plan-review-comments-round-<n>.yaml`
- Execution reviewer writes:
  - `reviews/engineering-execution-review-decision-round-<n>.yaml`
  - `reviews/engineering-execution-review-comments-round-<n>.yaml`

<!-- 
4. **Pre-handover validation gate**
   - Ensure markdown/YAML structure is valid (plan sections + list/table requirements, TODO checkbox format).
   - Verify constraints/logic consistency: units, build-zone fit, cost/weight bounds, and no invented catalog pricing.
   - Planner submission is treated as invalid if required files are missing or malformed.

5. **Handover and iteration loop**
   - Handover `plan.md` + planner-constrained objectives + `todo.md` to the Engineering Coder.
   - Engineering Coder implements and may request refusal only with proof that planner constraints/approach are infeasible.
   - Reviewer either confirms refusal (`confirm_plan_refusal`) and routes back to Planner for re-plan, or rejects refusal (`reject_plan_refusal`) and routes back to CAD implementation.

6. **Observability**
   - Emit structured events for plan submission, COTS search usage, markdown/YAML failures, logic/constraint failures, and plan-refusal decisions for downstream evaluation. -->

## Verification

The Engineer agent will verify its work by:

1. Checking the manufacturability of its solution, on
   1. Individual part scale (this part can be machined)
   2. Assembly scale - this assembly has no interference of parts and has valid part constraints.
2. Checking the cost of its solution; against the part count and unit cost as specified by the user.
3. Checking the weight of its solution.
4. Simulating - did the model achieve the goal as per the benchmark?
5. The Execution Reviewer assesses whether the successful simulation is stable and non-flaky for realistic repeated runs.

## COTS Search subagent

An engineering agent(s) can invoke a COTS (Commercial-Off-The-Shelf) search agent to delegate the search for off-the-shelf components. E.g.: search for motors.

Purpose: a lightweight subagent that performs catalog lookups and returns verified part candidates.

### Model and runtime

- Uses a smaller/cheaper model than the primary planner/engineer.
- Read-only access to the COTS catalog DB and/or CLI; no writes.
- No workspace file edits; results are returned to the caller as a structured payload.

### Inputs (from planner/engineer/Benchmark Planner/Benchmark Coder)

- One request string from `invoke_cots_search_subagent(...)`.
- That request string may contain part intent (e.g., "M3 fasteners", "servo motor 3-5 kg\*cm", "bearing 608") plus constraints such as quantity tier, max_unit_cost, size/torque/voltage limits, material, and mounting/shaft constraints.

### Tools

- Read-only SQL queries against the COTS catalog database.
- Read-only CLI helpers/scripts (if provided) for catalog search.

### Outputs (structured, concise):

- 3-10 candidate parts with `part_id`, `manufacturer`, `key specs`, `unit_cost`, `quantity_tier`, `source`, `why it fits`.
- If no match, explicitly say "no match" and list which constraints were too strict.

### Invocation

- Engineering Planner, Coder, reviewers and Benchmark Planner, Benchmark Coder, reviewers call the same prompt-only subagent node whenever a COTS part is needed (motors, bearings, fasteners, gears, etc.).
- There are no graph-specific COTS signature variants and no inherited task/plan/journal handoff.
- The returned part IDs and prices must be used in the plan and cost estimates.

Notably the Benchmark Planner will need it too since they are also responsible for (hard cap) price estimation.

### Reasons for invocation

1. Planners in benchmark and engineering invoke to check exact prices for subcomponents (both make price decisions)
2. Engineers invoke them because they need to use them and check prices for components (they are constrained by the price)
3. Reviewers may search for a better component *(suggestion: reviewers may want to read the search queries of invoking agents to decide if the part found was sufficiently good or not. If the query is good enough—can just skip!)*

## COTS catalog database (spec 006)

This system is backed by a SQL catalog built from `bd_warehouse`. The catalog is **read-only in workers** and queried only via the COTS Search subagent.

Database:

- Artifact: `parts.db` (SQLite; local on worker, read-only).
- Build-time indexer extracts metadata and usage snippets from `bd_warehouse`.
- Store reproducibility metadata: `catalog_version`, `bd_warehouse_commit`, `generated_at`, `catalog_snapshot_id`.
- `catalog_snapshot_id` is a deterministic snapshot fingerprint created at catalog build time (prefer hash of canonical exported catalog content; acceptable fallback is hash of `parts.db` bytes).
- Every COTS handoff artifact must persist:
  - catalog metadata (`catalog_version`, `bd_warehouse_commit`, `generated_at`, `catalog_snapshot_id`)
  - query snapshot (normalized intent + constraints + limit/sort)
  - selection snapshot (ordered returned candidates + final selected `part_id`s used by planner/engineer)
- COTS reproducibility is invalid if selected parts are persisted without the snapshot metadata above.

<!-- TODO move details away from this section into CAD section... -->
