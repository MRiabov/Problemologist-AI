# Agent handovers

## Scope summary

- Primary focus: deterministic handoff contracts between planner, implementer, and reviewer roles.
- Defines which files are handed over, what each artifact must contain, and when refusals are valid.
- Specifies routing behavior for review outcomes, replanning loops, and benchmark-to-engineer transition.
- Use this file to validate inter-agent boundary correctness.

## All handovers that happen

User prompt ->
benchmark planner agent <-> Benchmark CAD agent <-> benchmark reviewer (If plan is not valid - e.g. it specifies a conflicting geometry, the CAD agent will refuse; and send back to benchmark planner agent. However, the benchmark CAD agent can not refuse because it fails to do CAD, it can only refuse if the model is in fact invalid.)
(Benchmark reviewer to CAD agent - if the environment CAD 3d model does not adhere to the plan OR the environment CAD model has invalid geometry e.g. intersections OR it is impossible to solve, the benchmark reviewer agent can refuse)

Benchmark reviewer "accepts" and passes the environment to the "lead engineer" - the Engineering Planner model. (indirect contact - no actual "communication")

Lead engineer -> Engineering Plan Reviewer -> CAD modelling engineer -> Engineering Execution Reviewer

The lead engineer will try to think of the cheapest and most efficient, but *stable* approach of solving the environment given known constraints (objectives info, cost/weight info, geometry info, build zones).
CAD modelling engineer can refuse the plan if the plan was inappropriate, e.g. set too low price or the approach was inappropriate.
In this case, the CAD modelling engineer must provide `plan_refusal.md` (with role-specific reasons + evidence). The Engineering Plan Reviewer validates that refusal and either confirms (`CONFIRM_PLAN_REFUSAL`) and routes back to planner, or rejects (`REJECT_PLAN_REFUSAL`) and routes back to implementation.

The "reviews" are made more deterministic by passing YAML frontmatter to markdown review documents (which are later parsed deterministically). The reviews and plans must be appropriate.

## Reviewer manifest naming contract

Reviewer handoff manifests are reviewer-scoped. We do not use a shared generic filename.

Required manifest filenames:

1. Benchmark reviewer: `.manifests/benchmark_review_manifest.json`
2. Engineering plan reviewer: `.manifests/engineering_plan_review_manifest.json`
3. Engineering execution reviewer: `.manifests/engineering_execution_review_manifest.json`
4. Electronics reviewer: `.manifests/electronics_review_manifest.json`

Validation rule:

- Reviewer entry is blocked if the reviewer-specific manifest for that stage is missing, stale, or invalid for the latest revision.

## Benchmark generator Planner and Benchmark CAD designer

The Benchmark Generator Planner will submit multiple files to the CAD implementing agent.

The plan will have the following bullet points. The plan will be validated for consistency, and will not be accepted until the markdown passes strict formatting criteria, and will ensure that the bullet points are there:

1. `plan.md`:
    - Learning objective (summary of what the agents needs to or will learn as a result of this challenge);
    - The geometry of the environment:
        - coordinates of all major shapes in the environment + randomization.
        - Geometry and coordinates of all moving parts:
            - motors
            - non-fixed parts.
    - Input objective:
        - Shape of the input (can be anything; the ball is the easiest, any more complex shape is more difficult to solve (however it's more interesting too))
            - how shape is randomized (shape of the input should be held more or less constant throughout the run). This is a bullet point list.
        - Central position and position randomization margins (e.g. x:50, y:80, z:20, x_variation: 20, y_variation: 20, z_variation: 10)
    - Where the input objective is located (coordinates + randomization),
    - Objectives location:
        - A "forbid" objectives as a set of approximate AABB coordinates,
        - A "goal" objective as a single AABB coordinate
The agents' file must correspond to roughly the structure detailed above, with automatic checks in place.
2. A `todo.md` TODO list from the planner.
3. A draft of `objectives.yaml` with rough values filled in.
4. A draft of `assembly_definition.yaml` with per-part DOFs/control in `final_assembly.parts` (benchmark-local; not handed to engineering).
<!-- Note: it may be interesting that the implementer could try a few "approaches" on how to reduce costs without actually editing CAD, and would get fast response for cost by just editing YAML. However, it will almost by definition deviate from the plan. -->

The agent must make sure that the geometric plan is valid, the input objective does not interfere with anything (and goal objectives are not obstruted), that there is proper randomization, etc., no object coincides with each other.
If the user provides explicit benchmark objective overrides (for example `max_unit_cost`, `max_weight`, `target_quantity`), the planner preserves them semantically in `objectives.yaml` and must not silently mutate those constraints.

## Benchmark Generator with Engineer handover

The Engineer agent(s) (for whom the first point of access is the Planner/lead engineer) have access to meshes and a exact reconstruction of the environment as a starting point to their build123d scene, however they can not modify/move it from their build123d scene. In fact, we validate for the fact that the engineer wouldn't move it or changed it (validating for changing it via hashing) - in both MJCF and build123d.

Additionally, the engineering agent will be supplied with renders for preview automatically rendered from 24 views. (Clockwise, 8 pictures, on 30 degrees up or down (configurable)).

The engineer will also receive YAML files with:
    1. Exact positions (boundaries) of objectives.
    2. "Runtime" randomization, i.e. the randomization of the starting positions this environment will use. Note that it's different from the "static" randomization which is stronger.
    3. Maximum prices and weight. Prices should be estimated by the planner to be relatively challenging but feasible by the planner (feasible but challenging related to *our* pricing sheet, as defined in planner.md). I suggest initially giving 50% safety margin in pricing and weight.
    Note that the maximum price and weight are also set by the planner later internally. However, the planner sets their own constraints *under* the maximum price. Here the "maximum prices and weight" are a "customer-specified price and weight" (the "customer" being the benchmark generator), and the planner price and weight are their own price and weight.
    <!-- (in future work) Later on, we will challenge the agent to optimize its previous result. It would have to beat its own solution, by, say, 15%.  -->

The positions of objectives (including a build zone) and runtime randomization are in `objectives.yaml`. The benchmark planner's `assembly_definition.yaml` stays in the benchmark planner scope and is not handed over to engineering.

### A benchmark is reused multiple times

Notably, the benchmarks are randomized and reused multiple times under different variations. The Engineering agent only receives a "runtime" randomization - (as in the list of the relevant heading) currently only a (relatively) small jitter in the environment.

### What if the benchmark (planner/coder) agent set the price too low?

For now, nothing. I'll filter it out via SQL or similar later and make a "price discussion" between agents - probably. Or a human-in-the-loop method. Anyway, this is a non-priority item for now.

## Engineer Planner and "Coder" interaction

Engineering handoff includes two review gates:

1. Planner gate (`ENGINEER_PLAN_REVIEWER`): validates planner artifacts before coder entry.
2. Execution gate (`ENGINEER_EXECUTION_REVIEWER`): validates latest implementation handoff after validation/simulation success.

Engineer sends four files to the coder agent who has to implement the plan:

1. A `plan.md` file The plan.md is a structured document (much like the benchmark generator plan) outlining:
2. A stripped down `objectives.yaml` file, except the max price, weight are set by the planner now - and they are under the max weight set by the user/benchmark generator.
3. A `todo.md` TODO-list.
4. A `assembly_definition.yaml` file with per-part pricing inputs, `final_assembly` structure, and assembly totals produced by `validate_costing_and_price.py`.

Planner gate requirements (`ENGINEER_PLAN_REVIEWER` / coder entry contract):

- Source of truth contract: `ENGINEER_PLANNER_HANDOFF_ARTIFACTS` in node-entry validation.
- Required artifacts: `plan.md`, `todo.md`, `objectives.yaml`, `assembly_definition.yaml`
- Reviewer-stage manifest: `.manifests/engineering_plan_review_manifest.json` (planner handoff materialization for the plan-review stage)
- Plan reviewer responsibilities:
  - Reject unsupported/invented system components or mechanisms.
  - Reject inconsistent, infeasible, ambiguous, or incomplete plans.
  - Reject excessive DOFs; motion metadata must be minimal and mechanism-necessary, not convenience-driven.
  - Keep cost/weight target scrutiny as a mandatory realism check.
  - Optional future work: propose weight/cost optimization opportunities.

### `plan.md` structure for the engineering plan

```markdown
# Engineering Plan
## 1. Solution Overview
A brief description of the approach to solve the benchmark objective (e.g., "A gravity-fed ramp with a deflector plate to redirect the ball into the goal bin").
## 2. Parts List
For each part:
- **Part name**: e.g., `ramp_main`
- **Function**: What role it plays
- **Geometry**: An overview of geometry
    - **Mating points** with other connectors (list)
- **Manufacturing method**: CNC / Injection Molding / 3D Print
- **Material**: e.g., `aluminum-6061`, `abs-plastic`
- **Estimated dimensions**: Rough sizing
## 3. Assembly Strategy
- How parts connect (fasteners, etc.) <!--e.g. bearings in the future -->
- Mounting points to environment (if any drilling/attachment is allowed)
<!-- - Order of assembly --> 
<!-- Order of assembly is partially unnecessary because we kind of work in CAD. However, it's a good thing to think of. -->
## 4. Cost & Weight Budget
- `max_unit_cost`: $X (from objectives.yaml, planner's allocation)
- `max_weight`: Y kg
- Assembly breakdown per part
## 5. Risk Assessment
- Potential failure modes (e.g., "ball bounces off ramp edge")
- Mitigations for each risk
- Runtime randomization considerations (position jitter handling)
<!-- ## 6. Build123d Strategy
- CSG vs sketch-based approach
- Key operations (fillets, chamfers, patterns)
- Reference skills to consult -->
<!-- Note: the planner explicitly doesn't specify the CAD approach. It doesn't need to think about plans, it's about the geometry. -->
```

### `objectives.yaml`

`objectives.yaml` is a central data exchange object to the system. To centralize its structure:

```yaml
# =============================================================================
# OBJECTIVES.YAML - Your Task Definition
# =============================================================================
# This file defines WHAT you must achieve. Read it carefully before planning.
#
# YOUR MISSION: Guide the `moved_object` into the `goal_zone` while:
#   1. Staying WITHIN the `build_zone` (you cannot build outside it)
#   2. AVOIDING all `forbid_zones` (contact = failure)
#   3. Respecting `max_unit_cost` and `max_weight` constraints
#
# The environment geometry in this file is READ-ONLY. Engineering assembly
# motion metadata is stored under engineering assembly_definition.yaml
# final_assembly.parts and is also READ-ONLY once written.
# =============================================================================

objectives:
  # SUCCESS: The moved_object's center enters this volume
  goal_zone:
    min: [x_min, y_min, z_min]
    max: [x_max, y_max, z_max]

  # FAILURE: Any contact with these volumes fails the simulation
  forbid_zones:
    - name: "obstacle_collision_zone"
      min: [x1, y1, z1]
      max: [x2, y2, z2]
    # Additional forbid zones may be listed here

  # CONSTRAINT: Your entire design MUST fit within these bounds
  # Parts placed outside will fail validation
  build_zone:
    min: [x, y, z]
    max: [x, y, z]

# Hard simulation boundaries - objects leaving this volume = failure
simulation_bounds:
  min: [-50, -50, 0]
  max: [50, 50, 100]

# -----------------------------------------------------------------------------
# THE OBJECT YOU MUST DELIVER
# -----------------------------------------------------------------------------
# This object spawns at `start_position` (with runtime jitter applied).
# Your design must reliably guide it to the goal_zone.
moved_object:
  label: "projectile_ball"
  shape: "sphere"
  # Static randomization: shape varies between benchmark runs
  static_randomization:
    radius: [5, 10]  # [min, max] - actual value chosen per benchmark variant
  start_position: [x, y, z]
  # Runtime jitter: small position variation per simulation run
  # Your solution must handle ALL positions within this range
  runtime_jitter: [2, 2, 1]  # [±x, ±y, ±z] mm

# -----------------------------------------------------------------------------
# YOUR CONSTRAINTS
# -----------------------------------------------------------------------------
# These are challenging but achievable. Exceeding them = rejection.
constraints:
  max_unit_cost: 50.00  # USD - total cost of your manufactured parts
  max_weight: 1.2       # kg - total weight of your design

# Randomization metadata (for reproducibility)
randomization:
  static_variation_id: "v1.2"  # Which static variant this is
  runtime_jitter_enabled: true
```

<!-- Note: we are using metric units and degrees. -->

### `assembly_definition.yaml`

To reduce cost guessing, the Engineering Planner outputs a machine-readable estimate file that also serves as an assembly plan: it captures all pricing inputs per part plus the assembly structure used to derive quantities, reuse, and motion metadata.

Expected flow:

1. Planner drafts entries for all planned manufactured parts and COTS components.
2. Planner defines `final_assembly` (subassemblies, part membership, joints, and per-part motion metadata like `dofs`/`control`); under the hood we:
    - Calculate as much as possible to prevent the planner from needing to think (e.g.: cooling time in injection molding is autocalculated from wall thickness, 3d print time is autocalculated from volume, setup time is autocalculated etc.)
    - Estimate part reuse - if the part/subassembly is reused, unit costs go down as per manufacturing rules (making 2 equal parts is cheaper than making 1 due to economics of scale).
3. Planner runs `skills/manufacturing-knowledge/scripts/validate_and_price.py`.
    - The script validates schema consistency and computes assembly totals.
    - The script auto-populates the unit cost and weight to the objectives.yaml file (unless the file is corrupted).
4. If totals exceed `max_unit_cost` (or other numeric constraints), planner must re-plan before handoff.
5. Planner writes planner-owned constraints in `objectives.yaml` using validated assembly totals, under benchmark/customer caps.

Minimum motion metadata fields inside `final_assembly.parts` entries:

- `dofs`
- For motorized parts: `control.mode`, plus required control params (e.g. `speed`, `frequency`) per mode
- DOF minimization contract:
  - `dofs: []` is the default for non-moving parts.
  - Non-empty `dofs` must be explicitly justified in `plan.md` (`## 3. Assembly Strategy` or `## 5. Risk Assessment`) with objective-linked rationale.
  - Unjustified or excessive DOF assignments are plan-review rejection criteria.

Minimum per-manufactured-part fields:

- `part_name`, `part_id`, `description`, `manufacturing_method`, `material_id`
- `part_volume_mm3`
- method-specific costing inputs:
  - CNC: `stock_bbox_mm`, `stock_volume_mm3`, `removed_volume_mm3`
  - Injection molding / 3D print: required process-specific volume/thickness/time fields per validator contract
<!-- - `estimated_unit_cost_usd` - auto-calculated. The planner does not need to calculate each. However, it may be populated automatically by the script if it runs successfully? Again, the goal is to offload agent work and guessing (for performance reasons). -->
- `pricing_notes <!-- User review - maybe. Maybe for "confidence" scores or similar. -->

Minimum per-COTS-part fields:

- `part_id`, `manufacturer`, `unit_cost_usd`, `source`

Note: I will restate: any field that can be autopopulated is autopopulated. E.g. here the mandatory fields are only `part_id`, `manufacturer`, `source`, the `price` can be pulled from DB (errors are not triggered during validation too, however if it mismatches, the fields must be overwritten and the right price used.)

Required assembly fields:

- `final_assembly` containing subassemblies/parts/joints
- each part entry in `final_assembly.parts` includes `dofs`; moving motorized entries include `control`
- repeated part references are allowed and used by pricing logic to compute quantity effects

```yaml
version: "1.0"
units: #pre-populated in template.
  length: "mm"
  volume: "mm3"
  mass: "g"
  currency: "USD"
# constraints: # user review - no, unit constraints are written in objectives.yaml. 
#   benchmark_max_unit_cost_usd: 50.0
#   benchmark_max_weight_kg: 1.2
#   planner_target_max_unit_cost_usd: 34.0
#   planner_target_max_weight_kg: 0.9
manufactured_parts:
  - part_name: "ramp_main"
    part_id: "ramp_main_v1"
    description: "Primary sloped ramp for ball redirection"
    manufacturing_method: "CNC"
    material_id: "aluminum-6061"
    part_volume_mm3: 182340.0
    stock_bbox_mm: {x: 220.0, y: 120.0, z: 12.0}
    stock_volume_mm3: 316800.0
    # removed_volume_mm3: 134460.0 # no need to calculate, have script calculate it.
    estimated_unit_cost_usd: 18.70 # estimated automatically
    pricing_notes: "3-axis; no undercuts"
  - part_name: "guide_clip"
    part_id: "guide_clip_v1"
    description: "Guide clip holding edge trajectory"
    manufacturing_method: "INJECTION_MOLDING"
    material_id: "abs-plastic"
    part_volume_mm3: 24100.0
    wall_thickness_mm: 2.0
    cooling_time_s_estimate: 11.0
    estimated_unit_cost_usd: 0.82
    pricing_notes: "tooling amortized at target quantity"
cots_parts:
  - part_id: "M5x16-912-A2"
    manufacturer: "ISO"
    unit_cost_usd: 0.09 # auto-calculated.
    source: "parts.db"
  # user note: cots parts must be enforced to exist in the subassemblies, at least 1. Else why would it be here?
  # user note 2: reminder: search for COTS parts is performed by a subagent
final_assembly:
  - subassembly_id: "frame_and_ramp"
    parts:
      - ramp_main_v1:
          dofs: []
      - guide_clip_v1:
          dofs: []
      - guide_clip_v1:
          dofs: []
      - feeder_motor_env_v1:
          dofs: ["rotate_z"] # Degrees of freedom: rotate_x/y/z, slide_x/y/z
          control:
            mode: "sinusoidal" # Options: constant, sinusoidal, on_off
            speed: 1.0         # rad/s (for rotate) or units/s (for slide)
            frequency: 0.5     # Hz - for sinusoidal mode
      - passive_slider_env_v1:
          dofs: ["slide_y"]
    joints:
      - joint_id: "j1"
        parts: ["ramp_main_v1", "guide_clip_v1"]
        type: "fastener_joint"
totals:
  estimated_unit_cost_usd: 31.46
  estimated_weight_g: 742.0
  estimate_confidence: "medium"
```

Validation requirement:

- Submission is blocked if `assembly_definition.yaml` is missing, malformed, still template-like, fails `validate_costing_and_price.py`, or contains non-numeric values for required numeric fields (doesn't match schema in general)

## Coder and Execution Reviewer interaction

The execution reviewer (`ENGINEER_EXECUTION_REVIEWER`) is a post-validation/post-simulation stage.

1. Entry is blocked unless latest-revision reviewer handoff artifacts are valid (`script.py`, `validation_results.json`, `simulation_result.json`, `.manifests/engineering_execution_review_manifest.json`).
   - Source of truth contracts: `REVIEWER_HANDOFF_ARTIFACTS` + execution-review custom handover check in node-entry validation (using reviewer-scoped manifest filenames from this document).
2. The execution reviewer has read-only access to implementation and evidence files, plus write/edit only under `reviews/review-round-[round number]/`.
3. Primary review is robustness and realism: verify successful behavior is not flaky across runtime randomization and is likely repeatable.
4. Verify execution follows the approved plan or clearly justified deltas, including planned DOF limits.
5. Optional code-quality review is secondary and should only block for concrete correctness/safety risks.

The goal is to persist the reviews into a persistent file which the agent can reference at any time (alongside previous reviews), and see it only once; and to avoid plumbing to route "reviews" text when required.

Notably the Engineer will at this point already have passed the manufacturability constraint, cost constraint, and others.
<!-- and others - I need to ideate/remember what else it should review for.  -->

The reviews will also come with the following YAML frontmatter:

```yaml
decision: APPROVED # [APPROVED, REJECTED, REJECT_PLAN, REJECT_CODE, CONFIRM_PLAN_REFUSAL, REJECT_PLAN_REFUSAL]
comments: [
   # short commentary on issues, as a array.
   # Up to 100 chars per issue.
]
```

As usual, the reviews will be strictly typed.

## Clarification - definition of constraints in planning

1. Constraints are set first at the application level; e.g. the timeout of simulation is always 30 seconds
2. Then the benchmark planner/implementer set a more realistic constraint for them (e.g., they set a max cost, max weight for the simulation, similarly to how a "customer" would do it for an engineering company)
3. The engineering planner can set an even lower constraint. to force the engineering implementer to think on how to achieve a certain goal cost-effectively. The implementer won't pass the cost metric until it is done.

## Clarification: agents outputs will *never* be parsed via text heuristics.

Code is not to fall back to heuristic text parsing from agents; except of tools.

If the agent is to record structured output or an enum, it'll record it as an enum.

## Agents hard fail cases

The agents will fail on the following cases:
1. Timeout (configurable per agent, mostly 300s)
2. Max turns reached (for example, 150 turns (realistic with modern LLMs))
3. Exceeded some credits usage (e.g. 400k tokens input and output).
4. LLM loop-detection guard triggered:
    - If the same normalized failure fingerprint repeats for `N` consecutive retries, fail the stage.
    - Fingerprint is based on exception class/message plus latest LM output preview (or empty preview).
    - This guard is local to the node runtime and does not depend on DB episode counters, so local/CLI runs also terminate deterministically.

The user is able to have the agent continue for another number of turns.

All settings are configurable per-agent in @config/agents_config.yaml and should be rather permissive.

## Agents retry as much as possible

Clarification: models will retry until their quota is exceeded, as in one cases in the parent section. Until then, whatever execution environment (meaning, a failed tool call, an invalid script they've written); they must retry. 
Retries are still fail-closed by loop-detection and timeout guards; repeated identical failures must not spin forever.

## Steerability

Agents must be steered by the engineer towards the right solution. In essense, the engineer (user), seeing incorrect traces, can send a prompt to edit the prompt.
Just as with coding agents.

In the simplest example, if the plan is incorrect, the engineer will intervene and tell the model what to do differently.

The work here is the UI, the backend processing and prompt optimization.

## Steerability with exact pointing

It's desired that agents are able to understand the exact face or volume that the engineer wants to fix. The engineer, in the UI, will select the set of faces, or edges, or corners, and "attach" them to the prompt, similar to how a programmer can attach faulty logic to code. Thus, the agents will get the name of the part (or set of) they need to edit, and whatever features on them.

If a user selects a particular feature, the system automatically determines the best angle to view the feature (one from 8 of isometric views) and render the image with this view selected part more brightly.
If multiple features are selected, showcase one view.

The agent will also receive selection metadata (grouped per feature type):

- With faces:
    1. Which face number this is in particular (notably, the agent should not try to use the exact face number in the code, rather, they will select it using proper methodology).
    2. Where the face is located (center) and its normal.
- With edges:
  - Edge center and edge direction (if arc, specify that it is an arc)
- With vertices:
  - position

In addition, the user will be able to select macro features like parts and subassemblies. There will be a toggle in frontend that would allow switching between primitives (faces, vertices, edges), and parts and subassemblies (three buttons total). If the user selects a part or a subassembly, it gets attached to the prompt.

The agents will receive the prompt in a yaml-like format:

```yaml
Selected faces:
   face_1:
      center: [1,2,3]
      normal: ...
Selected vertices:
    position: [10,10,10]
Selected subassemblies:
    wheel_assembly:
        position: [20,20,20]
        parts_list: [part names] # don't spam models with logs in here.
        
```

Notably, its not strictly a YAML, it's just a similar representation of it in the prompt.

<!-- Likely, if there are a large number of volumes, don't send them all at once? -->

## Steering code directly

In addition, the engineers should be able to, just as in common software agentic environments, select some code and ask to it. It should be in the format of:
The user will see it as:

```md
@assembly_definition.yaml:136-139
```

While the LLM would see it as:

```md
@path/other_path/assembly_definition.yaml:136-139
```

The model should then, in 90% of cases at least, view the actual lines.

## Pointing in the chat

Because we have a bill of materials/part tree, it should be very straightforward to "@-mention" the part or subassembly that one wants to edit.

## Steerability - Other

1. Ideally, agents would have standard per-user memory sets.
2. The model should then, in 90% of cases at least, view the actual lines where the part, or direct code mention or whatever is defined; meaning that it doesn't ignore the user's output.

<!-- per-org, per-project memory is a TODO later.-->

## Hard constraints on Agent output

We should give agents less chances to fail (our) system, and fail the execution of our tasks.
While we don't enforce strict Pydantic schemas, we effectively validate all inputs as such.

E.g., if a part doesn't have a `material` custom property on it, we fail the script; we don't substitute to a default Aluminum.
