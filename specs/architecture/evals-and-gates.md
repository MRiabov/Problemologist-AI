# Agent Evaluations

## Scope summary

- Primary focus: evaluation architecture and quality gates for agent behavior.
- Defines fast/medium/slow evaluation tiers and measurable acceptance thresholds.
- Specifies terminal-state/failure classification requirements and fail-closed gating behavior.
- Use this file when adding or modifying eval criteria, gate logic, or episode terminalization policy.

Evaluations are treated as a first-class architecture in this application. In fact our work on manufacturability validation, code linting, simulation is just a tool for evaluation.

Seeded-eval preflight contract:

1. Seeded node-entry preflight is not allowed to be file-presence-only.
2. Preflight must schema-validate every present schema-backed handoff artifact before execution continues.
3. This includes all typed YAML/JSON handoff artifacts relevant to the current seed workspace (for example `benchmark_definition.yaml`, `assembly_definition.yaml`, `benchmark_assembly_definition.yaml`, `validation_results.json`, `simulation_result.json`, and reviewer/plan-review manifests under `.manifests/`).
4. Shared template files from `shared/agent_templates/common/` are expanded before validation and count as seed material, not as prompt-only rows.
5. For planner handoff artifacts, preflight must additionally run cross-contract semantic checks (engineering attachment/cost checks and benchmark planner semantic checks) equivalent to the runtime handoff gates.
6. If any schema-backed handoff artifact, shared-template materialization, or cross-contract check is invalid, preflight must fail closed and the eval run must not proceed into node execution.
7. Validation ownership remains in controller node-entry validation; evals are orchestration callers and must not duplicate artifact-schema or handoff-semantic validators locally.

<!-- To build great agents, we need agent needs a great evaluation pipelines.
We need 10 at least test prompts for each agent (primary) agent subtype - 3 for benchmark (Planner, CAD_agent, Reviewer), 3 for engineering (Planner, CAD_agent, Reviewer).

We will also need evaluations for an agent. -->

## Evaluations architecture

We need evaluation criteria that would be not only functional, but tied to numbers and count of tool calls/reviews from LLM-as-judge.

Bad example: specific as "Markdown would be valid."
Good example: Testing of markdown for structural validity be successful in 95% of cases.

## Reward config structure

`config/reward_config.yaml` is split into three metric classes per agent when reward shaping is used:

1. `hard_checks`: deterministic or directly computed checks such as artifact presence, schema validity, geometry validity, pricing caps, manufacturability, and simulation outcomes that do not require an LLM judge.
2. `judge_evaluation`: reviewer-driven or seeded-rubric metrics such as reviewer acceptance, decision correctness, actionability, and feedback-response quality.
3. `judge_evaluation.checklist`: weighted per-checklist-item reward terms that deliberately duplicate the canonical reviewer checklist. Each checklist key is rewarded individually rather than via one aggregate checklist score.
4. `milestones`: remaining non-review objective rewards that are neither pure fail-closed gates nor LLM-as-a-judge outputs. This bucket is allowed for backward compatibility and for downstream objective outcomes such as "planner led to successful implementation".

Rules:

1. Weights across `hard_checks`, `judge_evaluation`, `judge_evaluation.checklist`, and `milestones` sum to `1.0` per agent.
2. Reviewer-heavy agents should score canonical checklist keys individually rather than hiding checklist quality behind one aggregate checklist scalar.
3. Planner and coder agents may intentionally duplicate reviewer checklist keys in reward config when those keys represent the real quality gates for acceptance.
4. Coder agents should carry explicit reward for handling reviewer feedback, including partial credit for fixing valid failed checklist items and resisting incorrect reviewer requests in mixed-quality feedback evals.

## Multi-level evaluations architecture

We should be able to test evaluations on multiple tiers, specifically:

### Fast

1. Testing of markdown validity (for planners, TODO lists, reviewers) | 95% of cases on the first prompt.
2. Testing of code validity, ruff/pyright checks - 90% of cases on an average tool call
   - Including, specifically, build123d code. So we would generate a valid build123d code in 95% of the cases.
3. Testing that output YAML is valid in 95% of cases.
   - In `benchmark_definition.yaml`, the objectives do not intersect, specified objects do not intersect
   - In `benchmark_definition.yaml`, the objectives are in bounds of the model.
4. Given a prompt, agents read the necessary skill documents (as instructed).
   - For each agent.

### Medium evals

#### Medium evals - Engineer evaluations

##### Medium evals - Engineer Planner evaluations

<!-- We probably want to split this into further three sections below. -->

1. Given a prompt, an Engineering Planner uses appropriate components to solve a problem, and creates plans that are physically achievable, does not have intersecting models, and actually passes.
   - Validation:
     1. YAML
     2. LLM-as-a-judge (Plan Reviewer).
2. Given a prompt, the Engineering Planner uses accurate prices from the catalog and doesn't "come up" with prices.
3. Given a prompt, an Engineering Planner does not generate plans for features outside of a build zone.
4. Given a prompt, an Engineering Planner plans for a solution that is equal or lower to the `max_unit_cost`, `max_weight`, as well as other numeric constraints in 95% of cases.
5. Given a prompt, the Engineering Planner produces plans with correct units (e.g. metric or US customary units in 95% of cases).
6. If render images exist for the current revision, the Engineering Planner inspects at least the config-driven minimum number of images through `inspect_media(...)` before finishing. Current production policy is `min_images=1`.

##### Medium evals - Engineering Coder

1. Given a plan, the engineer will pass manufacturability checks in 70% during validation and pricing tool call. (note: lower than during submission because this is explicitly not submitting but validating the tool call).
2. Given a plan, the engineer will pass manufacturability checks in 90% of tool calls when expected during simulation or submission (when they *expect* that it will definitely pass). On second and third attempts this will improve to 95% and 97% respectively.
3. The engineer adheres to benchmark-defined environment attachment policy: benchmark-owned parts may be drillable or non-drillable, attachment is optional rather than mandatory, and any declared drilling in `assembly_definition.yaml.environment_drill_operations` must remain within the benchmark-side hole-count, size, and depth limits.
4. The engineer, after simulation, would interpret the simulation results correctly [...] (how to enforce it? they wouldn't always need to view the results, they can use final positions table that should be output or text too.)
5. The engineer will prefer CSG over sketches in 70% of the cases (soft requirement, but it makes it actually easier to build with code).
6. If render images exist for the current revision, the Engineering Coder inspects at least the config-driven minimum number of images through `inspect_media(...)` before finishing. Current production policy is `min_images=1`.
7. Given reviewer feedback from the prior round, the Engineering Coder resolves at least 70% of failed reviewer checklist items by the next round without introducing new blocker regressions.
8. Given mixed-quality or non-ideal reviewer feedback, the Engineering Coder follows valid failed checklist items and resists incorrect or non-applicable requests in at least 70% of seeded cases.

##### Medium evals - Engineering Plan Reviewer

1. Given a viewed plan package, the Plan Reviewer would correctly identify plan issues in at least 70% of cases.
   - Correctness: given a valid plan with an issue introduced by another LLM, a reviewer would spot the issue and the issue matches independent validation.
2. Given a viewed plan package, the Plan Reviewer rejects unsupported/invented components or mechanisms in at least 97% of cases.
3. Given a viewed plan package, the Plan Reviewer rejects inconsistent, infeasible, ambiguous, or incomplete plans in at least 90% of cases.
4. Given a viewed plan package, the Plan Reviewer rejects excessive/unjustified DOFs in at least 90% of seeded over-actuation cases.
5. Price/weight realism gate: if target budgets are set unrealistically, the Plan Reviewer rejects and requests concrete planner fixes.
6. Reviewer efficacy: plan-review feedback should lead to a corrected plan in at least 60% of failed first submissions.
7. If render images exist for the current revision, the Plan Reviewer inspects at least the config-driven minimum number of images through `inspect_media(...)` before approval. Current production policy is `min_images=1`.
8. The Plan Reviewer writes a stage-canonical checklist in the reviewer comments YAML, and the checklist keys/values match the seeded ground truth in at least 95% of cases.

##### Medium evals - Engineering Execution Reviewer

01. Given a viewed model with renders available, the Execution Reviewer correctly navigates output evidence and checks at least the config-driven minimum number of images through `inspect_media(...)` before decision. Current production policy is `min_images=1`.
02. Given a successful handoff package, the Execution Reviewer identifies plan-vs-execution deviations and robustness risks in at least 70% of cases.
03. Given a successful simulation result, the Execution Reviewer rejects flaky/non-robust solutions in at least 70% of known flaky cases.
04. Given a successful simulation result, the Execution Reviewer flags over-actuated solutions (excessive moving axes/parts) in at least 80% of seeded over-actuation cases.
05. Manufacturability awareness: the Execution Reviewer reports only changes appropriate to the manufacturing method in 97% of cases.
06. Given a viewed model, the Execution Reviewer requests cheaper/lighter improvements where feasible in at least 15% of cases.
07. Toolkit usage and diversity; the model will use:
    - fasteners in at least in 70% of builds
    - Motors in at least 20% of the builds
    - bearings in at least 10% of the builds
    - COTS search will be executed in at least 50% of builds (you need to look for specific version of fasteners, motors, etc.)
    - Other tools will be used at least reasonably often, or at least sometimes (3%?)
      (this requirement is more so for prompt debugging - that we don't miss including something into the prompt/skill sections.)
08. The model would be able to execute a search (or use a subagent) in COTS
09. If render evidence exists, approval is valid only when the reviewer used the dedicated media-inspection tool rather than text-only file inspection.
10. The Execution Reviewer writes a stage-canonical checklist in the reviewer comments YAML, and the checklist keys/values match the seeded ground truth in at least 95% of cases.

<!-- Future: Given a prompt, the Engineering Planner doesn't use components that are out of stock -->

<!-- FIXME: Underspec: we don't define coordinate system starting point.I.e. is the center defined as (0,0,0)? Else how do we define it?
Proposal: normalize the simulation to the center bottom of the build zone. So the bottom center of the simulation would be 0,0,0; whatever is under it would be -z, and everything would be normalized to it. I expect it'll help agents adjust the build. -->

#### Medium evals - Benchmark Generator

##### Medium evals - Benchmark Generator Planner

1. Given a prompt, a Benchmark Planner generates a plan that, upon valid scrutiny by the `Benchmark Plan Reviewer`, passes in 80% of cases.
2. Given a prompt, a benchmark generator planner generates a plan that would have unimpeded objectives (objectives obstructed by obstacles by no more than 35%) in 97% of the cases (calculated by volume - shouldn't be obstructed.)
3. Given a prompt, the benchmark generator will produce plans for various manufacturing quantities (prototype \<5, small volume \<100, mass-manufacturing - 3000)
4. Given a prompt, we will include to the solution proposed by the prompt:
   - will include correct:
     - Quantity
     - Max weight,
     - Max cost
     - And other numerical parameters specified in benchmark_definition.yaml.
5. The benchmark generator would be able to predict the price and weight the engineer will solve the solution in the range of 80-120% (with 20% error) of the final price in 80% of the cases, within 50-150% in 97% of cases (this is the standard price, not the "safe" price)
6. Benchmark-side motion is judged on explicit contract and evidence, not on DOF minimization; fully free benchmark fixtures are valid when the handoff declares them and the evidence matches.

##### Medium evals - Benchmark Plan Reviewer

1. Given a benchmark planner handoff package, the Benchmark Plan Reviewer correctly identifies nonexistent benchmark objects or inconsistent object references across `plan.md`, `benchmark_definition.yaml`, and benchmark-owned `benchmark_assembly_definition.yaml` in at least 90% of seeded cases.
2. Given a benchmark planner handoff package, the Benchmark Plan Reviewer rejects ambiguous, infeasible, or incomplete plans in at least 85% of seeded bad-plan cases.
3. Given a benchmark planner handoff package with renders available, the Benchmark Plan Reviewer inspects at least the config-driven minimum number of images through the dedicated media-inspection tool before approval. Current production policy is `min_images=1`.
4. Reviewer efficacy: benchmark plan-review feedback should lead to a corrected planner handoff in at least 60% of failed first submissions.
5. Given a benchmark planner handoff package with moving benchmark-owned fixtures, the Benchmark Plan Reviewer rejects missing motion-visible handoff data in at least 90% of seeded cases.
6. Given a benchmark planner handoff package with moving benchmark-owned fixtures, the Benchmark Plan Reviewer rejects missing, contradictory, or unsupported benchmark-side motion declarations in at least 85% of seeded bad-motion cases.
7. The Benchmark Plan Reviewer writes a stage-canonical checklist in the reviewer comments YAML, and the checklist keys/values match the seeded ground truth in at least 95% of cases.

##### Medium evals - Benchmark Coder

1. Given a plan, a Benchmark Coder generates a benchmark that is physically valid, uses features that actually exist in our pipeline (does correct constraints, etc) in 95% of cases after 30 turns, and 3 submissions to reviewer.

- First submission - 10 tool calls, 70% pass,
- Second submission - 20 tool calls, 85% pass,
- Third submission - 30 tool calls, 95% pass.

2. Given reviewer feedback from the prior round, a Benchmark Coder resolves at least 70% of failed reviewer checklist items by the next round without introducing new blocker regressions.
3. Given mixed-quality or non-ideal reviewer feedback, a Benchmark Coder follows valid failed checklist items and resists incorrect or non-applicable requests in at least 70% of seeded cases.

##### Medium evals - Benchmark Reviewer

1. Given a benchmark package with renders available, the Benchmark Reviewer inspects at least the config-driven minimum number of images through the dedicated media-inspection tool before approval. Current production policy is `min_images=1`.
2. Given a benchmark package with misleading text summaries but invalid visual geometry, the Benchmark Reviewer catches the issue in at least 70% of cases.
3. Listing `renders/` without actual media inspection does not satisfy reviewer-evidence criteria.
4. Given a moving benchmark with simulation video available, the Benchmark Reviewer inspects the latest dynamic evidence before approval in at least 95% of cases.
5. Given a moving benchmark whose declared fixture motion is missing, contradictory, or not reflected in simulation evidence, the Benchmark Reviewer rejects it in at least 80% of seeded cases.
6. Given a moving benchmark whose declared fixture behavior does not match observed simulation behavior, the Benchmark Reviewer rejects it in at least 85% of seeded mismatch cases.
7. The Benchmark Reviewer writes a stage-canonical checklist in the reviewer comments YAML, and the checklist keys/values match the seeded ground truth in at least 95% of cases.

All visual-inspection evals above are config-driven rather than prompt-only. The source of truth for required roles and image-count thresholds is `config/agents_config.yaml` (`visual_inspection.required`, `visual_inspection.min_images`, `visual_inspection.reminder_interval`), and the requirement is conditional on actual render-image availability for the current node/revision.

#### Medium evals - Skill Learning Agent (Async)

1. **Validity**: Generated skills (`SKILL.md`) are valid markdown/YAML and adhere to the skill schema in 100% of cases. (fast)
2. **Utility**: Generated skills are referenced/used by other agents in subsequent similar tasks (requires long-term tracking). (long-term)
3. **Non-duplication**: Generated skills do not duplicate existing skills (upon inspecting git changes after 30 turns- the skill rows aren't churned) (long-term) (not exactly an eval, but a tracking logic).
4. No overwrite: Skills aren't overwritten from scratch in 100% of cases
   - Skills can not be overwritten for more than 5 lines, to prevent catastrophic overwriting.

#### Journaling

1. Struggle detection: The agent detects and logs entries for 90% of "struggles" (failed tool calls > 4) detected in the logs.
2. **Linkage**: 97% journal entries are correctly linked to a unique problem/observation ID.

### Slow (essentially, production tasks)

#### Slow evals - Engineer

##### End-to-end eval

1. Given a benchmark and a stadard prompt, the engineer builds a build123d model that is valid and reaches the goal within 30 tool calls and three simulation attempts.
   - First submission - 10 tool calls, 70% pass,
   - Second submission - 20 tool calls, 85% pass,
   - Third submission - 30 tool calls, 95% pass.
2. The Engineer (Planner+Engineering Coder) agents would not create examples that would break parts in 90% of the cases (can estimate if it (only motors for now, soft bodies in the future) will break using a formula).

##### Engineering Planner

##### Engineering Coder

1. Given a plan, an Engineer builds a build123d model that is valid and reaches the goal within 30 tool calls and three simulation attempts.
   - First submission - 10 tool calls, 70% pass,
   - Second submission - 20 tool calls, 85% pass,
   - Third submission - 30 tool calls, 95% pass.
2. Robustness test: If at least 1 simulation passes, at least 70% of "runtime jitter" variations pass too (i.e. the solution is not "flaky" that only works for one specific seed, but is mechanically robust).
3. Given a plan and `benchmark_definition.yaml`, the Engineer will not try to submit a solution that is more unit expensive or heavy than unit tests set by the planner and will:
   - Cheaper than the max price: 80% on first attempt, 90% after the first failure, 95% after the second failure.
   - Small error: 0-20% more expensive - In 10% on first attempt, 5% after the first error, 3% after the second failure (only small failyre, not including large error).
   - Large error: >20% more expensive - In 10% of first failure, 5% on second failure, 3% after after the second failure.
4. Given reviewer feedback from the prior round, the Engineer improves the solution by flipping failed reviewer checklist items to passing in at least 70% of seeded revision loops while keeping previously passing checklist items stable in at least 90% of cases.

##### Reviewer

Plans not rejected by

#### Slow evals - Benchmark generator

##### Slow evals - Engineering Coder

1. Given a correct plan, a benchmark generator builds a benchmark that an engineer would be able to solve within 30 turns in 70% of cases.
2. Quality of Reasoning: The reasoning traces for benchmark generation are coherent and logically sound in >90% of cases (important for dataset quality).

### Integration/post-processing evals (more than one episode)

There are some episodes which can be take multiple episodes to run.

1. Plan-to-CAD fidelity: after the CAD was successfully reconstructed into a successful solution, generate a plan, and reconstruct again. The result (volume) should be approximately the same (80%).

<!-- That said, I'm not sure if it's necessary. -->

1. Robustness to randomization - having solved 1 seed from a randomization batch, an engineer would be statistically 50% more likely to other episodes in the batch relative to average performance (would fail 50% less).
2. If a reviewer said "this is acceptably the cheapest we can get" and then the model got cheaper by 50% unit cost at the same quantity (during subsequent planning) - the reviewer in fact didn't find an optimal quantity, and that shouldn't happen... (in which cases out of which?)
