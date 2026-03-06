# Agent Evaluations

Evaluations are treated as a first-class architecture in this application. In fact our work on manufacturability validation, code linting, simulation is just a tool for evaluation.

<!-- To build great agents, we need agent needs a great evaluation pipelines.
We need 10 at least test prompts for each agent (primary) agent subtype - 3 for benchmark (Planner, CAD_agent, Reviewer), 3 for engineering (Planner, CAD_agent, Reviewer).

We will also need evaluations for an agent. -->

## Evaluations architecture

We need evaluation criteria that would be not only functional, but tied to numbers and count of tool calls/reviews from LLM-as-judge.

Bad example: specific as "Markdown would be valid."
Good example: Testing of markdown for structural validity be successful in 95% of cases.

## Multi-level evaluations architecture

We should be able to test evaluations on multiple tiers, specifically:

### Fast

1. Testing of markdown validity (for planners, TODO lists, reviewers) | 95% of cases on the first prompt.
2. Testing of code validity, ruff/pyright checks - 90% of cases on an average tool call
    - Including, specifically, build123d code. So we would generate a valid build123d code in 95% of the cases.
3. Testing that output YAML is valid in 95% of cases.
    - In `objectives.yaml`, the objectives do not intersect, specified objects do not intersect
    - In `objectives.yaml`, the objectives are in bounds of the model.
4. Given a prompt, agents read the necessary skill documents (as instructed).
    - For each agent.

### Medium evals

#### Medium evals - Engineer evaluations

##### Medium evals - Engineer Planner evaluations

<!-- We probably want to split this into further three sections below. -->
1. Given a prompt, an engineering planner uses appropriate components to solve a problem, and creates plans that are physically achievable, does not have intersecting models, and actually passes.
    - Validation:
        1. YAML
        2. LLM-as-a-judge (plan reviewer).
2. Given a prompt, the engineering planner uses accurate prices from the catalog and doesn't "come up" with prices.
3. Given a prompt, an engineering planner does not generate plans for features outside of a build zone.
4. Given a prompt, an engineering planner plans for a solution that is equal or lower to the `max_unit_cost`, `max_weight`, as well as other numeric constraints in 95% of cases.
5. Given a prompt, the engineering planner produces plans with correct units (e.g. metric or US customary units in 95% of cases).

##### Medium evals - CAD Engineer

1. Given a plan, the engineer will pass manufacturability checks in 70% during validation and pricing tool call. (note: lower than during submission because this is explicitly not submitting but validating the tool call).
2. Given a plan, the engineer will pass manufacturability checks in 90% of tool calls when expected during simulation or submission (when they *expect* that it will definitely pass). On second and third attempts this will improve to 95% and 97% respectively.
3. The engineer would adhere to requests of the user (passed indirectly to plan) about availability or forbid of drilling of the surrounding environment (note: it should be per-surface, not globally. E.g. one may drill into the floor but may not drill into another machine; but that will come later)
4. The engineer, after simulation, would interpret the simulation results correctly [...] (how to enforce it? they wouldn't always need to view the results, they can use final positions table that should be output or text too.)
5. The engineer will prefer CSG over sketches in 70% of the cases (soft requirement, but it makes it actually easier to build with code).

##### Medium evals - Engineer Reviewer

1. Given a viewed model, the review agent will correctly navigate images of the output (would check at least 3 images) before solution
2. Given a viewed model, the review agent would be able to correctly identify the issues in the plan in at least 70% of the cases.
    - Correctness: given a valid plan with an issue introduced by another LLM, a reviewer would be able to spot the issue in the plan (with the first LLM or other LLM validating that in fact, the described found issue matches).
3. Given a viewed model, the reviewer would force the CAD engineer to provide a cheaper solution in at least 15% of the cases.
4. Reviewer efficacy - reviews would be successful enough that they lead to a solution in at least 60% of cases after a failure. This means the reviewer should look deeper than one surface problem.
5. Price/weight escalation refusal - If the price or weight was set inadequately, the reviewer would scrutinize how the CAD model can be improved; and if they can find ways to thin/optimize a shape that would in fact be sufficient, then they will refuse the manufacturing. (specifics?)
6. Manufacturability awareness - the reviewer would be reporting only solutions appropriate to a given manufacturing method.
    - The reviewer would not be reporting solutions inappropriate to the manufacturing method in 97% of the cases.
7. Toolkit usage and diversity; the model will use:
    - fasteners in at least in 70% of builds
    - Motors in at least 20% of the builds
    - bearings in at least 10% of the builds
    - COTS search will be executed in at least 50% of builds (you need to look for specific version of fasteners, motors, etc.)
    - Other tools will be used at least reasonably often, or at least sometimes (3%?)
    (this requirement is more so for prompt debugging - that we don't miss including something into the prompt/skill sections.)
8. The model would be able to execute a search (or use a subagent) in COTS

<!-- Future: Given a prompt, the engineering planner doesn't use components that are out of stock -->

<!-- FIXME: Underspec: we don't define coordinate system starting point.I.e. is the center defined as (0,0,0)? Else how do we define it?
Proposal: normalize the simulation to the center bottom of the build zone. So the bottom center of the simulation would be 0,0,0; whatever is under it would be -z, and everything would be normalized to it. I expect it'll help agents adjust the build. -->

#### Medium evals -  Benchmark Generator

##### Medium evals - Benchmark Generator Planner

1. Given a prompt, a benchmark planner generates a plan that upon valid scrutinizing of a plan reviewer, passes in 80% of cases.
2. Given a prompt, a benchmark generator planner generates a plan that would have unimpeded objectives (objectives obstructed by obstacles by no more than 35%) in 97% of the cases (calculated by volume - shouldn't be obstructed.)
3. Given a prompt, the benchmark generator will produce plans for various manufacturing quantities (prototype <5, small volume <100, mass-manufacturing - 3000)
4. Given a prompt, we will include to the solution proposed by the prompt:
   - will include correct:
      - Quantity
      - Max weight,
      - Max cost
      - And other numerical parameters specified in objectives.yaml.
5. The benchmark generator would be able to predict the price and weight the engineer will solve the solution in the range of 80-120% (with 20% error) of the final price in 80% of the cases, within 50-150% in 97% of cases (this is the standard price, not the "safe" price)

##### Medium evals - Benchmark Generator CAD engineer

1. Given a plan, a benchmark CAD drafting agent generates a benchmark that is physically valid, uses features that actually exist in our pipeline (does correct constraints, etc) in 95% of cases after 30 turns, and 3 submissions to reviewer.

- First submission - 10 tool calls, 70% pass,
- Second submission - 20 tool calls, 85% pass,
- Third submission - 30 tool calls, 95% pass.

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
2. The Engineer (Planner+CAD Engineer) agents would not create examples that would break parts in 90% of the cases (can estimate if it (only motors for now, soft bodies in the future) will break using a formula).

##### Engineering Planner

##### CAD Engineer

1. Given a plan, an Engineer builds a build123d model that is valid and reaches the goal within 30 tool calls and three simulation attempts.
    - First submission - 10 tool calls, 70% pass,
    - Second submission - 20 tool calls, 85% pass,
    - Third submission - 30 tool calls, 95% pass.
2. Robustness test: If at least 1 simulation passes, at least 70% of "runtime jitter" variations pass too (i.e. the solution is not "flaky" that only works for one specific seed, but is mechanically robust).
3. Given a plan and `objectives.yaml`, the Engineer will not try to submit a solution that is more unit expensive or heavy than unit tests set by the planner and will:
    - Cheaper than the max price: 80% on first attempt, 90% after the first failure, 95% after the second failure.
    - Small error: 0-20% more expensive - In 10% on first attempt, 5% after the first error, 3% after the second failure (only small failyre, not including large error).
    - Large error: >20% more expensive - In 10% of first failure, 5% on second failure, 3% after after the second failure.

##### Reviewer

Plans not rejected by

#### Slow evals - Benchmark generator

##### Slow evals - CAD Engineer

1. Given a correct plan, a benchmark generator builds a benchmark that an engineer would be able to solve within 30 turns in 70% of cases.
2. Quality of Reasoning: The reasoning traces for benchmark generation are coherent and logically sound in >90% of cases (important for dataset quality).

### Integration/post-processing evals (more than one episode)

There are some episodes which can be take multiple episodes to run.

1. Plan-to-CAD fidelity: after the CAD was successfully reconstructed into a successful solution, generate a plan, and reconstruct again. The result (volume) should be approximately the same (80%).
<!-- That said, I'm not sure if it's necessary. -->
1. Robustness to randomization - having solved 1 seed from a randomization batch, an engineer would be statistically 50% more likely to other episodes in the batch relative to average performance (would fail 50% less).
2. If a reviewer said "this is acceptably the cheapest we can get" and then the model got cheaper by 50% unit cost at the same quantity (during subsequent planning) - the reviewer in fact didn't find an optimal quantity, and that shouldn't happen... (in which cases out of which?)

##


