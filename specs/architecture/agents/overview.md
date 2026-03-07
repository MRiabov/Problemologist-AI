# Agents

## Scope summary

- Primary focus: top-level architecture map of the agent layer.
- Defines the two main workflows: benchmark generator and engineer.
- Use this file first when deciding which agent-specific spec to open next.

We have two agents (or agent graphs): the benchmark generator and the engineer.

## Benchmark generator workflow

The benchmark generator flow is:

1. `benchmark_planner`
2. `benchmark_coder`
3. `benchmark_reviewer`

## Engineering workflow

The engineering flow has two separate review stages:

1. `engineer_planner` (+ `electronics_planner`)
2. `engineer_plan_reviewer` (plan-quality gate before coding)
3. `engineer_coder` (+ `electronics_engineer` and `electronics_reviewer`)
4. `engineer_execution_reviewer` (post-validation/post-simulation execution gate)

The split between plan and execution reviewers is mandatory.

- Plan review checks planning quality and contract completeness before implementation starts, and persists reviewer output to a stage-specific file (`reviews/engineering-plan-review-round-<n>.md`).
- Execution review runs only after validation/simulation success artifacts are present for the latest revision, then checks robustness/non-flakiness and plan adherence, and persists reviewer output to `reviews/engineering-execution-review-round-<n>.md`.
