# Agents

## Scope summary

- Primary focus: top-level architecture map of the agent layer.
- Defines the two main workflows: benchmark generator and engineer.
- Use this file first when deciding which agent-specific spec to open next.

We have two agents (or agent graphs): the benchmark generator and the engineer.

Runtime conversations use four message roles: `system`, `user`, `assistant`, and `tool`.
`assistant` is the agent-authored/model role; `tool` is runtime-owned tool responses; `system` carries deterministic runtime control messages such as gates, reminders, and recovery instructions.

## Benchmark generator workflow

The benchmark generator flow is:

1. `Benchmark Planner`
2. `Benchmark Coder`
3. `Benchmark Reviewer`

## Engineering workflow

The engineering flow has two separate review stages:

1. `Engineering Planner` (+ `Electronics Planner`)
2. `Engineering Plan Reviewer` (plan-quality gate before coding)
3. `Engineering Coder` (+ `Electronics Engineer` and `Electronics Reviewer`)
4. `Engineering Execution Reviewer` (post-validation/post-simulation execution gate)

The split between plan and execution reviewers is mandatory.

- Plan review checks planning quality and contract completeness before implementation starts, and persists reviewer output to a stage-specific file (`reviews/engineering-plan-review-round-<n>.md`).
- Execution review runs only after validation/simulation success artifacts are present for the latest revision, then checks robustness/non-flakiness and plan adherence, and persists reviewer output to `reviews/engineering-execution-review-round-<n>.md`.
- Visual-inspection policy for vision-using roles is config-driven via `config/agents_config.yaml`; required roles must inspect render images through `inspect_media(...)` before valid finish/approval when images are available for the current node/revision.
