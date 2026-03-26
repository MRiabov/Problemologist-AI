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
