---
name: engineer-plan-reviewer
description: Engineer-side review workflow for validating plan and execution handoffs, review manifests, render and simulation evidence, exact inventory grounding, motion-contract plausibility, plan refusals, and stage-scoped review YAML outputs. Use when reviewing engineering `plan.md`, `todo.md`, `benchmark_definition.yaml`, `assembly_definition.yaml`, `solution_script.py`, validation or simulation artifacts, review manifests, or refusal evidence for the Engineering Plan Reviewer or Engineering Execution Reviewer roles; or when applying the engineer review checklist for plan, execution, and refusal gates.
---

# Engineer Solution Reviewer

## Review Checklist

- [ ] Confirm the latest revision and matching stage manifest.
- [ ] Treat planner and coder artifacts as read-only.
- [ ] Inspect render or simulation media with `inspect_media(...)` whenever they exist.
- [ ] Write only the stage-scoped review decision and comments YAML pair.
- [ ] Route invalid refusals back to coding unless the refusal proves infeasibility.

### Plan Review Checklist

- [ ] Read `plan.md`, `todo.md`, `benchmark_definition.yaml`, `assembly_definition.yaml`, `solution_plan_evidence_script.py`, and `solution_plan_technical_drawing_script.py`.
- [ ] Treat `solution_plan_evidence_script.py` and `solution_plan_technical_drawing_script.py` as the inspectable source of the approved plan, and inspect preview evidence with `inspect_media(...)` when present.
- [ ] Verify exact inventory grounding, exact identifier mentions, budget realism, and operating-envelope clarity.
- [ ] Reject invented materials, unsupported mechanisms, hidden DOFs, or a technical-drawing script without a real `TechnicalDrawing` construction path.
- [ ] Reject plans that exceed benchmark caps or leave the solution mechanically ambiguous.

### Execution Review Checklist

- [ ] Read `solution_script.py`, helper modules, `validation_results.json`, `simulation_result.json`, and the active plan context.
- [ ] Require validation and simulation success for the latest revision.
- [ ] Verify plan fidelity, robustness, manufacturability, and cost/weight compliance against the approved plan.
- [ ] Reject flaky runtime-jitter behavior, excessive or unjustified DOFs, or any render/video evidence that was not inspected.

### Refusal Review Checklist

- [ ] Read `plan_refusal.md` and the rejected plan evidence.
- [ ] Confirm only when the refusal contains role-specific reasons and concrete evidence that the plan is infeasible.
- [ ] Reject generic coding failure or a refusal that can still be implemented.

## Comment Checklist

- [ ] Plan reviewers: `cross_artifact_consistency`, `feasible_mechanism`, `budget_realism`, `dof_minimality`.
- [ ] Execution reviewers: `latest_revision_verified`, `validation_success`, `simulation_success`, `visual_evidence_checked`, `dynamic_evidence_checked`, `plan_fidelity`, `robustness`, `cost_weight_compliance`, `manufacturability_compliance`, `dof_deviation_justified`.
- [ ] Use `pass`, `fail`, or `not_applicable` values only where the stage contract requires them.
- [ ] Keep the summary factual and the required fixes concrete.

## References

- `references/review_contracts.md`
- `specs/architecture/agents/handover-contracts.md`
