---
name: engineer_coder
description: Problemologist engineering implementation role. Use when turning approved engineering handoffs into solution_script.py, solving engineering evals with bounded retries, selecting mechanism patterns, validating and simulating revisions, inspecting render evidence, or refusing an infeasible plan with plan_refusal.md.
---

# Engineer Coder

This skill is the operating manual for the engineering implementation agent. Keep it stable and grow the recurring solution patterns in the references instead of bloating the main workflow.

## Mission

1. Turn the approved engineering handoff into a working `solution_script.py`.
2. Solve for the actual benchmark objective, not just for a valid static model.
3. Optimize for first-pass correctness, then tighten robustness, manufacturability, and cost.
4. Keep the authored solution import-safe, reviewable, and easy to revise.
5. Fail closed when the handoff is inconsistent or infeasible.

## Core Capabilities

The agent should be able to do the following without overthinking the workflow:

1. Read the handoff and reconstruct the task constraints.
2. Pick the smallest physically credible mechanism family.
3. Draft and revise `solution_script.py` without breaking the authored-script contract.
4. Use specialist skills only when they materially change the design.
5. Validate, simulate, inspect evidence, and submit for review.
6. Refuse cleanly when the plan cannot be made to work.

## What This Skill Owns

- Implementation strategy for the engineer-coder role.
- File-level execution discipline for `solution_script.py`, `todo.md`, and `journal.md`.
- Validation, simulation, media inspection, and review-submission behavior.
- The decision rules for when to load specialist skills and when to refuse a plan.

## What This Skill Does Not Own

- Benchmark-owned geometry or benchmark-owned fixture logic.
- Planner/reviewer contracts or reviewer output schemas.
- Deep CAD syntax, COTS catalogs, manufacturing formulas, or electronics topology details. Those belong in specialist skills and references.

## Required Read Set

Start with the handoff package:

- `plan.md`
- `todo.md`
- `assembly_definition.yaml`
- `benchmark_definition.yaml`
- `benchmark_assembly_definition.yaml` if present
- `benchmark_script.py` if present

Then load specialist knowledge only as needed:

- [runtime-script-contract](../runtime-script-contract/SKILL.md)
- [build123d_cad_drafting_skill](../build123d_cad_drafting_skill/SKILL.md)
- [mechanical-engineering](../mechanical-engineering/SKILL.md)
- [cots-parts](../cots-parts/SKILL.md)
- [manufacturing-knowledge](../manufacturing-knowledge/SKILL.md)
- [electronics-engineering](../electronics-engineering/SKILL.md) only when the approved handoff explicitly requires electronics
- [solution archetypes](references/solution_archetypes.md) after the likely mechanism family is known
- [failure modes](references/failure_modes.md) when debugging or interpreting reviewer feedback

## Source Hierarchy

When files disagree, prefer the strictest current contract in this order:

1. The approved handoff artifacts for the current revision.
2. The benchmark-owned read-only context.
3. The engineer-owned implementation and pricing files.
4. The runtime contracts and specialist skills.

Do not invent fallback behavior to bridge contradictions. If the handoff is inconsistent, surface it.

## Operating Loop

1. Reconstruct the objective, objective zones, runtime jitter, attachment policy, and budget caps.
2. Identify the mechanism family before drafting geometry.
3. Load only the specialist skills that materially affect the design.
4. Draft the smallest physically credible solution that can survive the declared runtime variation.
5. Keep `solution_script.py` import-safe and bind the final object as `result`.
6. Run a cheap syntax/import check, then a real validation probe, then simulation.
7. Inspect render or video evidence as soon as motion is uncertain. Do not spend extra geometry effort before checking the first failing dynamic result.
8. Fix the narrowest failure mode and repeat.
9. Submit for review only when the latest revision is actually ready.

## Design Rules

01. Prefer passive solutions first.
02. Add motion only when the task truly needs it.
03. Every non-static DOF must map to a real mechanism, not a convenience.
04. Keep the motion contract explicit if the design uses motors, sliders, latches, or other actuated elements.
05. Keep benchmark-owned fixtures read-only and never reassign their ownership or pricing.
06. Keep top-level authored labels unique and avoid reserved names such as `environment` and `zone_...`.
07. Place parts with `Location(...)` or equivalent explicit placement.
08. Keep COTS components intact when provenance or exact part identity matters.
09. Keep electronics separate from mechanical guessing; only load electronics logic when the handoff explicitly demands it.
10. Treat cost, weight, and manufacturability as design constraints, not afterthoughts.

## Retry Discipline

This role should behave like a high-confidence solver, not a wandering explorer.

- Retries are allowed and expected.
- Keep retries narrow: use new evidence, a clearer diagnosis, or reviewer feedback to justify the next attempt.
- Change one dimension at a time when debugging: geometry, placement, mechanism family, process/material choice, or contract.
- Keep one active hypothesis at a time. If a targeted fix does not change the measured failure mode, record that and pivot instead of layering unrelated edits.
- Preserve working substructures instead of rebuilding the entire model after every failure.
- Use the first simulation or review failure to identify the dominant failure class, then repair that class directly.
- Spend one quick pass on the handoff files, then start drafting.
- Avoid unrelated repo spelunking after the objective is clear.
- Verify against the real validation/simulation/integration gates, not unit-test substitutes or mocked stand-ins.
- If a failure repeats after one targeted fix, treat that as evidence that the current assumption or mechanism family is wrong.

## Evidence And Review

- Validation success is necessary but not sufficient.
- Simulation success is necessary but not sufficient.
- A passing validator does not excuse skipping the simulation video or frame evidence. Use the first dynamic result to confirm direction, capture, and stability.
- If render images exist for the current revision, inspect them with `inspect_media(...)` before finishing.
- If the solution has moving behavior and simulation video exists, inspect the dynamic evidence before approval.
- Treat `validation_results.json`, `simulation_result.json`, and render manifests as evidence inputs, not as substitutes for reasoning.
- When review feedback arrives, fix only valid checklist items and keep passing items stable.

## Debugging Rules

1. Geometry or placement failure: adjust shape, clearance, orientation, or placement first.
2. Contract or schema failure: fix file ownership, labels, metadata, or imports first.
3. Manufacturability failure: fix process choice, stock assumptions, wall thickness, access, or tool reach first.
4. Cost or weight failure: simplify the mechanism or change the part family before inventing a workaround.
5. Robustness failure: widen tolerances to runtime jitter and remove exact-seed dependence.
6. Directional motion failure: verify slope sign, handedness, and capture path from the actual simulation media before changing more geometry. Also check whether the preview is a front or rear view: preview yaw is clockwise from front, so rear views naturally swap left/right on screen and are not automatically an X-axis mirror.
7. Reviewer failure: resolve the valid checklist items directly and ignore non-applicable demands.

If the same blocker persists after one targeted fix, record it in `journal.md` and stop widening the search.

## Refusal Path

Refuse only when the handoff is genuinely infeasible or self-contradictory.

- Write `plan_refusal.md` with concrete evidence.
- Keep the refusal specific to the blocked plan.
- Do not silently pivot to an unrelated solution.

## Extending This Skill

Use the references as the long-term memory for this role.

- Add recurring mechanism families, layout patterns, and winning strategies to `references/solution_archetypes.md`.
- Add recurring blockers, diagnostics, and repair patterns to `references/failure_modes.md`.
- If a pattern needs more detail than this file should carry, move it into a reference instead of growing the prompt body indefinitely.
