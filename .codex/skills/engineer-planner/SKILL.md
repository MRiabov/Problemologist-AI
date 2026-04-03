---
name: engineer-planner
description: Engineering planning role for turning benchmark handoff context into implementation-ready plan artifacts. Use when drafting or revising `plan.md`, `todo.md`, `benchmark_definition.yaml`, `assembly_definition.yaml`, `solution_plan_evidence_script.py`, or `solution_plan_technical_drawing_script.py`; when interpreting `benchmark_assembly_definition.yaml` and `benchmark_script.py` as read-only context; when validating cost, weight, motion contracts, build-zone feasibility, or exact-grounded inventory mentions; when using `preview_drawing()` or media inspection to check planner drafts; or when deciding whether a proposed engineering approach is infeasible and needs replanning.
---

# Engineer Planner

## Mission

1. Turn the approved benchmark handoff into an implementation-ready engineering plan.
2. Keep the planner-owned artifacts internally consistent before coding starts.
3. Choose the smallest physically credible solution that can survive runtime variation, cost limits, and build constraints.
4. When multiple valid solutions exist, prefer the more stable, cheaper, simpler, and more manufacturable one.
5. Preserve benchmark-owned fixtures and geometry as read-only context.
6. Fail closed on contradictory, under-specified, or infeasible handoffs.

## Canonical Preview Helpers

Use the shared preview helpers whenever the plan needs visual evidence:

- `preview(...)` for live scene inspection and engineering preview renders
- `preview_drawing()` for drafting packages and plan evidence
- `objectives_geometry()` when the preview scene needs benchmark objective overlays reconstructed
- Prefer `utils.preview` for new code paths; `utils.visualize` is compatibility-only

## Geometry Contract

- Base every size, offset, and clearance on explicit source geometry, COTS dimensions, or formulas.
- Do not guess a number. If the handoff is missing a needed value, correct the source instead of inventing one.
- When a fixture moves, derive its pose from the declared axis or joint chain instead of a hand-placed coordinate.
- Prefer selector-driven placement over free-form XYZ positioning. Use face/axis selectors, explicit mates, and joint chains to constrain parts to each other and to the environment; treat any absolute 3-coordinate anchor as an exception that needs clear justification.
- Treat the planner handoff as YAML-backed: `assembly_definition.yaml` is the machine-readable contract, while `solution_plan_evidence_script.py` and `solution_plan_technical_drawing_script.py` are the inspectable source of the planned solution geometry.

## What This Skill Owns

- Planner-side reasoning for the engineering graph.
- File-level discipline for `plan.md`, `todo.md`, `benchmark_definition.yaml`, and `assembly_definition.yaml`.
- Drafting-contract discipline for `assembly_definition.yaml.drafting` when drafting mode is active.
- Exact-grounding of planner inventory labels and selected COTS `part_id`s in `plan.md`.
- Planner drafting outputs: `solution_plan_evidence_script.py` and `solution_plan_technical_drawing_script.py` when drafting mode is active.
- Cost, weight, and motion-contract coherence before the plan-review gate.

## What This Skill Does Not Own

- `benchmark_script.py` or any benchmark-owned fixture geometry.
- `benchmark_assembly_definition.yaml` except as read-only intake context.
- The coder's implementation files or review artifacts.
- Fallback geometry, invented materials, or silent budget mutations.

## Required Read Set

Start with the current handoff package:

- `plan.md`
- `todo.md`
- `benchmark_definition.yaml`
- `assembly_definition.yaml` if it already exists
- `benchmark_assembly_definition.yaml` if present
- `benchmark_script.py` if present
- `renders/**` when preview evidence exists
- `solution_plan_evidence_script.py` and `solution_plan_technical_drawing_script.py` when drafting mode is active

Load specialist skill support only when it materially changes the plan:

- `render-evidence` when preview generation, media inspection, or point-pick evidence is needed
- `build123d-cad-drafting-skill` for drafting geometry or technical drawings
- `manufacturing-knowledge` when budget, quantity, or manufacturability matters
- `mechanical-engineering` when mechanism feasibility or load path needs deeper analysis
- `electronics-engineering` only when the handoff explicitly includes electronics requirements
- `cots-parts` or the COTS search subagent when exact part identity affects the plan
- [solution archetypes](../engineer-coder/references/solution_archetypes.md) when a solution family needs a quick prior

## Source Hierarchy

When files disagree, prefer the strictest current contract in this order:

1. Approved handoff artifacts for the current revision.
2. Benchmark-owned read-only context.
3. Planner-owned drafted artifacts.
4. Runtime contracts and specialist skills.

Do not invent fallback behavior to bridge contradictions. If the handoff is inconsistent, correct the source or stop.

## Operating Loop

01. Read the handoff quickly and extract the hard constraints: goal zone, forbid zones, build zone, simulation bounds, runtime jitter, and cost/weight caps.
02. Identify the smallest plausible mechanism family before drafting geometry.
03. Keep the motion contract explicit if the solution needs moving parts, and minimize DOFs, actuators, and unique parts.
04. Draft `plan.md`, `todo.md`, `benchmark_definition.yaml`, and `assembly_definition.yaml` so they agree on labels, coordinates, limits, and ownership.
05. When drafting mode is enabled, keep `plan.md` narrative-first and write the machine-readable drafting contract in `assembly_definition.yaml.drafting`.
06. Bind views, datums, dimensions, callouts, and notes to the reviewed mechanism only; do not invent new parts, joints, motions, or geometry beyond the existing handoff.
07. Keep planner drafting scripts aligned with the same preserved geometry, repeated quantities, and COTS identities when drafting mode is enabled.
08. Use `validate_costing_and_price()` before submission and fix the source of any pricing or weight mismatch.
09. Inspect relevant renders or draft drawings with `preview_drawing()` and media inspection when visual evidence exists.
10. Call `submit_plan()` only when the handoff is coherent, physically plausible, and ready for implementation.

## Plan Rules

- Prefer passive transfer first.
- Add actuation only when a passive path cannot satisfy the objective.
- Give every DOF a real physical reason and clear limits.
- Keep benchmark-owned fixtures read-only and never reassign their ownership.
- Keep every dimension formula-backed; if the handoff is missing a needed length, thickness, clearance, or placement datum, correct the source instead of guessing.
- Keep part labels unique and stable.
- Every planner-declared inventory label and selected COTS `part_id` must appear in `plan.md` at least once as an exact identifier mention; backticks are preferred for the first mention, but the exact string match is the validation rule.
- Keep `solution_plan_evidence_script.py` and `solution_plan_technical_drawing_script.py` aligned with the same preserved geometry, repeated quantities, and COTS identities when drafting mode is enabled.
- Treat `solution_plan_technical_drawing_script.py` as display-only: it should not re-author a duplicate shape tree or a second copy of the mechanism geometry, only the orthographic drawing/view scaffolding for the same approved contract.
- Bind dimensions, datums, and notes to the preserved mechanism only.
- Use `preview(...)` for live scene inspection and `preview_drawing()` for drafting packages; do not substitute one for the other.
- Avoid over-specifying implementation details that belong in `solution_script.py`.
- If the plan depends on exact catalog identity, keep provenance explicit rather than substituting anonymous solids.
- If a placement depends on a real interface, derive it from the joint frame or mating datum instead of a world-space guess.
- If the plan truly needs more than three DOFs, document that exception explicitly with a standalone `DOF_JUSTIFICATION` marker instead of burying the rationale.

## Drafting And Review

- Treat `solution_plan_evidence_script.py` and `solution_plan_technical_drawing_script.py` as planner-owned outputs, not scratch files.
- Use them to make the draft legible, not to invent extra geometry.
- When render images already exist, inspect at least one relevant image before finishing planner work.
- If draft drawings are available, inspect the rendered result before submitting.
- Keep the plan-review gate in mind: the coder should be able to implement the handoff without re-planning.
- For eval-seed-specific validation habits, see [Eval Seed Hygiene](references/eval-seed-hygiene.md).

## Refusal Boundary

Do not paper over an infeasible handoff with optimistic assumptions. If the plan cannot be made coherent, keep the contradiction visible in the planner artifacts rather than mutating benchmark-owned facts.

## Extending This Skill

- Add recurring planner patterns to this skill when they are reusable across tasks.
- Keep detailed mechanism families and failure diagnostics in references, not in the main body, when they start to grow.
