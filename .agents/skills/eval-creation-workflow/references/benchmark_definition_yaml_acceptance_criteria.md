# benchmark_definition.yaml Acceptance Criteria

Use this reference when creating or reviewing seeded benchmark planner artifacts.

## Role of the File

`benchmark_definition.yaml` is the benchmark-owned contract for the task geometry, objective zones, randomization, environment assumptions, and benchmark/customer caps.

Downstream roles should treat it as read-only context, except for planner-owned cap materialization when the current handoff explicitly allows it.

## What We Are Looking For

- A benchmark-owned contract that makes the task solvable without downstream guesswork.
- Exact geometry, zone, and randomization data that other handoff files can cross-check.
- Stable benchmark-owned identifiers and labels that survive into planner, coder, reviewer, and evidence artifacts.
- A meaningful challenge, not a degenerate stub that only satisfies shape checks.

## Must-Have Content

- Exact goal-zone and forbid-zone geometry.
- Exact build-zone bounds.
- Moved-object definition with a top-level `start_position`.
- Static and runtime randomization ranges.
- Physics/backend notes.
- Benchmark caps and estimate fields.
- Exact benchmark-owned fixture metadata and labels.

## Hard Requirements

- The file schema-validates before execution continues.
- All object labels are unique and exact-mention complete across the handoff package.
- Goal and forbid zones do not intersect the moved object at spawn.
- The moved object remains inside the build zone after static randomization and runtime jitter.
- Numeric values are exact, not approximate, and do not rely on later inference.
- The file contains no placeholders, invented fields, or unstated benchmark fixtures.
- `moved_object.material_id` is present and resolves to a known material from `manufacturing_config.yaml`.
- Any benchmark-owned moving fixture is declared explicitly, with motion visible in the handoff and evidence.

## Cross-References and Cross-Validation

- Compare against `plan.md` for the exact stage narrative and identifier mentions.
- Compare against `todo.md` for current-stage work items and completion state.
- Compare against `benchmark_assembly_definition.yaml` for benchmark-owned fixture and motion context.
- Compare against `benchmark_plan_evidence_script.py` and `benchmark_plan_technical_drawing_script.py` when drafting mode is active.
- Compare against `benchmark_script.py` for downstream inventory and geometry materialization.
- Compare against `manufacturing_config.yaml` for material IDs and other benchmark-customer assumptions.
- Compare against `references/role_input_index.md` and the live seeded-entry validator.
- Compare against any render or simulation evidence when the current stage produces it.

## Quality / Suitability for Evaluation

- The file is specific enough that downstream roles must use it instead of inferring missing geometry or limits.
- Randomization is valid but bounded, so the task remains solvable and repeatable.
- The contract exposes a real challenge without hidden contradictions, impossible geometry, or unsupported motion.
- The file makes it easy to catch incorrect model behavior because the required identifiers and bounds are exact.
- If used as a negative case, it should still pass deterministic entry checks and fail later for substantive reasons.

## Reviewer Use

- Reject schema-invalid or stale revisions.
- Reject benchmark-owned geometry that is inconsistent with the plan or evidence.
- Reject hidden or contradictory motion declarations.
- Reject plans that rely on coordinates or values not grounded in the benchmark contract.
- Reject seeds that only pass by stubbing out required fields or by leaving fields empty/default to evade validation.
