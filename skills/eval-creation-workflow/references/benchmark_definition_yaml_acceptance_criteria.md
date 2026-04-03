# benchmark_definition.yaml Acceptance Criteria

Use this reference when creating or reviewing seeded benchmark planner artifacts.

## Role of the File

`benchmark_definition.yaml` is the benchmark-owned contract for the task geometry, objective zones, randomization, environment assumptions, and benchmark/customer caps.

Downstream roles should treat it as read-only context, except for planner-owned cap materialization when the current handoff explicitly allows it.

## Must-Have Content

- Exact goal-zone and forbid-zone geometry.
- Exact build-zone bounds.
- Moved-object definition with a top-level `start_position`.
- Static and runtime randomization ranges.
- Physics/backend notes.
- Benchmark caps and estimate fields.
- Exact benchmark-owned fixture metadata and labels.

## Acceptance Criteria

- The file schema-validates before execution continues.
- All object labels are unique and exact-mention complete across the handoff package.
- Goal and forbid zones do not intersect the moved object at spawn.
- The moved object remains inside the build zone after static randomization and runtime jitter.
- Numeric values are exact, not approximate, and do not rely on later inference.
- The file contains no placeholders, invented fields, or unstated benchmark fixtures.
- Any benchmark-owned moving fixture is declared explicitly, with motion visible in the handoff and evidence.

## Reviewer Use

- Reject schema-invalid or stale revisions.
- Reject benchmark-owned geometry that is inconsistent with the plan or evidence.
- Reject hidden or contradictory motion declarations.
- Reject plans that rely on coordinates or values not grounded in the benchmark contract.
