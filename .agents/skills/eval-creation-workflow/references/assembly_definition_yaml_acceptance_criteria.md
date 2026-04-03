# assembly_definition.yaml Acceptance Criteria

Use this reference when creating or reviewing engineer-side seeded workspaces.

## Role of the File

`assembly_definition.yaml` is the engineer-owned contract for solution-side parts, constraints, pricing, totals, and final assembly.

## Must-Have Content

- `manufactured_parts` with method/material and method-specific costing fields.
- `cots_parts` with exact catalog-backed `part_id`, manufacturer, and source data.
- `final_assembly` with subassemblies, reuse, and joints.
- Planner-owned cap copies from `benchmark_definition.yaml`.
- Planner-target unit cost and weight fields derived from validated totals.
- Electronics fields when the role explicitly requires them.

## Acceptance Criteria

- The file schema-validates before execution continues.
- Every COTS component is backed by an exact `part_id` from the current episode.
- No invented prices, manufacturers, or catalog IDs appear.
- Derived numeric values are exact, not approximated later.
- The file stays internally consistent with `plan.md`, `todo.md`, and `benchmark_definition.yaml`.
- Any benchmark-owned context copied into the workspace remains read-only.
- The final assembly is grounded in validated totals and does not exceed benchmark caps.

## Reviewer Use

- Reject placeholder values, schema drift, or unvalidated totals.
- Reject pricing that does not come from the configured catalog/workspace sources.
- Reject any solution that treats benchmark-owned geometry as editable engineer scope.
