# benchmark_assembly_definition.yaml Acceptance Criteria

Use this reference when creating or reviewing benchmark-owned fixture context.

## Role of the File

`benchmark_assembly_definition.yaml` is the read-only benchmark-owned fixture and motion context copied into engineer and reviewer workspaces.

## Must-Have Content

- Full benchmark-owned fixture structure.
- Exact benchmark part references and labels.
- Explicit moving-fixture motion contract, when present.
- One-axis motion per moving fixture unless the benchmark contract says otherwise.
- Controller facts, motion bounds, and reviewer-visible motion limits.

## Acceptance Criteria

- The file is schema-valid and complete.
- Every referenced benchmark-owned object exists in `benchmark_definition.yaml`.
- Motion is explicit, visible, and consistent across plan, todo, and evidence.
- Hidden motion, contradictory motion, or unsupported motion is rejected.
- The file stays read-only for downstream roles.
- No engineer-owned parts, solution-side assemblies, or invented fixtures appear in this file.

## Reviewer Use

- Reject any motion contract that is missing, contradictory, or unsupported by evidence.
- Reject cross-revision drift between the benchmark-owned fixture and the rest of the handoff package.
- Reject any attempt to reinterpret benchmark-owned fixtures as engineer-owned deliverables.
