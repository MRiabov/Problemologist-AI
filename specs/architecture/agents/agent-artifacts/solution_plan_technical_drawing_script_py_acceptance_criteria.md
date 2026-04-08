# solution_plan_technical_drawing_script.py Acceptance Criteria

## Role of the File

This script is the orthographic drawing companion for the engineering plan.
It is worth being a dedicated artifact because it helps reviewers validate the proposed solution without changing the actual solution contract.

## Hard Requirements

- The script matches the same geometry and labels as the engineering evidence script.
- The script supports plan review without drifting from the contract.
- The script is consistent with `assembly_definition.yaml` and the plan text.
- The script structurally constructs build123d `TechnicalDrawing` at least once; comments or string literals do not satisfy that requirement.

## Quality Criteria

- The drawing is orthographic, readable, and useful for reviewer comparison.
- The annotations help explain the proposed solution rather than adding noise.
- The companion remains aligned with the same labels and assembly structure as the plan.

## Reviewer Look-Fors

- Unsupported views, callouts, dimensions, or geometry appear.
- The script diverges from the evidence script or from `assembly_definition.yaml`.
- `TechnicalDrawing` is not a real construction path.
- The drawing attempts to justify a different contract than the one in the plan.

## Cross-References

- `specs/architecture/agents/handover-contracts.md`
- `specs/architecture/agents/artifacts-and-filesystem.md`
- `specs/architecture/agents/engineering-planner-technical-drawings.md`
