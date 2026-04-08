# benchmark_plan_technical_drawing_script.py Acceptance Criteria

## Role of the File

This script is the orthographic drawing companion for the benchmark plan.
It is worth being a dedicated artifact because it turns the benchmark handoff into reviewable drawing output without changing the geometry contract.

## Hard Requirements

- The script matches the same geometry and labels as the benchmark evidence script.
- The script remains grounded in the benchmark contract.
- The script does not invent extra geometry, labels, views, or dimensions.
- The script structurally constructs build123d `TechnicalDrawing` at least once; comments or string literals do not satisfy that requirement.
- The script is suitable for `preview_drawing()` before submission.

## Quality Criteria

- The drawing is orthographic and readable rather than a second opaque geometry contract.
- The views and annotations help the reviewer validate the benchmark package faster.
- The drawing companion stays strictly aligned with the benchmark inventory and motion facts.

## Reviewer Look-Fors

- Unsupported views, callouts, datums, or dimensions appear.
- The script diverges from the evidence script or the benchmark YAML.
- `TechnicalDrawing` is missing as a real construction path.
- The drawing introduces labels or geometry that are not grounded in the benchmark contract.

## Cross-References

- `specs/architecture/agents/handover-contracts.md`
- `specs/architecture/agents/artifacts-and-filesystem.md`
- `specs/architecture/agents/engineering-planner-technical-drawings.md`
