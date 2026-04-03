# precise_path_definition.yaml Acceptance Criteria

Use this reference when creating or reviewing `precise_path_definition.yaml` for engineer-coder motion/path proof.

## Role of the File

`precise_path_definition.yaml` is the engineer-owned higher-resolution path and contact proof.
It refines the coarse motion forecast rather than replacing it, and it should support backend-specific contact evidence and preview overlays. The file also anchors the precise trace to the solution backend and initial pose.

## What We Are Looking For

- A strict engineer-owned motion/path contract with explicit endpoint semantics.
- A denser, backend-specific path that is consistent with the approved coarse forecast.
- Exact moving-part names, backend selection, initial pose, and terminal event data that let reviewers validate the path without guessing.
- A file that proves implementation detail, not planner prose.

## Must-Have Content

- Ordered motion anchors with explicit positions and tolerance data.
- The first anchor marked `build_zone_valid: true`.
- Explicit moving-part names that match the approved coarse motion forecast.
- A terminal contact/event assertion for the final state.
- A sample stride that is finer than or equal to the coarse forecast stride.
- Motion metadata that can be used for preview overlays and contact proof.

## Hard Requirements

- The file schema-validates and parses cleanly.
- The file contains no template placeholders or empty stub values.
- `backend` and `initial_pose` must be present and consistent with the implementation backend.
- `moving_part_names` must match the approved coarse motion forecast.
- `sample_stride_s` must not exceed the coarse motion forecast stride.
- `sample_stride_s` must stay within the engineer-coder motion forecast budget from `config/agents_config.yaml`.
- The first anchor must lie within `benchmark_definition.objectives.build_zone` and explicitly set `build_zone_valid: true`.
- The file must prove the terminal goal-zone entry/contact in the last anchor or `terminal_event`, not both.
- If `terminal_event` is used, its `zone_name` must be `goal_zone`, its `contact_surfaces` must be populated, and its position must lie within the goal zone.
- The file must remain consistent with `benchmark_definition.yaml` and any coarse motion forecast or benchmark motion evidence in the workspace.
- The file must not loosen or contradict the approved build-safe start or goal-contact finish semantics.
- If `plan.md` includes timing, speed, or contact-window math, that math must use the same waypoint sequence and terminal proof as `precise_path_definition.yaml`; the plan may describe an average-segment envelope, but it may not silently claim a different motion trace.

## Cross-References and Cross-Validation

- Compare against `benchmark_definition.yaml` for benchmark-owned objective and motion context.
- Compare against the coarse `motion_forecast` contract when present.
- Compare against `config/agents_config.yaml` for motion cadence/tolerance policy.
- Compare against `worker_heavy/utils/file_validation.py` for the live validation rules.
- Compare against `specs/architecture/agents/handover-contracts.md` and `specs/architecture/simulation-and-rendering.md` for the contract intent.
- Compare against preview evidence when `motion_forecast=True` overlays are present.

## What Good Looks Like

- The path is precise enough to support contact proof, but still remains readable and reviewable.
- The file helps the reviewer see why the solution starts safely and finishes with explicit goal contact.
- The path is strict enough that malformed or under-specified motion cannot slip through as a valid seed.
- The path and the planner math tell the same story at the same timestamps.

## Negative Example

- A file that copies the coarse motion forecast without adding the required precision or terminal proof is not sufficient.
- A stub that merely satisfies YAML shape while omitting the contract semantics is not valid.

## Review Questions

- Does the precise path refine, rather than replace, the coarse forecast?
- Are the moving part names, cadence, and terminal event exact?
- Would the current workspace pass validation because of this file, or would it still rely on guesswork?
