Workspace: current directory
Agent: benchmark_plan_reviewer
Task ID: bpr-012-gap-bridge-hidden-dof
Seed dataset: dataset/data/seed/role_based/benchmark_plan_reviewer.json

Task:
Review the seeded benchmark planner handoff for the gap-bridge cube transfer variant that adds a small self-centering correction on the bridge reference table. Confirm the plan package is internally consistent across `plan.md`, `todo.md`, `benchmark_definition.yaml`, and `benchmark_assembly_definition.yaml`, determine whether the bridge really remains a passive benchmark fixture or whether the seeded motion metadata introduces unsupported benchmark-side actuation, inspect the seeded render evidence when present, and return a structured benchmark plan review decision grounded in the seeded artifacts.

Workspace contract:
- Use workspace-relative paths only. Do not use absolute workspace root prefixes.
- The workspace already contains the starter files, role templates, and any copied seed artifacts. Treat `.manifests/` as system-owned and do not edit it directly.
- You are the Plan Reviewer.
- Inspect the planner artifacts and write the stage-specific review decision and comments files under `reviews/` with `write_file`.
- When the review is ready, run `bash scripts/submit_review.sh` to validate the handoff.
- Use the latest planner handoff state; do not edit planner-owned source files.
- If the benchmark or engineer plan is invalid, reject it with concrete reasons in the review files.
