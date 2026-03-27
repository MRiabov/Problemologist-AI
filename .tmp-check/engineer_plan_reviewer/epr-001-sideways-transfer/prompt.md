Workspace: current directory
Agent: engineer_plan_reviewer
Task ID: epr-001-sideways-transfer
Seed dataset: dataset/data/seed/role_based/engineer_plan_reviewer.json

Task:
Review the seeded planner handoff for the motorized sideways transfer. Confirm the plan package is internally consistent across `plan.md`, `todo.md`, `benchmark_definition.yaml`, and `assembly_definition.yaml`, verify that single driven roller lane remains the only justified DOF and the reserved wiring corridor stays intact, and return a structured engineering plan review decision grounded in the seeded artifacts.

Workspace contract:
- Use workspace-relative paths only. Do not use absolute workspace root prefixes.
- The workspace already contains the starter files, role templates, and any copied seed artifacts. Treat `.manifests/` as system-owned and do not edit it directly.
- You are the Plan Reviewer.
- Inspect the planner artifacts and write the stage-specific review decision and comments files under `reviews/` with `write_file`.
- When the review is ready, run `bash scripts/submit_review.sh` to validate the handoff.
- Use the latest planner handoff state; do not edit planner-owned source files.
- If the benchmark or engineer plan is invalid, reject it with concrete reasons in the review files.
