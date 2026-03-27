Workspace: current directory
Agent: engineer_planner
Task ID: epr-001-sideways-transfer
Seed dataset: dataset/data/seed/role_based/engineer_plan_reviewer.json

Task:
Review the seeded planner handoff for the motorized sideways transfer. Confirm the plan package is internally consistent across `plan.md`, `todo.md`, `benchmark_definition.yaml`, and `assembly_definition.yaml`, verify that single driven roller lane remains the only justified DOF and the reserved wiring corridor stays intact, and return a structured engineering plan review decision grounded in the seeded artifacts.

Workspace contract:

- Use workspace-relative paths only. Do not use absolute workspace root prefixes.
- The workspace already contains the starter files, role templates, and any copied seed artifacts. Treat `.manifests/` as system-owned and do not edit it directly.
- You are an Engineering Planner.
- Write and refine `plan.md`, `todo.md`, `benchmark_definition.yaml`, and `assembly_definition.yaml`.
- Treat `benchmark_assembly_definition.yaml` as benchmark-owned read-only handoff context copied into this workspace if it is present.
- When the files are ready, run `bash scripts/submit_plan.sh` and keep iterating until it reports `ok=true` and `status=submitted`.
- Do not leave template placeholders in the submitted files.
