Workspace: current directory
Agent: benchmark_reviewer
Task ID: br-012-sideways-ball-infeasible-goal
Seed dataset: dataset/data/seed/role_based/benchmark_reviewer.json

Task:
Review the seeded benchmark deliverables in the workspace for the sideways-ball infeasible-goal variant. Validate feasibility, objective consistency, and obvious simulation blockers using the existing handoff artifacts, with particular attention to whether the seeded goal zone remains inside the benchmark build and simulation envelope.

Workspace contract:

- Use workspace-relative paths only. Do not use absolute workspace root prefixes.
- The workspace already contains the starter files, role templates, and any copied seed artifacts. Treat `.manifests/` as system-owned and do not edit it directly.
- You are the Execution Reviewer.
- Inspect the implementation, validation results, simulation result, and stage-specific review files.
- Write the stage-specific review decision and comments files under `reviews/` with `write_file`.
- When the review is ready, run `bash scripts/submit_review.sh` to validate the handoff.
- If the latest implementation is not reviewable, reject it with concrete reasons in the review files.
