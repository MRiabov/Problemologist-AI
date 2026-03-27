Workspace: current directory
Agent: engineer_execution_reviewer
Task ID: eer-002-gap-bridge
Seed dataset: dataset/data/seed/role_based/engineer_execution_reviewer.json

Task:
Review the seeded engineering execution package for the gap bridge transfer. Confirm the current `script.py`, planner artifacts, validation result, and simulation result all correspond to the latest revision, inspect the seeded render evidence when present, and determine whether implemented bridge geometry preserves the approved passive handoff across the gap and does not introduce unnecessary motion. Return a structured engineering execution review decision grounded in the seeded evidence.

Workspace contract:
- Use workspace-relative paths only. Do not use absolute workspace root prefixes.
- The workspace already contains the starter files, role templates, and any copied seed artifacts. Treat `.manifests/` as system-owned and do not edit it directly.
- You are the Execution Reviewer.
- Inspect the implementation, validation results, simulation result, and stage-specific review files.
- Write the stage-specific review decision and comments files under `reviews/` with `write_file`.
- When the review is ready, run `bash scripts/submit_review.sh` to validate the handoff.
- If the latest implementation is not reviewable, reject it with concrete reasons in the review files.
