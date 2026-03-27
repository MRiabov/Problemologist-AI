Workspace: current directory
Agent: electronics_reviewer
Task ID: erv-002-diverter-gate-review
Seed dataset: dataset/data/seed/role_based/electronics_reviewer.json

Task:
Review the seeded electronics implementation for the single-motor diverter gate. Confirm the 12V power path is complete, both wires stay inside the left-wall cable corridor, the seeded validation and simulation evidence correspond to the current script revision, and the reviewer returns a structured electronics decision.

Workspace contract:

- Use workspace-relative paths only. Do not use absolute workspace root prefixes.
- The workspace already contains the starter files, role templates, and any copied seed artifacts. Treat `.manifests/` as system-owned and do not edit it directly.
- You are the Execution Reviewer.
- Inspect the implementation, validation results, simulation result, and stage-specific review files.
- Write the stage-specific review decision and comments files under `reviews/` with `write_file`.
- When the review is ready, run `bash scripts/submit_review.sh` to validate the handoff.
- If the latest implementation is not reviewable, reject it with concrete reasons in the review files.
