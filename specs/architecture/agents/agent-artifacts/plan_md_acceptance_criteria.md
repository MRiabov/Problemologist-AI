# plan.md Acceptance Criteria

## Role of the File

`plan.md` is the stage narrative that makes the handoff readable to humans and downstream agents.
It is worth being a dedicated artifact because it binds the exact stage objective, the inventory names, and the intended sequence of work in a stable, reviewable form.

## Hard Requirements

- Headings match the active role contract for the seeded stage.
- The plan states the role-specific objective clearly.
- Exact identifiers, labels, and file names used elsewhere in the handoff appear where the contract expects them.
- The plan is consistent with the current seeded revision and does not drift from the YAML, scripts, or review artifacts.
- The plan does not invent parts, zones, motion, pricing, or unsupported work.

## Quality Criteria

- The plan is specific enough that a downstream role can start work without guessing missing contract details.
- The narrative is coherent and stage-ordered rather than a loose collection of notes.
- The plan uses exact names from the workspace instead of generic placeholders.

## Reviewer Look-Fors

- Generic prose appears where the contract expects exact names or counts.
- Required headings or exact identifier mentions are missing.
- The plan conflicts with the YAML files, scripts, or review artifacts.
- Stale stage content or unsupported work appears in the handoff.

## Cross-References

- `specs/architecture/agents/roles.md`
- `specs/architecture/agents/handover-contracts.md`
- `specs/architecture/agents/artifacts-and-filesystem.md`
