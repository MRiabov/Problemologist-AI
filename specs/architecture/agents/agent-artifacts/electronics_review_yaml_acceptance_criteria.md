# electronics_review_yaml Acceptance Criteria

## Role of the File

This file family is the electronics review decision/comments pair.
It is worth being a dedicated artifact because explicit electromechanical work needs its own stage-canonical verdict and evidence trail instead of being folded into a generic engineering review.

## Hard Requirements

- The decision YAML is stage-canonical and revision-specific.
- The comments YAML is factual, evidence-based, and grounded in the latest implementation.
- Checklist keys match the expected electronics-review schema.
- The review package includes the electronics review manifest.
- Decision and comments files refer to the same latest implementation package.

## Quality Criteria

- The comments identify concrete electromechanical issues, not generic feedback about the broader assembly.
- The checklist matches the explicit electronics contract that was actually seeded.
- The verdict aligns with the implementation evidence and the current stage scope.

## Reviewer Look-Fors

- The decision and comments disagree or reference different revisions.
- The package relies on stale manifest data or a stale implementation snapshot.
- The checklist invents keys or omits required stage-canonical fields.
- The comments discuss electronics work that was not actually part of the current contract.

## Cross-References

- `specs/architecture/agents/handover-contracts.md`
- `specs/architecture/electronics-and-electromechanics.md`
- `specs/architecture/agents/artifacts-and-filesystem.md`
