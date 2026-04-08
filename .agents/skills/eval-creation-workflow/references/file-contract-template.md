# File Contract Template

Use this as the fallback only for new eval artifact types that are not listed yet in `specs/architecture/agents/agent-artifacts/`.
Start from this structure, then specialize it for the concrete filename, role, and validator.

## Role of the File

Describe what the file owns in the seeded workspace, which stage writes it, and which stage reads it.

## Hard Requirements

- The file has the expected name and stage ownership.
- The file schema-validates or parses as expected.
- Required fields are present, correctly typed, and populated with exact values.
- Any deterministic or derived fields are materialized at seed time, not inferred later.
- Exact cross-file references are present for every identifier the role must preserve.
- The file contains no placeholders, invented identifiers, or stale revision data.
- The file does not rely on lenient validation, default values, or empty stubs to pass.

## Quality Criteria

- The file expresses the stage contract that downstream code or reviewers actually consume.
- The file is grounded in the active handoff package, not generic boilerplate.
- Exact identifiers, counts, paths, hashes, and derived values match other seeded artifacts.
- The file gives the downstream role enough signal to evaluate the task without guessing.

## Reviewer Look-Fors

- The file is the right artifact for the role and stage, not a neighboring file that only seems related.
- The hard checks pass without any validator weakening or placeholder content.
- Cross-file names, counts, hashes, and identifiers are exact.
- The file would expose a bad model output rather than papering over it.

## Cross-References and Cross-Validation

- Compare against `references/role_input_index.md` for the stage-specific input bundle.
- Compare against the active role contract in `specs/architecture/agents/handover-contracts.md`.
- Compare against the canonical artifact library in `specs/architecture/agents/agent-artifacts/`.
- Compare against the consuming validator, schema, or runtime code path.
- Compare against sibling handoff artifacts for matching identifiers, counts, motion facts, manifests, and revision metadata.
- If the file participates in review routing, compare it against the stage-specific manifest and review artifact rules.

## What Good Looks Like

- The file is minimal but sufficient for the target role.
- The file exercises the intended behavior instead of bypassing it.
- The file is specific enough that invalid or inconsistent outputs are obvious in review.
- The file stays stable across the exact seeded revision and does not drift under downstream interpretation.
- For negative evals, the seed still passes deterministic hard checks and fails later for substantive reasons.
