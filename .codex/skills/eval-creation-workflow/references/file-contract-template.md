# File Contract Template

Use this as a fallback only for new eval artifact types that are not listed yet.

## Role of the File

Describe what the file owns in the seeded workspace and which stage writes it.

## Acceptance Criteria

- The file has the expected name and stage ownership.
- The file schema-validates or parses as expected.
- The file is internally consistent with the rest of the handoff package.
- The file contains no placeholders or cross-stage drift.

## Review Questions

- Is the file present for the right role?
- Does it match the live validator and the role input index?
- Are all exact identifiers grounded in other handoff artifacts?
