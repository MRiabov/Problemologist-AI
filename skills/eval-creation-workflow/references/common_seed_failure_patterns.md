# Common Seed Failure Patterns

Use this as a quick triage list when a seeded eval fails before LLM evaluation.

## Frequent Failures

- wrong file set for the role
- stale revision evidence
- placeholder text left in a seeded file
- schema drift between planner handoff files
- missing visual inspection or review manifest
- review file present under the wrong stage name
- derived numeric fields not materialized exactly

## First Fix Rule

Fix the smallest source-level mismatch first.
Do not weaken validation to let a broken seed pass.

## Suggested Order

1. Compare the row against `references/role_input_index.md`.
2. Check the file-specific acceptance criteria for the file that failed.
3. Repair the exact source file, not the validator.
4. Re-run the narrowest passing slice.
