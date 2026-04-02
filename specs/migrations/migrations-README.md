# Migration Doc Guide

This directory contains implementation-facing migration specs. A migration doc
describes how the repo moves from an existing contract to a new contract.
It is not a general design essay, task list, or issue tracker.
This guide only defines how migration docs should be written; it does not
override `specs/desired_architecture.md` or any file under `specs/architecture/`.

## Scope

- Use a migration doc when the change alters a contract, runtime boundary,
  workspace shape, validation gate, or documented ownership rule.
- Write the doc in the `major/` folder when the change is cross-cutting or
  architecture-level.
- Write the doc in the `minor/` folder when the scope is narrower but still
  contract-bearing.
- Keep the filename descriptive and specific to the migration.

## Required Shape

Follow this order unless a migration has a strong reason to add a small
section of its own:

01. `# Title`
02. Optional status comment such as `<!-- Investigation doc. No behavior change yet. -->`
03. `## Purpose`
04. `## Problem Statement`
05. `## Current-State Inventory`
06. `## Proposed Target State`
07. `## Required Work`
08. `## Non-Goals`
09. `## Sequencing`
10. `## Acceptance Criteria`
11. `## Migration Checklist`
12. `## File-Level Change Set`

Optional follow-up sections may include `Open Questions`, `Risks`, `Test Impact`,
`Verified This Pass`, `Seed and Fixture Updates`, and `Completion Note`.

## Writing Rules

- Use declarative language for the contract sections: `Purpose`,
  `Problem Statement`, `Current-State Inventory`, `Proposed Target State`,
  `Non-Goals`, `Sequencing`, and `Acceptance Criteria`.
- Use action-oriented language for `Required Work` and the checklist, because
  those sections track implementation steps and verification status.
- Name the exact files, modules, routes, prompts, or artifacts that own the
  contract.
- Prefer concrete, testable statements over vague goals.
- Use plain Markdown only. Keep headers, lists, and tables simple.
- Use HTML comments only for temporary notes, investigation markers, or frozen
  reminders that are not part of the active contract.
- Keep repeated guidance minimal. If the same rule already appears elsewhere in
  the spec set, reference it instead of restating it.

## Section Guidance

### Purpose

- State the migration target in one short paragraph.
- Link the owning architecture spec or specs that define the desired state.
- Make clear whether the doc is an investigation, a planned implementation, or
  a completed migration record.

### Problem Statement

- Describe the current mismatch or failure mode.
- Explain why the current behavior is wrong for the target architecture.
- Keep the focus on the contract gap, not on coding steps.

### Current-State Inventory

- Use a table or compact bullet list.
- Include the relevant repo areas and the reason each one must change.
- Prefer exact file paths and runtime names over generic descriptions.

### Proposed Target State

- Write numbered, declarative outcomes.
- Each item should be verifiable.
- Make ownership and failure behavior explicit.
- Include only the state that the migration is meant to establish.

### Required Work

- Write these items as implementation tasks, not as passive system statements.
- Group work by contract area, plumbing, validation, docs, and tests.
- Use subsections when the migration spans several distinct surfaces.
- Keep the items bounded enough that each one can be checked off cleanly.

### Non-Goals

- State what this migration will not change.
- Use this section to prevent scope creep and accidental follow-on work.

### Sequencing

- List the safe implementation order when ordering matters.
- Keep the sequence short and practical.
- If ordering does not matter, say so directly.

### Acceptance Criteria

- Write concrete pass/fail statements.
- Tie each criterion to a visible contract, artifact, gate, or test.
- Use this section as the migration's success definition.

### Migration Checklist

- Use `[ ]` for pending items and `[x]` only when the item is actually done.
- Keep checklist items atomic and checkable.
- Put checklist items under topical subheadings when that makes the work easier
  to track.
- Mark an item complete only after the relevant integration evidence exists.
- If an item is no longer needed, leave a short written waiver instead of
  silently deleting the intent.
- Treat the checklist as the progress record for the migration.
- Phrase checklist items as concrete completion conditions, not aspirational
  goals.

### File-Level Change Set

- List the smallest realistic set of files that enforce the new contract.
- Include spec files, runtime files, prompt files, templates, tests, and seed
  fixtures when they are part of the change.
- If a file is only a temporary compatibility alias, say so explicitly.

## Completion Rule

When a migration is complete, add a short completion note and keep the doc
frozen unless a later architecture change reopens the scope. Do not keep
editing a finished migration as if it were a living task list.

## Preferred Style

- Use the same direct, fail-closed tone as the architecture docs.
- Keep the document readable at a glance.
- Be specific about contracts, but avoid unnecessary verbosity.
- Treat the migration doc as the implementation record for that migration; the
  architecture docs remain the source of truth for the target state.
