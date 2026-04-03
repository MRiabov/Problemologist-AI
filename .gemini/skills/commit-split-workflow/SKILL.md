---
name: commit-split-workflow
description: Manual trigger only. Use this skill only when the user explicitly asks for `$commit-split-workflow` (or clearly requests this exact workflow) to split changes into meaningful commits with the repository's commit-message conventions.
---

# Commit Split Workflow (Manual Only)

Use this skill only when explicitly requested by name.
Do not auto-trigger for general git or commit tasks.

## Goal

Split the current git changes into small, reviewable groups, then commit each group separately with a message that makes the intent obvious later.

The goal is not to "commit everything quickly".
The goal is to produce a commit history that lets the reviewer answer:

- what changed in this commit
- why it changed
- which files belong together
- what is already done versus still pending

## Commit message naming rules

Use these prefixes when applicable:

- `(spec)`: for editing markdown spec files (especially spec-kitty files).
- `(refactor)`: internal refactors with no user-facing behavior change.
- `(debug)`: debugging and bug-fix work.
- `(test)`: test coverage additions/fixes.
- `(devops)`: infra/deployment config changes (Kubernetes, Vercel, etc.).

For user-facing UI/backend feature commits, do not prefix.

Examples:

- `(spec) Specified spec 025`
- `(spec) Planned for spec 025`
- `(refactor) Make webhook logic more modular`
- `(debug) Fix test failure in submission routing`

## Process

1. Inspect `git status` and split changes by intent/meaning.
2. Build a commit map before staging anything: group related hunks by behavior, feature, refactor, test, spec, or devops work.
3. If a file contains multiple unrelated changes, split it by hunk instead of forcing the whole file into one commit.
4. Stage and commit each logical group separately.
5. Give each commit a descriptive message that says what changed, not a generic placeholder.
6. If a change lives in a nested repo/submodule, commit that repo's change as part of the split instead of leaving it dirty.

Commit grouping rules:

- Group by intent, not by file count.
- Do not merge unrelated fixes into one "cleanup" commit.
- Keep tests with the behavior they verify when possible.
- Keep spec or docs updates separate when they describe the work rather than implement it.
- Do not skip markdown changes. If markdown files are present, include them in the relevant logical commit or a dedicated spec/doc commit.
- If a change set still reads as "and also" when described, it probably needs another split.

Commit message rules:

- Use the existing prefixes when they fit.
- Prefer short, concrete titles over broad summaries.
- Name the result or behavior, not the implementation mechanics.
- Make it easy to match a commit message back to the grouped files.

## Guardrails

- Do not stage throwaway scripts. Watch especially for names like:
  - `repro_*`
  - `debug_*`
  - `verify_*`
  - `check_*`
- Do not leave submodule changes uncommitted if they are part of the current work. Treat them like any other logical change group and commit them in the nested repo before finalizing the parent repo.
- If found (especially at repo root), call them out to the user and suggest moving them to `/scripts/throwaway/`.

## Quality gate

Before committing, prefer running:

- `pre-commit run --all-files`

This avoids hook failures and auto-fixes formatting/lint issues up front.

## Final response requirement

Always include the final commit names in the summary.
Also include a short mapping from each commit name to the change group it covered so the reviewer can tell what was done in each commit.
