---
name: commit-split-workflow
description: Manual workflow for splitting a dirty git tree into meaningful commits with the repo's commit-naming conventions.
---

# Commit Split Workflow

Use this skill only when the user explicitly asks to split changes into commits or triggers `$commit-split-workflow`.

## Workflow

1. Inspect `git status` and group changes by meaning.
2. Stage and commit one logical group at a time.
3. Keep throwaway repro scripts out of commits unless the user explicitly wants them. Typical names include `repro_`, `debug_`, `verify_`, and `check_`.
4. Use these commit prefixes when they match the change:
   - `(spec)` for markdown spec updates, especially spec-kitty files
   - `(refactor)` for internal refactors with no user-facing change
   - `(debug)` for bug fixes and investigation-driven changes
   - `(test)` for test coverage or test fixes
   - `(devops)` for Kubernetes, Vercel, and similar infra files
5. Leave user-facing UI or backend feature commits unprefixed unless another prefix clearly fits.
6. Run pre-commit before committing when hooks are present.
7. If backend and frontend changes are mixed, commit backend-related changes first when hooks may rewrite derived artifacts.

## Commit Message Style

Keep commit titles short and specific. Examples:

- `(spec) Planned for spec 025`
- `(refactor) Make webhook logic more modular`
- `(debug) Fix the test failure in worker transport`
- `(test) Add coverage for commit splitting`
- `(devops) Update deployment config`
