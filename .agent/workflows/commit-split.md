---
description: Commit changes in individual commits.
---

Split git status by meaning and commit it.
When doing commit names, I use the following rules:
Rules that I use:
(spec) - for editing markdown files, in particular spec-kitty files.
(refactor) - refactoring; no user-facing changes were done but internally we refactored code.
(debug) - debug the code.
(test) - add test coverage/fix existing ones.
(devops) - editing Kubernetes, vercel, etc files.

Any UI or backend (user-facing features) are not prefixed.

Examples of commit names:

- (spec) Specified spec 025 (when edited spec.md at spec 025)
- (spec) Planned for spec 025
- (refactor) Make webhook logic more modular
- (debug) fix the test failure at [...]

In addition, don't stage throwaway scripts. If (most commonly in backend) a script has names like "repro_", "debug_...", "verify_...", "check_"... and they are especially in root repo, it's likely intermediate, throwaway reproduction scripts that I don't need. Notify me about them. Maybe put them into `/scripts/throwaway/` folder.

NOTE: please skip the planning mode and commit directly. It's an easy task.

NOTE 2: we have pre-commit hooks. I suggest running `pre-commit run --all-files` before this command; you'll have everything formatted and autofixed; otherwise checks will fail and you'll have to do it manually.
