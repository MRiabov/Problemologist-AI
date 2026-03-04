# Engineering Agent Guidelines

This file contains specific instructions and tips for software engineering agents (like Jules) working on this repository.

## Synchronization and Pre-commit

### 1. Update to Latest
If you want to rebase your changes onto the latest `main` branch, run:

```bash
python3 scripts/update_to_latest.py
```

This script will:
- Fetch the latest changes from `origin`.
- Rebase your current branch onto `origin/main`.
- Automatically update dependencies (via `uv`) if `uv.lock` or `pyproject.toml` have changed.

### 2. Conflict Resolution
If the rebase fails due to conflicts, you are expected to resolve them autonomously if possible, or request user assistance for complex logical conflicts.

### 3. Verification
After any update or rebase, you must verify that the system is still stable by running relevant tests (e.g., `./scripts/run_integration_tests.sh` or specific unit tests).

## Integration Failures Triage

If the user pastes integration test failures, you **MUST** read:

`./.codex/skills/integration-tests-workflow/SKILL.md`

before proposing or applying fixes.

Key workflow knowledge to apply:
- Use `./scripts/run_integration_tests.sh` for integration runs; do not rely on plain `pytest` because required services/env setup may be missing.
- Treat integration tests as HTTP-level system tests (real endpoints, no internal mocks/monkeypatching of `controller.*`, `worker.*`, `shared.*`).
- Validate behavior at system boundaries (HTTP payloads, logs, DB/S3/events), and keep `INT-xxx` mapping/conventions intact.

## Typed Schema Skill Trigger

When the task adds or modifies a class, or plans/implements storage or API communication logic (function interfaces or HTTP endpoints), you **MUST** apply:

`./.agent/skills/classes-instead-of-dicts/SKILL.md`
