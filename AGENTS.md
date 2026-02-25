# Engineering Agent Guidelines

This file contains specific instructions and tips for software engineering agents (like Jules) working on this repository.

## Synchronization and Pre-commit

To minimize merge conflicts and ensure you are working with the latest code, you **MUST** run the update utility before submitting your changes.

### 1. Update to Latest
Before finalizing your work or submitting a Pull Request, run the following command to rebase your changes onto the latest `main` branch:

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

`./.agent/workflows/integration-tests.md`

before proposing or applying fixes.

Key workflow knowledge to apply from that document:
- Use `./scripts/run_integration_tests.sh` for integration runs; do not rely on plain `pytest` because required services/env setup may be missing.
- Treat integration tests as HTTP-level system tests (real endpoints, no internal mocks/monkeypatching of `controller.*`, `worker.*`, `shared.*`).
- Validate behavior at system boundaries (HTTP payloads, logs, DB/S3/events), and keep `INT-xxx` mapping/conventions intact.
