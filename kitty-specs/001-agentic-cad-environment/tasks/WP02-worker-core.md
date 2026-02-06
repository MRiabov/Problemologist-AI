---
work_package_id: "WP02"
title: "Worker Core (FS & Runtime)"
lane: "planned"
dependencies: ["WP01"]
subtasks: ["T007", "T008", "T009", "T010", "T011"]
---

# WP02: Worker Core (FS & Runtime)

**Goal**: Implement the sandboxed S3-backed filesystem and secure Python runtime execution in the Worker.

## Context

The Worker container is stateless. Its "filesystem" is an S3 bucket (MinIO during dev).
Agents interact with this FS. `deepagents` provides patterns for this.

## Subtasks

### T007: Implement S3 Connection

File: `src/worker/filesystem/db.py` (or similar)

- Use `fs-s3fs` or `boto3`.
- Configure via env vars from `docker-compose.yml`.
- Verify connection on startup.

### T008: Implement SandboxFilesystemBackend

File: `src/worker/filesystem/backend.py`

- Implement a class wrapping the S3 operations.
- Methods: `ls`, `read`, `write`, `edit`.
- **Constraint**: Must handle "virtual" paths.
  - `/` in agent view maps to `s3://bucket/session_id/` in reality.
  - Implement a `SessionManager` to inject `session_id`.

### T009: Runtime Module

File: `src/worker/runtime/executor.py`

- Function: `run_python_code(code: str, env: dict) -> tuple[str, str, int]` (stdout, stderr, exit_code).
- Use `subprocess.run` (or `asyncio.create_subprocess_exec`).
- **Security**: Run as non-root user (should be enforced by Docker user, but double check).
- **Timeouts**: Enforce execution timeout (e.g. 30s defaults).

### T010: FileSystem Abstraction & Restrictions

File: `src/worker/filesystem/router.py`

- Implement restrictions:
  - `utils/` and `skills/` are **Read-Only**.
  - Block attempts to write to them.
- Populate `utils/` from a local source (git repo or container integration) if they are "read-only" but need to exist. *Actually, Spec says they are read-only.*
- **Strategy**:
  - `files/` (workspace) -> S3 (Read/Write).
  - `utils/` -> Local container disk (Read Only).
  - Overlay them into a unified view for `ls`.

### T011: Verify Dependencies

- Create a test script that imports `build123d` and `mujoco`.
- Run it using the `runtime` executor.
- Verify no `ImportError`.

## Verification

1. Write a file to "virtual" path `hello.txt`. Check MinIO browser to see it in the correct bucket path.
2. Try to write to `utils/forbidden.py`. Should raise PermissionError.
3. Run `print("hello")` via executor. Get "hello".
4. Run code that imports `build123d`.
