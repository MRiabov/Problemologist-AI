---
work_package_id: WP02
title: Worker Core (FS & Runtime)
lane: "done"
dependencies: [WP01]
base_branch: 001-agentic-cad-environment-WP01
base_commit: 38f54c08d08793fe06905d56119097be4aec0653
created_at: '2026-02-06T09:48:51.142814+00:00'
subtasks: [T007, T008, T009, T010, T011]
shell_pid: "546162"
agent: "Gemini"
reviewed_by: "MRiabov"
review_status: "approved"
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

- Implement a class wrapping the S3 operations using `deepagents`'s `SandboxFilesystemBackend` pattern.
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
  - `utils/`, `skills/`, and `reviews/` are **Read-Only**.
  - Block attempts to write to them.
- Populate `utils/` and `skills/` from local source.
- **Strategy**:
  - `files/` (workspace) -> S3 (Read/Write).
  - `utils/`, `skills/`, `reviews/` -> Local container disk (Read Only).
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

## Activity Log

- 2026-02-06T09:48:51Z – Antigravity – shell_pid=518871 – lane=doing – Assigned agent via workflow command
- 2026-02-06T11:13:12Z – Antigravity – shell_pid=518871 – lane=done – Manually moved to done as requested by user.
- 2026-02-06T11:19:04Z – Gemini – shell_pid=546162 – lane=doing – Started review via workflow command
- 2026-02-06T11:25:56Z – Gemini – shell_pid=546162 – lane=done – Review passed: S3 backend, runtime executor, and filesystem router with read-only restrictions are correctly implemented. Dependency verification for build123d and mujoco confirmed. All tests passed.
