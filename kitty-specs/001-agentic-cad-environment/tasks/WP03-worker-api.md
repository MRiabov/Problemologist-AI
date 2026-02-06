---
work_package_id: WP03
title: Worker API & Client
lane: "done"
dependencies: [WP02]
base_branch: 001-agentic-cad-environment-WP02
base_commit: 332b881de4d065a33983144d2dfd87c173d38df6
created_at: '2026-02-06T11:26:22.006465+00:00'
subtasks: [T012, T013, T014, T015, T016]
shell_pid: "556670"
agent: "gemini"
reviewed_by: "MRiabov"
review_status: "approved"
---

# WP03: Worker API & Client

**Goal**: Expose Worker capabilities via strict OpenAPI and generate a client for the Controller.

## Context

The Controller communicates with the Worker *only* via this HTTP API.
Strict schema is required for reliable tool calling.

## Subtasks

### T012: Setup FastAPI App

File: `src/worker/app.py`

- Initialize `FastAPI`.
- Add Middleware (CORS, trusted hosts if needed).
- Define root endpoint (Health check).

### T013: Define Pydantic Models

File: `src/worker/api/schema.py`

Models:

- `ListFilesRequest(path: str)`
- `ReadFileRequest(path: str)`
- `WriteFileRequest(path: str, content: str)`
- `EditFileRequest(path: str, edits: list[EditOp])`
- `ExecuteRequest(command: str, timeout: int)`
- `ExecuteResponse(stdout: str, stderr: str, exit_code: int)`

### T014: Implement FS Endpoints

File: `src/worker/api/routes.py`

- `POST /fs/ls`: Calls backend `ls`.
- `POST /fs/read`: Calls backend `read`.
- `POST /fs/write`: Calls backend `write`.
- `POST /fs/edit`: Calls backend `edit`.

**Note**: All paths in requests are "virtual" (e.g. `src/main.py`). The route handler uses the backend that abstracts the S3/Local mapping.

### T015: Implement Runtime Endpoint

File: `src/worker/api/routes.py`

- `POST /runtime/execute`: Calls `runtime.run_python_code`.
- Handle timeouts (return 504 or structured error).

### T016: Generate Client

- Use `fastapi-openapi-generator` or similar, OR:
- Write a simple Python client in `src/controller/clients/worker.py`.
- **Requirements**:
  - `WorkerClient` class.
  - Methods corresponding to endpoints.
  - Uses `httpx` (async).
  - Handles URL configuration (Worker URL).

### T036: Implement schemathesis fuzzing

- Setup `schemathesis` to run against Worker API.
- Verify OpenAPI compliance and robustness of input handling.
- Integrate into CI/CD or local test suite.

## Verification

1. Start Worker: `uvicorn src.worker.app:app`.
2. Access `/docs` -> OpenAPI UI.
3. Test `ExecuteRequest` via docs: `print("test")`. Expect structured response.
4. Verify `WorkerClient` can connect to local worker.

## Activity Log

- 2026-02-06T11:26:22Z – gemini-cli – shell_pid=550696 – lane=doing – Assigned agent via workflow command
- 2026-02-06T11:32:50Z – gemini-cli – shell_pid=550696 – lane=for_review – Ready for review: Worker API implemented with FastAPI, including Pydantic schemas, FS and Runtime endpoints, async WorkerClient, and schemathesis fuzzing tests.
- 2026-02-06T11:34:09Z – gemini – shell_pid=556670 – lane=doing – Started review via workflow command
- 2026-02-06T12:14:34Z – gemini – shell_pid=556670 – lane=done – Review passed: Implemented FastAPI app with Structlog, Pydantic schemas, and FS/Runtime endpoints. WorkerClient generated. Unit tests and Schemathesis fuzzing (7 endpoints) passed.
