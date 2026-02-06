---
work_package_id: WP04
title: Controller & DeepAgents Integration
lane: "doing"
dependencies: [WP03]
base_branch: 001-agentic-cad-environment-WP03
base_commit: f674a15ba42a8a71761ab2d7ca73b4515d1b082b
created_at: '2026-02-06T11:50:04.690481+00:00'
subtasks: [T017, T018, T019, T020, T021]
shell_pid: "563842"
agent: "Gemini"
---

# WP04: Controller & DeepAgents Integration

**Goal**: Integrate the Worker Client into the Controller's `deepagents` middleware.

## Context

The Controller runs the LangGraph agent.
`deepagents` usually assumes local FS or Sandboxed FS. We need to implement a "Remote" FS middleware that proxies calls to our Worker.

## Subtasks

### T017: Setup Controller App

File: `src/controller/api/main.py`

- Initialize FastAPI app.
- Config: `WORKER_URL` from env (default `http://worker:8001`).

### T018: Internalize Worker Client

Copy/Ensure `src/controller/clients/worker.py` (from WP03) is available and functional.

### T019: RemoteFilesystem Middleware

File: `src/controller/middleware/remote_fs.py`

- Inherit from `deepagents.middleware.FilesystemMiddleware` (or implement protocol).
- **Core Logic**: Instead of local IO, call `WorkerClient`.
- **Constraint**: Must respect and enforce read-only path constraints (`skills/`, `utils/`, `reviews/`).
- Methods:
  - `list_files` -> `client.ls`
  - `read_file` -> `client.read`
  - `write_file` -> `client.write`
  - `run_command` -> `client.execute`

### T020: Define LangChain Tools

File: `src/controller/tools/fs.py`

- Create `StructuredTool` instances.
- `ls_tool`: Uses `RemoteFilesystem`.
- `read_tool`: Uses `RemoteFilesystem`.
- `write_tool`: Uses `RemoteFilesystem`.
- `exec_tool`: Uses `RemoteFilesystem`.

### T021: Setup Basic Agent Graph

File: `src/controller/graph/agent.py`

- Define a simple ReAct agent using LangGraph.
- Tools: [ls, read, write, exec].
- System Prompt: "You are a coding agent..." (Basic version).
- Test Endpoint: `POST /agent/run` (mock input).

## Verification

1. Start Controller (connected to Worker).
2. Invoke the agent via API/Script.
3. Ask it to "write a file named `test.py` with content `print('hello')`".
4. Verify file exists on Worker (minio).
5. Ask it to "run `test.py`".
6. Verify output "hello" returned to agent.

## Activity Log

- 2026-02-06T11:50:04Z – Gemini – shell_pid=563842 – lane=doing – Assigned agent via workflow command
