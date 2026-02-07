# Architecture Review: Controller vs. Worker Boundary

## 1. Executive Summary

The current implementation has significant deviations from the `garbage-specs/desired_architecture.md`. The most critical discrepancy is the location of the **Agent Logic** and the **Filesystem Management**. Currently, the "Agent" runs entirely within the Controller process, manipulating an S3-backed virtual filesystem. The Worker is treated primarily as a stateless remote execution service (Function-as-a-Service), rather than a stateful container where the agent "lives".

## 2. Comparison Matrix

| Feature | Desired State (Spec) | Current Implementation | Status |
| :--- | :--- | :--- | :--- |
| **Agent "Home"** | Agents "live" directly in the filesystem of the container assigned to them (Worker). | Agents run in `controller` process. | 🔴 Critical Violation |
| **Filesystem** | Local/Mounted volume on Worker. "Edits sent directly to container". | S3-backed virtual FS (`SandboxFilesystemBackend`). Accessed directly by Controller. Shared via S3. | 🔴 Critical Violation |
| **Code Execution** | "Executes the python scripts... on the worker node". | `validator_node` sends script content string to Worker API. Worker executes via `exec()` or `importlib`. | 🟡 Partial Compliance |
| **Git Management** | "git commit & git push directly from workers". | `validator_node` (Controller) pulls S3 files to local `/tmp` and commits them. | 🔴 Violation |
| **Tool Execution** | "Controller parses tool call... sends request to worker to execute". | Controller executes `write`/`edit` tools locally against S3 backend. | 🔴 Violation |
| **Generators** | Logic split or in Worker. | `benchmark` generator logic resides in `controller/agent/benchmark`. | 🔴 Violation |

## 3. Detailed Findings

### 3.1. Agent Logic Location

**Spec**: "The Agent (managed by LangGraph) never 'knows' about distributed workers. It only calls an async Python function (a tool). It is the job of the tool to dispatch a job."
**Reality**:

- The `benchmark` agent is defined in `controller/agent/benchmark/nodes.py`.
- It uses `SandboxFilesystemBackend` which runs *locally* in the Controller process (communicating with S3).
- The "tools" for file manipulation (`write`, `edit`) are executed by the Controller.
- **Impact**: The Controller carries the burden of file management and state maintenance, rather than just orchestration.

### 3.2. Filesystem & state

**Spec**: "The file writes don't persist in the Controller... Their edits are sent directly into the container mounted (ephemeral) storage."
**Reality**:

- `SandboxFilesystemBackend` maps paths to `s3://bucket/session_id/...`.
- Both Controller and Worker (via `routes.py` -> `create_filesystem_router`) use this S3 backend.
- The consistency model relies on S3.
- **Critical Issue**: The Worker's execution model (`_load_component` in `routes.py`) relies on **local filesystem presence** or **direct string injection**.
  - If `script_content` is passed (as `validator_node` does), it works for single files.
  - If the agent creates multiple files (e.g., `utils.py`), the Worker execution will fail because `import utils` looks for a local file, but the file exists only in S3. The Worker does not automatically sync S3 to its local disk before execution.

### 3.3. Git Integration

**Spec**: "git commit & git push directly from workers."
**Reality**:

- `validator_node` in `controller/agent/benchmark/nodes.py` performs manual synchronization:

    ```python
    backend._fs.get(s3_root, str(local_repo_path), recursive=True)
    commit_all(local_repo_path, ...)
    ```

- This happens on the **Controller** machine (in `/tmp`), creating a bottleneck and violating the distributed nature of the spec.

### 3.4. Dependency Leakage

- `controller/agent/benchmark/nodes.py` imports `worker.filesystem.backend` and `worker.utils.git`.
- This confirms a monolithic structure where Controller is tightly coupled to Worker's implementation details, rather than interacting solely via API/RPC.

## 4. Recommendations

1. **Move Filesystem to Worker**:
    - The `SandboxFilesystemBackend` should be wrapped in a `RemoteFilesystemBackend` in the Controller.
    - When the Agent calls `write_file`, the Controller should make an RPC call (`client.write_file`) to the Worker.
    - The Worker should write to its **Local Filesystem** (Docker volume), not S3 (immediately).
    - S3 Sync should happen asynchronously or upon explicit "Snapshot/Commit" commands, performed by the Worker.

2. **Fix Git Management**:
    - Expose a `/git/commit` endpoint on the Worker.
    - The Controller should call this endpoint to trigger a commit on the Worker's local filesystem.

3. **Decouple Execution**:
    - The `validator_node` should not pass `script_content`. It should pass `script_path`.
    - Since the Agent would have already "written" the file to the Worker (via step 1), the file would exist locally on the Worker.
    - `api_simulate` should then just run the local file. This supports multi-file projects naturally.

4. **Refactor Generators**:
    - Move `benchmark` generator logic to `worker/generators` (if the intention is for the *Code* of the generator to be there) OR keep the *Graph* in Controller but ensure all *Nodes* delegate 100% of work to Worker. Currently, `planner` and `coder` do too much "local" work (S3 manipulation).

## 5. Conclusion

The current architecture implements a "Cloud State" (S3) model rather than a "Distributed Worker State" model. While S3 provides durability, the lack of local synchronization on the Worker breaks the ability to run complex, multi-file agents and puts unnecessary load/logic on the Controller. Aligning with the spec by moving FS operations to the Worker Node will resolve these issues.
