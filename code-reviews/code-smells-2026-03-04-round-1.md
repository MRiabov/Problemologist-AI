# Code Review: Architectural & Implementation Smells - March 4, 2026 (Round 1)

## 1. Global Scalability Bottleneck: `HEAVY_OPERATION_LOCK`
**Location:** `worker_heavy/api/routes.py`

**Description:**
The `worker_heavy` service utilizes a global `asyncio.Lock()` (`HEAVY_OPERATION_LOCK`) to serialize all major operations including `/benchmark/verify`, `/benchmark/simulate`, `/benchmark/validate`, `/benchmark/analyze`, and `/benchmark/preview`.

**Reasoning:**
While intended to prevent resource exhaustion (e.g., GPU memory or CPU saturation by Genesis/MuJoCo), a single global lock across all sessions is a massive scalability bottleneck. In a multi-user environment, one user's 30-second simulation will block all other users' previews or validations. This will lead to cascading HTTP timeouts in the controller and a poor user experience.

**Recommendation:**
Replace the global lock with a semaphore that allows a configurable number of concurrent operations based on available hardware resources, or move to a task queue (like Temporal, which is partially used but bypassed for "direct" calls) for all heavy operations to manage concurrency and priority more gracefully.

**User Review:**


---

## 2. Layer Violation: Domain Logic in Filesystem Middleware
**Location:** `controller/middleware/remote_fs.py` (`write_file` method)

**Description:**
The `RemoteFilesystemMiddleware.write_file` method contains hardcoded logic to detect writes to `assembly_definition.yaml`, parse the YAML content, and emit a `COTSSelectionEvent`.

**Reasoning:**
This is a classic leaky abstraction and layer violation. A filesystem middleware should be agnostic to the content of the files it manages. Injecting domain-specific logic (COTS selection) into a generic I/O proxy makes the middleware fragile and difficult to reuse or test in isolation. If the schema of `assembly_definition.yaml` changes, the middleware breaks.

**Recommendation:**
Move this event emission logic to the Agent nodes (e.g., `EngineerCoder`) or a dedicated `HandoverService` that monitors workspace state changes. Use an observer pattern if the middleware must remain involved.

**User Review:**


---

## 3. Tool Intent Mismatch: `execute_command` vs. `execute_python`
**Location:** `controller/agent/tools.py` and `controller/middleware/remote_fs.py`

**Description:**
The tool exposed to agents as `execute_command` has a docstring stating: "Execute a shell command in the workspace." However, the implementation calls `fs.run_command(command)`, which ultimately invokes `self.client.execute_python(code)`.

**Reasoning:**
There is a fundamental mismatch between the tool's name/description and its implementation. If an agent tries to run a shell command like `ls -R` or `pip install`, it will fail because the backend expects Python code. This leads to agent confusion, unnecessary retries, and fragile implementations where the agent "learns" to pass Python code to a tool named `execute_command`.

**Recommendation:**
Rename the tool to `execute_python` and update the docstring to clearly state that it expects Python code. If shell execution is required, implement a separate `execute_shell` tool.

**User Review:**


---

## 4. Type Inconsistency in `grep_raw`
**Location:** `shared/workers/filesystem/router.py`

**Description:**
The `FilesystemRouter.grep_raw` method returns a list of dictionaries: `{"path": virt_f, "line": line_num, "text": line}`. However, the `BackendProtocol` and the `WorkerClient` expect a list of `GrepMatch` (Pydantic model) objects.

**Reasoning:**
While Python's duck typing allows this to work in some contexts, it breaks when the results are passed to components expecting model attributes (e.g., `match.path` vs `match["path"]`). Indeed, previous code reviews (`march-3-review-round-2.md`) already noted a bug where `FileInfo` objects were treated as dicts. This inconsistency in `grep_raw` is a similar landmine.

**Recommendation:**
Ensure `grep_raw` consistently returns `GrepMatch` instances.

**User Review:**


---

## 5. Performance Anti-pattern: Redundant Context Instantiation
**Location:** `controller/agent/nodes/coder.py` (and other node factories)

**Description:**
Every time a node like `coder_node` is called, it executes `SharedNodeContext.create(...)`, which instantiates a new `WorkerClient`, `RemoteFilesystemMiddleware`, and `PromptManager`.

**Reasoning:**
This prevents the `WorkerClient` from utilizing HTTP connection pooling effectively, as a new `httpx.AsyncClient` is created (indirectly) or managed via a new client instance for every single turn of the agent. It also adds unnecessary CPU and I/O overhead for re-loading configs and templates on every turn.

**Recommendation:**
Pass the `SharedNodeContext` (or the individual clients) through the `AgentState` or utilize a dependency injection container that persists these objects for the duration of the LangGraph execution.

**User Review:**


---

## 6. Fragile Serialization: Un-sanitized `stress_fields`
**Location:** `worker_heavy/utils/validation.py`

**Description:**
The `simulate` function returns a `SimulationResult` containing `stress_fields`. While `stress_summaries` are sanitized for `NaN/Inf` values using `_sanitize_stress_summaries`, the `stress_fields` (which contain the raw node stresses) are not.

**Reasoning:**
Physics simulations frequently produce `NaN` or `Inf` values in corner cases (e.g., singular matrices or divergent FEM solutions). If these values reach the FastAPI response layer without being converted to null or finite floats, the JSON serialization will fail with a `ValueError`, resulting in a 500 Internal Server Error instead of a graceful failure message.

**Recommendation:**
Apply similar sanitization logic to `stress_fields` or ensure the physics backend/wrapper handles these values before populating the `SimulationResult`.

**User Review:**
