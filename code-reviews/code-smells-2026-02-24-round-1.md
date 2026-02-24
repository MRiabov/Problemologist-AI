# Code Review - Round 1 (2026-02-24)

## Architectural Issues

### 1. Fragile Async/Sync Bridge and Test Instability
The codebase makes extensive use of `asyncio.new_event_loop()` and `run_until_complete()` inside synchronous wrappers for asynchronous tools (e.g., in `BaseNode._get_tool_functions` and `WorkerInterpreter.__call__`). This is often combined with `nest_asyncio.apply()`.
**Impact:** This is a major architectural smell that is currently causing massive failures in the integration test suite ("RuntimeError: Runner.run() cannot be called from a running event loop"). It makes the system fragile and difficult to debug or extend.
**Recommendation:** Transition to a more robust bridge like `anyio.from_thread.run` if tools must remain synchronous, or ideally, update the agent orchestration (DSPy) to handle asynchronous tool calls natively.

**User Review:**

### 2. Monolithic and Non-portable Orchestrator Scripts
The `nexus_orchestrator.py` and `overnight_orchestrator.py` scripts are monolithic and contain hardcoded absolute paths (e.g., `/home/maksym/Work/proj/Problemologist/Problemologist-AI`).
**Impact:** These scripts will fail on any other machine or container environment. They also duplicate a significant amount of logic for test execution and Jules session management.
**Recommendation:** Use relative paths or environment variables for project roots. Consolidate shared orchestrator logic into a proper module (e.g., `controller/orchestration/`).

**User Review:**

### 3. Inefficient Context and Client Lifecycle Management
The `SharedNodeContext.create` method is called in every agent node factory (Planner, Coder, etc.), recreating the `WorkerClient`, `PromptManager`, and `httpx.AsyncClient` for every single turn.
**Impact:** This leads to significant overhead, potential socket exhaustion, and leaks `httpx` clients (especially with the loop-local client cache in `WorkerClient` which is never closed). `PromptManager` also re-renders templates frequently.
**Recommendation:** Implement a long-lived context or a singleton pattern for these shared services within the Controller's lifecycle.

**User Review:**

## Implementation Smells

### 4. Hardcoded Electronics Terminals Mismatch
`shared/circuit_builder.py` and `shared/pyspice_utils.py` hardcode terminal names like `_in`, `_out` for switches and `+`, `-` for motors.
**Impact:** If the CAD design or user-provided `objectives.yaml` uses different terminal names (e.g., `1`, `2` for a switch), the circuit builder will create floating, disconnected resistors, and validation will fail incorrectly. The `resolve_node_name` function also doesn't handle these special names consistently for switches.
**Recommendation:** Dynamically discover terminals from the `wiring` list in the `ElectronicsSection` instead of relying on hardcoded name conventions.

**User Review:**

### 5. Fragile TODO State Management in Agent Nodes
`CoderNode` and `ElectronicsEngineerNode` update the `todo` state by performing string replacements on the *original* state string, ignoring any changes the agent might have made to `todo.md` on the worker during its execution.
**Impact:** If an agent adds new tasks or modifies existing ones in `todo.md`, these changes are overwritten and lost when the node returns the "updated" state. Furthermore, the updated state is never synced back to the worker's filesystem.
**Recommendation:** Use the `artifacts` returned from `_run_program` (which contains the current state of `todo.md` on the worker) as the source of truth, and ensure the final state is synced back if modified by the node.

**User Review:**

### 6. Electronics Fallback Discrepancy
The `ElectronicsManager._fallback_update` implementation is a simple BFS that doesn't actually check switch states or handle terminal-specific connectivity, despite documentation and memory suggesting it does.
**Impact:** The fallback power calculation is inaccurate and will likely power components that should be disconnected by open switches.
**Recommendation:** Implement the terminal-based BFS as described in the project goals, respecting the `switch_states`.

**User Review:**

### 7. Missing Review Decision Mapping
`ExecutionReviewerNode` lacks mappings for some `ReviewDecision` values like `CONFIRM_PLAN_REFUSAL`, which defaults to `CODE_REJECTED`.
**Impact:** Unexpected agent behavior or incorrect state transitions when a reviewer confirms a plan refusal.
**Recommendation:** Explicitly handle all `ReviewDecision` enum values in the node's status map.

**User Review:**
