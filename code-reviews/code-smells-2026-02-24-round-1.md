# Code Review - Feb 24, 2026 - Round 1

## Overview
This review identifies architectural and implementation "code smells" across the Problemologist codebase, focusing on efficiency, consistency, and alignment with the system's stated goals.

## Identified Issues

### 1. Inefficient Event Loop Management in `BaseNode`
**File:** `controller/agent/nodes/base.py`
The `_get_tool_functions` method creates a new `asyncio` event loop for every asynchronous tool call made by the DSPy ReAct program. This is extremely inefficient and can lead to resource leaks or conflicts if multiple loops are active.
**Recommendation:** Use `asyncio.get_event_loop()` or `asyncio.run_coroutine_threadsafe` if running in a separate thread, or ensure the entire tool execution context is properly managed within a single loop (e.g., using `nest_asyncio`).
**User Review:** [Pending]

### 2. Misleading Tool Naming: `execute_command`
**File:** `controller/agent/tools.py`, `controller/middleware/remote_fs.py`
The tool named `execute_command` has a docstring stating it "executes a shell command", but the implementation actually expects raw Python code and calls `execute_python` on the worker. This is misleading for agents and can lead to failures if they attempt to run actual shell commands (like `ls` or `pip`).
**Recommendation:** Rename the tool to `execute_python` and update the docstring to accurately reflect its function.
**User Review:** [Pending]

### 3. Inconsistent Error Handling in `get_episode_schematic`
**File:** `controller/api/routes/episodes.py`
The `get_episode_schematic` endpoint returns an empty list `[]` upon any exception. This masks failures (like missing files or schema validation errors) and violates the contract expected by integration tests and robust frontends which may expect a 500 error with details.
**Recommendation:** Raise an `HTTPException` (500) with relevant details/traceback instead of returning an empty list.
**User Review:** [Pending]

### 4. Implementation Gap: `GenesisBackend` Features
**File:** `worker_heavy/simulation/genesis_backend.py`
The `GenesisBackend` currently lacks implementations for `set_site_pos` and `get_site_state`, which are present in the `MuJoCoBackend`. Additionally, it does not cache `get_contact_forces` results per simulation step, leading to potentially redundant and expensive calls to the physics engine during collision checks.
**Recommendation:** Implement `set_site_pos` and `get_site_state` to match MuJoCo functionality, and add a per-step cache for contact forces.
**User Review:** [Pending]

### 5. Inefficient HTTP Client Creation in Episode Routes
**File:** `controller/api/routes/episodes.py`
Endpoints like `get_episode_asset` and `get_episode_schematic` create new `httpx.AsyncClient` or `WorkerClient` instances for every request. This prevents connection pooling and increases overhead for high-frequency requests.
**Recommendation:** Use a shared `httpx.AsyncClient` (e.g., managed via app state or a global dependency) for all proxy and worker communication.
**User Review:** [Pending]

### 6. Simplistic Schematic Generation
**File:** `shared/schematic_utils.py`
The `generate_schematic_soup` function assumes all components have exactly two pins and uses a "risky" fallback in `get_schematic_pin_index` that defaults to pin "1" for unknown terminal names. This will produce incorrect schematics for more complex components (relays, connectors, etc.).
**Recommendation:** Implement a more robust terminal-to-pin mapping and support components with varying pin counts.
**User Review:** [Pending]

### 7. Rigid and Potentially Inefficient Agent Graph
**File:** `controller/agent/graph.py`
The agent graph enforces a linear flow: `planner -> electronics_planner -> plan_reviewer`. This results in an unnecessary `electronics_planner` LLM call even for tasks that have no electronic requirements.
**Recommendation:** Introduce conditional logic after the `planner` node to skip the `electronics_planner` if it's determined that the task is purely mechanical.
**User Review:** [Pending]

### 8. Mock LLM Tool Command Mismatch
**File:** `controller/agent/mock_llm.py`
The `MockDSPyLM` generates `execute_command` tool calls using `python3 -c` shell wrappers. However, the actual `execute_command` tool expects raw Python code. This mismatch causes failures in integration tests that rely on mock execution.
**Recommendation:** Update `MockDSPyLM` to provide raw Python code in the `command` argument for `execute_command`.
**User Review:** [Pending]

### 9. Handover Event Noise
**File:** `controller/agent/nodes/electronics_engineer.py`
The `ElectronicsEngineerNode` records an `ElecAgentHandoverEvent` at the start of its execution, even if it immediately determines there are no electronics tasks and returns the state unchanged. This pollutes the event log with "empty" handovers.
**Recommendation:** Move the handover event recording to after the node has confirmed there is work to be done.
**User Review:** [Pending]

### 10. Logic Gap in `should_continue`
**File:** `controller/agent/graph.py`
The `should_continue` routing logic does not explicitly handle the `AgentStatus.FAILED` state. If a node fails (e.g., due to a system error), it may fall through and loop back to the `coder` node if `iteration < 5`, potentially causing infinite loops or redundant retries of failed system states.
**Recommendation:** Explicitly handle `AgentStatus.FAILED` in `should_continue` to either terminate or go to a specialized recovery/skills node.
**User Review:** [Pending]

## Conclusion
The codebase is functionally rich but contains several areas where efficiency and consistency can be improved, particularly around agent-worker communication and simulation backend parity. Addressing these "code smells" will improve the system's scalability and reliability.
