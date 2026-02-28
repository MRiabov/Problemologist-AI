# Code Review - Feb 28, 2026 - Round 1

## Overview
This review identifies several architectural and implementation issues across the `controller`, `worker_heavy`, and `shared` modules. Key concerns include resource management, inefficient database usage, and brittle logic in the agent graph and simulation.

## Identified Issues & Recommendations

### 1. Resource Leak in `WorkerClient`/`SharedNodeContext`

**Component:** `controller/agent/nodes/base.py` and `controller/clients/worker.py`

**Issue:** `SharedNodeContext.create` is invoked in the factory function of every agent node (e.g., `planner_node`, `coder_node`). This method creates a new `WorkerClient` instance every time. While `WorkerClient` caches `httpx.AsyncClient` instances in `_loop_clients`, these are never automatically closed or shared across different node executions within the same session. This leads to a gradual accumulation of open HTTP clients.

**Recommendation:** Share a single `SharedNodeContext` (or at least the `WorkerClient`) across the entire agent run for a given `session_id`. Implement an explicit lifecycle management or use a global registry to reuse clients.

**User Review:** [Pending]

---

### 2. Inefficient Database Engine Lifecycle in COTS Search

**Component:** `shared/cots/runtime.py`

**Issue:** The `search_parts` function calls `create_engine(f"sqlite:///{db_path}")` on every invocation. In SQLAlchemy, creating an engine also creates a new connection pool. Repeatedly creating engines for every search query bypasses the benefits of connection pooling and adds unnecessary overhead.

**Recommendation:** Cache the SQLAlchemy engine instance based on the `db_path` to ensure that connection pools are reused across multiple search requests.

**User Review:** [Pending]

---

### 3. In-Place Mutation of Electronics Configuration

**Component:** `worker_heavy/simulation/loop.py`

**Issue:** In `_handle_wire_failure`, the code calls `self.electronics.wiring.remove(wire)`. This mutates the `electronics` section of the simulation configuration in-place. If the configuration object is reused for subsequent simulations or retries, the torn wire will remain missing, making the simulation state non-idempotent and potentially causing hard-to-debug side effects.

**Recommendation:** Treat configuration objects as immutable. Create a copy of the electronics configuration before modifying it, or track the state of "broken" components separately from the base configuration.

**User Review:** [Pending]

---

### 4. Hardcoded Heuristics in Motor Physics

**Component:** `shared/cots/parts/motors.py`

**Issue:** The `retrieve_cots_physics` function uses hardcoded constants to derive KP and KV gains from motor torque (`kp = torque / 0.2`, `kv = kp * 0.1`). These heuristics assume a fixed "saturation error" of 0.2 rad and a fixed damping ratio, which might not be appropriate for all motor types or simulation scenarios, leading to potential instability or unrealistic behavior.

**Recommendation:** Move these constants to a configuration file or include them as part of the COTS database metadata, allowing for part-specific physics tuning.

**User Review:** [Pending]

---

### 5. Brittle Graph Transitions and Task Filtering

**Component:** `controller/agent/nodes/coder.py`

**Issue:** The `coder` node's `_get_next_step` method uses a hardcoded list of `elec_keywords` (`["circuit", "wire", "electronics", "routing", "psu", "power"]`) to decide whether to skip a task. This is brittle; if a task description uses different terminology or if the list is incomplete, the coder might attempt to handle tasks it's not designed for, or skip valid ones.

**Recommendation:** Use more formal task categorization (e.g., tags or metadata added during the planning phase) instead of keyword-based filtering to route tasks between the coder and the electronics engineer.

**User Review:** [Pending]

---

### 6. Redundant Tensor/Numpy Conversions

**Component:** `worker_heavy/simulation/genesis_backend.py`

**Issue:** In `get_body_state`, the `_to_flat_list` helper function, which handles the conversion of Genesis's batched tensors to CPU-bound lists, is defined twice (at lines 708 and 772). This redundancy increases the maintenance burden and makes the function unnecessarily large.

**Recommendation:** Extract `_to_flat_list` into a private method or a utility function within the module to reduce duplication and improve readability.

**User Review:** [Pending]

## Conclusion
While the system is functional, these "smells" indicate areas where resource efficiency, robustness, and maintainability can be significantly improved. Addressing the resource leaks and redundant logic should be a priority for long-term stability.
