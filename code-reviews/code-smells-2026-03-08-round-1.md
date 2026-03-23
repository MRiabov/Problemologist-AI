# Code Review: Code Smells Round 1 (2026-03-08)

## 1. Fragile Cross-Worker Communication

**File**: `shared/utils/agent/__init__.py` -> `_call_heavy_worker`
**Description**: The architecture physically splits "light" and "heavy" operations. The `WorkerClient` in the controller correctly handles workspace bundling (base64 tarball) when calling the heavy worker. However, the `shared.utils.agent` module (used by agents inside their scripts) makes direct HTTP calls to the heavy worker *without* providing the workspace bundle.
**Risk**: In a truly distributed environment (separate containers without shared volumes), `simulate()` or `validate()` calls from within an agent's script will fail because the heavy worker won't have the necessary `script.py` or dependencies to execute the build.
**User Review**:
Yep, that's a critical error. It's also strange we didn't test it in integration tests as well.

______________________________________________________________________

## 2. Synchronous Blocking in Async Worker

**File**: `shared/utils/agent/__init__.py` -> `_call_heavy_worker`
**Description**: The `_call_heavy_worker` function uses a synchronous `httpx.Client` with a 300s timeout. When an agent runs a script on the light worker that calls `simulate()`, the light worker's thread is blocked for the entire duration of the simulation.
**Risk**: This severely limits concurrency. A few simultaneous simulations can exhaust the light worker's thread pool, leading to API timeouts for other agents trying to perform simple tasks like `read_file`.
**User Review**:

______________________________________________________________________

## 3. Redundant and Side-Effect-Prone Execution

**File**: `shared/workers/loader.py` -> `load_component_from_script`
**Description**: The loader uses `spec.loader.exec_module(module)` followed by an explicit `module.build()` call.
**Risk**: If an agent's script calls `build()` in its global scope or under an `if __name__ == "__main__":` block (that isn't guarded against the dynamic module name), the build logic runs twice on the heavy worker. Furthermore, since the script on the light worker *also* ran the build to trigger the proxy call, we have triple execution of the CAD logic.
**User Review**:

______________________________________________________________________

## 4. Incomplete DSPy ReAct Integration

**File**: `controller/agent/nodes/base.py` -> `_run_program`
**Description**: A `WorkerInterpreter` is initialized on line 861 but is never passed to the `dspy.ReAct` constructor.
**Risk**: DSPy's ReAct module is currently restricted to the provided tool list. If the intention was to allow agents to generate and test snippets of code using an interpreter (as suggested by the existence of `WorkerInterpreter`), that capability is currently "dead code" and unused.
**User Review**:

______________________________________________________________________

## 5. "Fake" Incremental Streaming Observability

**File**: `controller/agent/nodes/base.py` -> `stream_lm_history_live`
**Description**: The streaming logic polls `dspy_lm.history` every 0.2s. However, most LiteLLM/DSPy adapters only populate the history object *after* the full LLM response is received.
**Risk**: This violates the architectural mandate for "incremental streaming" of thoughts. Users in the UI will see a long "thinking" state followed by a sudden dump of the entire reasoning trace, rather than seeing the agent's thoughts emerge in real-time.
**User Review**:

______________________________________________________________________

## 6. Oversimplified Physics: Center-Distance Failure Check

**File**: `worker_heavy/simulation/genesis_backend.py` -> `_check_electronics_fluid_damage`
**Description**: The `ELECTRONICS_FLUID_DAMAGE` check uses a hardcoded 5cm Euclidean distance from the entity's *center* to detect fluid contact.
**Risk**: This is physically inaccurate for engineering CAD. A large flat PCB or a long motor cable might be "touched" by fluid far from its center, while a 5cm radius might trigger a "damage" failure for fluid that is actually nowhere near the surface of a small component. It fails the "real-world-like" simulation requirement.
**User Review**:

______________________________________________________________________

## 7. Broken "Calculated Pricing" Promise

**File**: `skills/manufacturing-knowledge/scripts/validate_and_price.py`
**Description**: The script only calculates stock/removed volumes and validates the schema. It does NOT actually compute the `estimated_unit_cost_usd` or `estimated_weight_g`. The `AssemblyDefinition` Pydantic model simply checks if the totals (manually provided by the agent) are within caps.
**Risk**: The Engineering Planner is forced to manually calculate costs and weights, which is exactly the "guessing" the architecture spec (WP01/WP04) aimed to eliminate. This leads to high estimation error rates and brittle plans.
**User Review**:

______________________________________________________________________

## 8. Distributed Lock Deficiency

**File**: `worker_heavy/api/routes.py` -> `HEAVY_OPERATION_LOCK`
**Description**: Simulation serialization is enforced via a local `asyncio.Lock` and a single-worker `ProcessPoolExecutor`.
**Risk**: If the `worker-heavy` service is scaled horizontally (multiple containers), the lock is not shared. This could lead to multiple Genesis/MuJoCo simulations running on the same physical host (if not careful with affinity) or oversubscribing shared resources like GPUs, violating the "only one simulation runs at a time" constraint.
**User Review**:

______________________________________________________________________

## 9. Missing Node Entry Validation Requirements

**File**: `controller/agent/node_entry_validation.py` -> `build_engineer_node_contracts`
**Description**: `ELECTRONICS_ENGINEER` and `ELECTRONICS_REVIEWER` only require `script.py`. They do not require the electrical planning artifacts (`assembly_definition.yaml` with the `electronics` section) produced by the `ELECTRONICS_PLANNER`.
**Risk**: The electronics implementation nodes might start executing without a valid electrical plan, leading to hallucinations or "starting from scratch" instead of following the intended design.
**User Review**:
