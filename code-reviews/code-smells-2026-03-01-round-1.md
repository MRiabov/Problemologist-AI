# Code Smells - March 1, 2026 - Round 1

## 1. Systemic Use of Bare `except:` and `except Exception: pass`
**Issue**: Despite guiding principles, the codebase is littered with bare `except:` and `except Exception: pass` blocks.
**Examples**:
- `controller/agent/nodes/base.py`: Multiple `with suppress(Exception):` and `except Exception: pass` when tracing tools or reading steer context.
- `worker_heavy/simulation/loop.py`: `except Exception: pass` when caching actuator limits.
- `shared/pyspice_utils.py`: `except Exception:` in the ngspice monkeypatch.
- `shared/workers/filesystem/backend.py`: Bare `except:` blocks.
**Impact**: This makes debugging extremely difficult as it masks real errors, including `KeyboardInterrupt` (in the case of bare `except:`) and unexpected `AttributeError` or `TypeError`.
**Recommendation**: Replace with specific exception types and at least log the exception using `structlog` if it must be suppressed. Use `contextlib.suppress` only for truly ignorable errors where the intent is documented.

**User Review**:

---

## 2. Brittle and Duplicated Task Selection Logic
**Issue**: `CoderNode._get_next_step` and `ElectronicsEngineerNode._get_next_electronics_step` use duplicated hardcoded keyword lists to route tasks.
**Code**:
```python
elec_keywords = ["circuit", "wire", "electronics", "routing", "psu", "power"]
```
**Impact**: High risk of misrouting tasks if the user uses different terminology (e.g., "Connect the motor to the battery"). It also violates DRY.
**Recommendation**: Centralize task classification into a utility or use the LLM to classify tasks during the planning phase, assigning a `category` or `target_agent` to each TODO item.

**User Review**:

---

## 3. Non-Idempotent Mutation of State in `SimulationLoop`
**Issue**: `SimulationLoop._handle_wire_failure` in `worker_heavy/simulation/loop.py` modifies the input `electronics` object by removing wires from its list.
**Code**:
```python
self.electronics.wiring.remove(wire)
```
**Impact**: The `electronics` object (often passed from the controller) is mutated in-place. If the simulation is retried or if the results are used to generate a report, the "initial" state is lost. This makes the simulation non-idempotent.
**Recommendation**: Treat the `electronics` configuration as immutable within the simulation. Track broken wires in a separate list within `SimulationLoop` or create a deep copy of the configuration before starting the simulation.

**User Review**:

---

## 4. Missing Open-Circuit Check Propagation in `calculate_power_budget`
**Issue**: `calculate_power_budget` in `shared/pyspice_utils.py` does not accept a `section` (ElectronicsSection) argument, meaning it cannot perform the proactive open-circuit checks implemented in `validate_circuit`.
**Impact**: Power budget analysis might return "Safe" for a completely disconnected circuit that `validate_circuit` would correctly identify as failing due to open circuits.
**Recommendation**: Update `calculate_power_budget` to accept an optional `section: ElectronicsSection` and pass it through to `validate_circuit`.

**User Review**:

---

## 5. Resource Leak in `SharedNodeContext`
**Issue**: `SharedNodeContext` in `controller/agent/nodes/base.py` creates a new `WorkerClient` for every node execution. `WorkerClient` manages an `httpx.AsyncClient`.
**Impact**: There is no mechanism to close these clients (no `aclose` call in the node lifecycle). This leads to an accumulation of open sockets and potential file descriptor exhaustion in the controller.
**Recommendation**: Implement an `aclose()` method on `SharedNodeContext` and ensure it is called by the agent runner, or use a singleton/cached client managed at the controller level.

**User Review**:

---

## 6. Duplicated Utility Functions in `GenesisBackend`
**Issue**: `_to_flat_list` is defined twice within `GenesisBackend.get_body_state` in `worker_heavy/simulation/genesis_backend.py`.
**Impact**: Redundant code and maintenance overhead.
**Recommendation**: Move `_to_flat_list` to a class method `self._to_flat_list` or a module-level utility function.

**User Review**:
