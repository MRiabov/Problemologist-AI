# Code Review: Spec Drift & Architectural Alignment (March 6, 2026)

## 1. Major Inconsistencies (Spec Drift)

### Filesystem Permission Drift (`config/agents_config.yaml`)

- **The Issue:** The `engineer_coder` (CAD Implementer) is currently granted `write` access to `plan.md`, `benchmark_definition.yaml`, and `assembly_definition.yaml`.
- **The Spec:** Section `Filesystem -> agents_config.yaml` explicitly states these are "control" files owned by the Planner. The Coder should only have `read` access to them. Allowing the Coder to modify the plan or objectives breaks the "Architect vs. Implementer" hierarchy.
- **Impact:** Agents may "cheat" by lowering constraints (e.g., increasing `max_unit_cost`) in `benchmark_definition.yaml` rather than solving the engineering problem.

### COTS Search Implementation

- **The Issue:** The spec (Section `COTS search subagent -> Tools`) calls for "Read-only SQL queries against the COTS catalog database" by a lightweight model.
- **The Reality:** The implementation in `shared/cots/agent.py` uses a full DSPy `ReAct` agent and a structured Python search library (`search_parts`). While more robust, it adds overhead and deviates from the "lightweight SQL lookup" intent.

### Agent Role Naming Discrepancies

- **The Issue:** `shared/enums.py` and `agents_config.yaml` use `engineer_coder` and `electronics_engineer`.
- **The Spec:** The documentation frequently refers to these as `CAD Engineer` or `Engineer Implementer`. This causes minor confusion in telemetry and prompt engineering.

## 2. Strategic Alignment (Correct Implementation)

### Worker Service Topology

The split between `worker-light` (FS, Git, Runtime) and `worker-heavy` (Simulation, Rendering, DFM) is perfectly implemented according to the spec, including the Temporal activity workers for long-running physics tasks.

### Physics & Verification Logic

- **Motor Overload:** The 2.0s clamp threshold in `worker_heavy/simulation/evaluator.py` matches the spec exactly.
- **Immutability:** Git-based hash assertions are correctly implemented in `worker_heavy/utils/file_validation.py` to prevent tampering with benchmark baselines.
- **Fastener Logic:** The `fastener_hole` utility in `shared/utils/fasteners.py` correctly implements the `bd-warehouse` integration and `RigidJoint` naming convention required for CAD stability.

### Handover Validation

The `validate_node_output` function enforces the presence of required markdown sections and the absence of template placeholders (`[x, y, z]`, etc.) before a node can transition.

## 3. Minor Observations

- **Skill Agent Permissions:** The `skill_agent` in `config/agents_config.yaml` is allowed to write to `journal.md`, but the spec suggests it should primarily focus on updating the `skills/` directory.
- **Electronics Split:** The architecture correctly implemented the split `electronics_planner` and `electronics_engineer` to handle the complexity of 3D wiring and circuit validation, as requested in the "Actual implementation" section of the spec.

## 4. Actionable Recommendations

1. **Surgical Update to Permissions:** Restrict `engineer_coder` and `benchmark_coder` to `read` only for `.yaml` and `.md` files (except `journal.md` and `todo.md`).
2. **Harmonize Naming:** Update `AgentName` enums to match the "Implementer" nomenclature used in the architectural diagrams to reduce cognitive load during prompt review.
3. **Validate COTS reproducibility:** Ensure the `catalog_snapshot_id` and `cots_query_id` are being strictly persisted in `assembly_definition.yaml` during every search, as the code suggests it's implemented but the spec marks it as a high-priority reproducibility requirement.
