# Codebase Spec Drift Report

This report summarizes discrepancies between the current codebase and the specifications defined in `specs/desired_architecture.md`, `specs/frontend-specs.md`, and `specs/integration-tests.md`.

## 1. Agent Tools and Naming Drifts

The following naming discrepancies exist between the "Agent-native tools" section in `desired_architecture.md` (lines 2224+) and the implementation in `controller/agent/tools.py`:

| Spec Name | Implementation Name | Status |
|-----------|---------------------|--------|
| `ls_info` | `list_files` | Naming Drift |
| `read` | `read_file` | Naming Drift |
| `write` | `write_file` | Naming Drift |
| `edit` | `edit_file` | Naming Drift |
| `grep_raw` | `grep` | Naming Drift |
| `execute` | `execute_command` | Naming Drift |

### Missing Tools
The following tools are defined in the spec but missing from the implementation:
- `glob_info` (Common)
- `upload_files` (Common)
- `download_files` (Common)
- `preview_design` (Engineer/Benchmark)
- `get_docs_for` (Engineer/Benchmark)

### signature variations
- `validate_and_price`, `simulate`, and `submit_for_review` are missing from `get_engineer_tools` in `controller/agent/tools.py`. They are present in `get_benchmark_tools` but with different signatures/naming than the "Exact tools logic" (lines 2301+) section (e.g., `submit` instead of `submit_for_review` in the middleware).

## 2. Handover Validation and Artifacts

Drifts identified in `worker_heavy/utils/file_validation.py` and `desired_architecture.md`:

- **Mandatory `journal.md`**: The spec identifies `journal.md` as a mandatory initial file for almost all agents (lines 591+, 664+), but `validate_node_output` does not enforce its presence.
- **Missing `script.py` in Validation**: `script.py` is not listed as a required file for `coder` or `benchmark_coder` nodes in `validate_node_output`, despite being the primary execution artifact.
- **Missing `assembly_definition.yaml` in Coder Validation**: The `coder` node validation does not require `assembly_definition.yaml`, although it is a key artifact for engineers.
- **Lack of Node-Specific Validation**: `Reviewer` and `COTS Search` nodes lack explicit required file validation in the current implementation.

## 3. Integration Test Gaps and Logic Drifts

Drifts identified in `tests/integration/` against `specs/integration-tests.md`:

- **ID Collision**: `INT-004` is used for "Simulation serialization" in the spec but "Episode artifact persistence" in some code parts (e.g., `test_missing_p0.py`).
- **Assertion Gaps**:
    - `INT-005` (Mandatory artifact gate) does not verify `journal.md` presence.
- **Missing Tests (P0 Baseline)**:
    - `INT-012` (COTS search read-only behavior)
    - `INT-013` (COTS output contract)
    - `INT-016` (Review decision schema gate)
    - `INT-017` (Plan refusal decision loop)
    - `INT-024` (Worker benchmark validation toolchain)
- **Logic Drift / Instability**: `test_int_022_motor_overload_behavior` failed during execution due to a 90s timeout, suggesting either a performance regression in the Genesis backend or an inadequately small timeout for the environment.

## 4. Frontend Observations

Observations from `frontend/src/` compared to `specs/frontend-specs.md`:

- **3-Column Layout**: Implemented, but the default split values in `UnifiedGeneratorView.tsx` (33.3% and 75%) seem mathematically inconsistent (summing to > 100%) even if the library handles it.
- **Plan Approval**: The "Execution Plan Ready" card in `ChatWindow.tsx` allows "Confirm & Start" and "Request Changes", which aligns with the spec's requirement for plan approval.
- **Topology Selection**: Correctly implements 3 buttons for `FACE`, `PART`, and `SUBASSEMBLY` in `ModelViewer.tsx`.
- **Stop Button**: Correctly implemented and wired to `interruptAgent`.
- **Theme Persistence**: Light/Dark mode is implemented but the spec's "resize persistence" (INT-171) needs verification if it survives a full page reload (it uses `localStorage`, so likely OK).
