# Spec Drift and Logic Inconsistency Report

This report summarizes discrepancies between the current codebase and the specifications defined in `specs/desired_architecture.md`, `specs/frontend-specs.md`, and `specs/integration-tests.md`.

## 1. Agent Roles and Permissions

| Area | Current Implementation | Specification Requirement | File(s) |
|---|---|---|---|
| **Role Naming** | `engineer_coder` used in nodes and config. | `Mechanical Engineer` or `Engineering CAD implementer`. | `controller/agent/nodes/coder.py`, `config/agents_config.yaml` |
| **Role Redundancy** | Both `engineer_coder` and `engineering_mechanical_coder` exist with identical permissions. | Single role per responsibility. | `config/agents_config.yaml` |
| **Factory Mismatch** | `reviewer_node` (benchmark) uses `engineering_reviewer` role. | Should use `benchmark_reviewer` role. | `controller/agent/benchmark/nodes.py` |
| **Missing Roles** | `benchmark_reviewer`, `cots_search_subagent`, `skill_creator_learner` missing from config. | Centralized policy for all roles. | `config/agents_config.yaml` |

## 2. Agent-Native Tooling

| Tool Category | Current Implementation | Specification Requirement | File(s) |
|---|---|---|---|
| **Naming Drift** | `list_files`, `read_file`, `write_file`, `execute_command`. | `ls_info`, `read`, `write`, `execute`. | `controller/agent/tools.py` |
| **Missing Tools** | `glob_info`, `upload_files`, `download_files` not exposed to ReAct. | Mandatory agent-native tools. | `controller/agent/tools.py` |
| **Logic Mismatch** | Benchmark `simulate`/`validate` take `script_path` (str). | Spec implies taking `Part` or `Compound` objects. | `controller/agent/benchmark/tools.py`, `controller/agent/tools.py` |
| **Engineer Tools** | `simulate` and `validate` missing from ReAct toolset. | Engineer has access to `simulate(Compound)` and `validate_and_price`. | `controller/agent/tools.py` |
| **COTS Search** | Returns ID, Cost, Weight, Recipe. | Should also return `manufacturer` and `why it fits`. | `shared/cots/agent.py` |

## 3. Observability and Events

| Requirement | Status | Finding | File(s) |
|---|---|---|---|
| **INT-026 Coverage** | Partial | `PlanSubmissionBenchmarkEvent` is defined but never emitted in the benchmark workflow. | `controller/agent/benchmark/nodes.py` |
| **Event Linkage** | Incomplete | `user_session_id` is present in schemas but not consistently propagated through all worker middleware calls. | `controller/middleware/remote_fs.py` |

## 4. Integration Test Mapping (Logic Drift)

| Test ID | Drift Type | Description | File(s) |
|---|---|---|---|
| **INT-004** | Collision | ID reused for "Simulation serialization" (arch) and "Episode artifact persistence" (missing). | `test_architecture_p0.py`, `test_missing_p0.py` |
| **INT-005** | Collision | ID reused for "Mandatory artifacts gate" (arch) and "Trace realtime broadcast" (missing). | `test_planner_gates.py`, `test_missing_p0.py` |
| **INT-011** | Redundancy | Dual implementations in `test_missing_p0.py` and `test_planner_gates.py`. | `test_missing_p0.py`, `test_planner_gates.py` |
| **Assertions** | Incomplete | Many tests skip assertions against `events.jsonl` or specific failure enums required by the spec. | `tests/integration/architecture_p0/` |

## 5. Frontend Implementation

| Feature | Current Implementation | Specification Requirement | File(s) |
|---|---|---|---|
| **Layout Ratio** | Fixed 320px Sidebar + 33%/66% split. | 3:3:6 column split. | `frontend/src/components/layout/AppLayout.tsx` |
| **Selection Modes** | FACE, PART, SUBASSEMBLY. | Missing explicit VERTEX and EDGE selection primitives. | `frontend/src/components/visualization/ModelViewer.tsx` |
| **Mentions** | start-end line mentions supported. | Mentions for CAD entities and subassemblies also required in chat input. | `frontend/src/components/Chat/ChatInput.tsx` |
