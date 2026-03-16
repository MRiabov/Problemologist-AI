# Integration Test Specification

## **CRITICAL NOTE TO IMPLEMENTATION AGENTS**

These workflows are integration workflows. Do **not** satisfy them with unit tests disguised as integration tests.
Do not mock project/runtime boundaries in integration tests; only unavoidable third-party instability may be isolated, and each such case must be justified in the test docstring.

**CONTROL THE TEST EXECUTION *ONLY* BY HTTP REQUESTS IN EVERY TEST. DO NOT DO TESTS THAT ARE EFFECTIVELY UNIT TESTS**
**FILE CREATION SHOULD BE MADE VIA TOOL CALLS. EXECUTION LOGIC SHOULD BE MADE VIA TOOL CALLS. EVERYTHING SHOULD BE MADE VIA HTTP REQUESTS, NOT VIA MOCKS AND UNIT TESTS!**

**ASSERT AGAINST *CONTAINERS* NOT AGAINST PYTHON**

*If you cannot assert something via HTTP calls, assert it via the `events.jsonl` event stream.*

Additionally:

1. `kitty-specs/desired_architecture.md` is the central source of truth for the application; all integration tests should be relevant to it and reviewed against it before implementation. Every added test must map to an INT-xxx ID from `kitty-specs/integration-tests.md`.
2. Prefer black-box integration (compose services + real boundaries); avoid heavy mocks except where unavoidable.
3. Include both happy-path and required fail-path assertions (especially gating/validation/refusal/event emission).
4. Do not silently drop existing integration coverage; replacements must preserve or improve mapped coverage.
5. Mark unimplemented IDs explicitly as deferred (P1/P2) instead of omitting them.

## Non-negotiable Integration Execution Contract (applies to every `INT-xxx`)

1. Test target is a running compose stack (`controller`, `worker`, `controller-worker`, infra services), not imported Python functions.
2. Test traffic goes through HTTP APIs only.
3. Files/artifacts are created by API/tool-call pathways, not by direct local writes to app internals.
4. No `patch`, `monkeypatch`, or fake clients for controller/worker/temporal/s3 paths in integration tests.
5. Assertions are on observable boundaries: HTTP responses, container logs, DB rows, object storage objects, emitted events.
6. Every test must include at least one expected-fail assertion where architecture specifies fail behavior.
7. Any unavoidable mock must be isolated to external third-party instability only, and must not mock project modules.
8. A test that imports app internals (`controller.*`, `worker.*`) to invoke business logic directly is **not** integration coverage.
9. Every `build123d` script used in tests must ensure every part has a `.metadata` attribute initialized with a `PartMetadata` or `CompoundMetadata` instance (imported from `shared.models.schemas`), following strict typing rules.
10. Every JSON, YAML, XML is converted to models e.g. Pydantic models and only then assertions are happening against them (for test maintainability and explainability). If there is no model for a JSON or similar schema, add it.
11. Tests must utilize state-based polling (e.g., waiting for specific API responses, DB states, or UI elements) rather than hardcoded sleeps or generic timeouts. This ensures tests are deterministic and fail-fast.

*Exception to the importing rules*: you can import python models and enums to use appropriate schema to avoid using pure json which will need to be manually updated later.
Commonly, these models and enums would be in `shared/` folder.

## Infrastructure & Helpers
The integration suite is designed for high-velocity local execution and CI parity.

- **`IS_INTEGRATION_TEST=true`**: This environment variable is automatically exported by the runner. It enables "smoke test mode" in simulation kernels, significantly reducing particle counts and simulation time for faster feedback.
- **`INTEGRATION_EARLY_STOP_ON_BACKEND_ERRORS`**: Defaults to `1` in `scripts/run_integration_tests.sh`. When enabled, the runner monitors machine-readable backend error JSON (`logs/integration_tests/current/json/*_errors.json`, with compatibility symlinks at `logs/integration_tests/json/*_errors.json`) for the first **non-allowlisted** error event and reports it; pytest then fails the active test through the backend-error fixture (no runner SIGINT injection). Matching uses global `BACKEND_ERROR_ALLOWLIST_REGEXES` and per-test `@pytest.mark.allow_backend_errors` patterns keyed by `session_id` `INT-xxx` prefixes. Set to `0` to disable this detector and keep full-duration runs.
- **`tests/integration/mock_responses/`**: Integration mode uses `MockDSPyLM` for agent-node LLM responses, and scenarios are loaded from this directory. Each scenario file must use strict `INT-###.yaml` naming (for example `INT-074.yaml`), and any other naming convention is rejected by a one-time integration startup assertion. Large `write_file` payloads may be extracted into adjacent fixture files via `content_file` references. When `INT-xxx` behavior depends on deterministic mock outputs, keep the corresponding scenario entries current.
- **`scripts/run_integration_tests.sh`**: The central entry point for the integration suite. It manages infrastructure spin-up (Docker), local service lifecycle, and `pytest` execution.
- **`logs/integration_tests/runs/run_<timestamp>/`**: canonical per-run log root. `logs/integration_tests/current/` points to the active/latest run, and compatibility symlinks remain at `logs/integration_tests/*.log`.
- **`logs/integration_tests/current/json/`**: machine-readable backend error files emitted as JSON Lines from native structlog event dicts (also reachable via compatibility symlinks in `logs/integration_tests/json/`):
  - `controller_errors.json`
  - `worker_light_errors.json`
  - `worker_heavy_errors.json`
  - `temporal_worker_errors.json`
  - `backend_error_allowlisted_prefixes.json` (auto-generated/cache file containing per-`INT-xxx` backend-error allowlist rules derived from test markers; consumed by integration-runner early-stop filtering)
- **Backend error-log attribution contract**: Structured `ERROR` lines written to dedicated backend error logs must include `session_id` or `episode_id` (ideally both) so strict integration teardown can attribute failures to the owning test context and avoid cross-session leakage.
- **`test_output/`**: Stores JUnit XML results and the persisted test history used for trend analysis.
- **Worker FS read/write permissions bypass mechanism (`agents_config.yaml` enforcement tests)**: privileged bypass of per-agent filesystem policy is enabled only when both are present in the same HTTP request: request payload flag `bypass_agent_permissions=true` and header `X-System-FS-Bypass: 1`. Header-only and payload-only requests must remain policy-enforced (no bypass). 
<!--Note: the bypass logic is to bypass permissions that restrict agents, but agents shouldn't be able to (obviously) use the bypass.-->

### Command Reference
```bash
# Run P0 architecture baseline
./scripts/run_integration_tests.sh -m integration_p0

# Run with high-fidelity simulation (disable smoke mode)
./scripts/run_integration_tests.sh --no-smoke

# Run specific test file
./scripts/run_integration_tests.sh tests/integration/test_full_workflow.py
```

### Simulation Backend Matrix Execution Contract

The simulation-facing integration suite must run as a two-backend matrix aligned with `specs/architecture/distributed-execution.md` and `specs/architecture/simulation-and-dod.md`.

- Discovery scope: include tests that call `/benchmark/simulate` or `/benchmark/verify` (directly or via helper wrappers).
- Run order: execute MuJoCo first, then Genesis, to surface compatibility regressions before Genesis-only feature checks.
- Backend selection: use `SIMULATION_DEFAULT_BACKEND` in the integration runner environment and resolve backend in tests through `tests/integration/backend_utils.py:selected_backend()`.
- Portable tests: when a test is expected to pass on both backends, the test payload must set backend from `selected_backend()` instead of hardcoding `"GENESIS"`/`"MUJOCO"`.
- Backend-specific tests: when a test asserts behavior that is intentionally unavailable on one backend, gate it explicitly for that backend.
- MuJoCo exclusion rule: softbody and dynamic-wire behavior tests (for example INT-126 wire-tear dynamics) are not required on MuJoCo and must be skipped there.

Recommended execution pattern:

```bash
# MuJoCo pass
SIMULATION_DEFAULT_BACKEND=MUJOCO ./scripts/run_integration_tests.sh <simulation-test-nodes>

# Genesis pass
SIMULATION_DEFAULT_BACKEND=GENESIS ./scripts/run_integration_tests.sh <simulation-test-nodes>
```

### Backend Skip Policy (`skipif` vs `skip`)

Use explicit backend gating in test definitions.

- Preferred: `@pytest.mark.skipif(...)` for backend exclusions that are known at collection/import time.
- Allowed: `pytest.skip(...)` inside test bodies only when the skip condition depends on runtime-only state (for example downstream service health or artifact/runtime capability discovered after setup).
- Every backend skip must include a concrete reason mentioning the unsupported contract.

Recommended pattern:

```python
import pytest
from shared.simulation.schemas import SimulatorBackendType
from tests.integration.backend_utils import selected_backend

@pytest.mark.skipif(
    selected_backend() != SimulatorBackendType.GENESIS,
    reason="INT-126 requires backend-supported dynamic wire/softbody behavior",
)
async def test_int_126_wire_tear_behavior():
    ...
```

This keeps backend intent explicit in pytest collection output and reduces ambiguity during matrix triage.

### Validation Preview Backend Contract

`/benchmark/validate` is not the simulation-backend parity path.

The architecture contract is:

1. `/benchmark/validate` performs geometric validation plus static preview generation.
2. The static preview generation path uses MuJoCo by default, even when `physics.backend=GENESIS`.
3. `/benchmark/validate` does not perform an extra Genesis load/render/build gate solely for backend parity.
4. Genesis parity remains covered by the simulation-facing backend matrix and by Genesis simulation tests where Genesis behavior is required.

Validation-preview tests should therefore assert:

1. the preview artifact contract still holds,
2. the preview backend routing is correct,
3. the selected simulation backend is not silently mutated for `/benchmark/simulate`.
4. MuJoCo-backed preview images persist as RGB/depth/segmentation sibling files in `renders/`.
5. `config/agents_config.yaml render.{rgb,depth,segmentation}` disables only the requested preview artifact types.
6. `renders/render_manifest.json` persists per-image metadata, including segmentation legend entries with semantic labels and unique instance identifiers.
7. RGB preview output reflects configured material colors for differing `material_id` values.
8. When `benchmark_definition.yaml` contains `goal_zone`, `forbid_zones`, and `build_zone`, the RGB preview output includes visible green, red, and gray objective boxes rather than dropping those visuals.

## Required integration test suite

Priorities:

- `P0`: Must pass before merge to `main`.
- `P1`: Must pass in nightly/pre-release.
- `P2`: Extended production-quality/evaluation suite.

### P0: Architecture parity baseline

| ID | Test | Required assertions |
|---|---|---|
| INT-001 | Compose boot + health contract | `controller`, `worker`, `controller-worker`, `postgres`, `minio`, `temporal` become healthy/started; health endpoints return expected status payload. |
| INT-002 | Controller-worker execution boundary | Agent-generated execution happens on worker only; controller never runs LLM-generated code. |
| INT-003 | Session filesystem isolation | Two concurrent sessions cannot read each other's files. |
| INT-004 | Heavy-worker single-flight admission | Multiple agents may run, but each heavy-worker instance accepts only one active heavy job; while that job is active, `/ready` reports not-ready, and concurrent requests to the same instance receive deterministic busy responses (no in-worker buffering/scheduling). Multi-worker throughput/fan-out behavior is out of scope for this test. |
| INT-187 | Heavy-worker crash containment boundary | Force deterministic simulation child-process failure and assert fail-closed request failure while `worker-heavy` API health stays up and subsequent heavy requests can still be served (no whole-service crash from one simulation failure). |
| INT-005 | Engineer planner mandatory artifact gate | Engineer planner must block handoff unless planner artifacts are present/valid (`plan.md`, `todo.md`, `benchmark_definition.yaml`, `assembly_definition.yaml`) and traces contain explicit `TOOL_START` for `submit_plan` with `node_type=engineer_planner`. After `submit_plan`, episode must reach `PLANNED`; if it reaches `FAILED`, test fails. Missing `submit_plan` must fail closed (no success-like status transition). |
| INT-006 | `plan.md` structure validation | Exact required engineering plan headings enforced (`1..5` sections). |
| INT-007 | `todo.md` checkbox integrity | Required checkbox format is enforced; deleted mandatory checklist entries are rejected. |
| INT-008 | `benchmark_definition.yaml` logic validation | Build/goal/forbid constraints validated: bounds checks, no illegal intersections, and valid benchmark-owned fixture metadata (`benchmark_parts` unique IDs/labels, required `material_id` or `cots_id` when metadata is declared, attachment policy uses typed `attachment_methods` with only `fastener`/`none`). |
| INT-009 | `assembly_definition.yaml` schema gate | Required fields and numeric types enforced per method; malformed/template-like files rejected. |
| INT-010 | Planner pricing script integration | `validate_costing_and_price` runs, computes totals, includes static benchmark drilling cost from `manufacturing_config.yaml` when `environment_drill_operations` is present, and blocks handoff when over caps. |
| INT-011 | Planner caps under benchmark caps | Planner-owned `max_unit_cost`/`max_weight` are <= benchmark/customer limits. |
| INT-012 | COTS search read-only behavior | COTS search path can query catalog, cannot mutate DB/files beyond allowed journal logging. |
| INT-013 | COTS output contract | Output contains required candidate fields (`part_id`, manufacturer, specs, price, source, fit rationale) or explicit no-match rationale. |
| INT-014 | COTS propagation into planning artifacts | Selected COTS part IDs/prices propagate into plan and cost-estimation artifacts. |
| INT-015 | Engineer handover immutability checks | Engineer cannot modify benchmark environment geometry (hash/checksum immutability checks across handover). |
| INT-016 | Review decision YAML schema gate | Reviewer decision YAML supports only allowed decision values, valid reviewer stage, and stage-appropriate reason codes. |
| INT-017 | Plan refusal decision loop | Refusal requires proof; reviewer confirm/reject branches route correctly. |
| INT-018 | `validate_and_price` integration gate | `simulate` and coder `submit_for_review(Compound)` are hard-blocked unless the latest `validate_and_price` for the same revision succeeded and produced valid handover artifacts (`validation_results.json`, `simulation_result.json`). `submit_for_review(Compound)` must then produce a valid latest-revision stage-specific manifest (`.manifests/engineering_execution_review_manifest.json` for engineering execution review; `.manifests/benchmark_review_manifest.json` for benchmark review). No fallback or inferred success is allowed. |
| INT-019 | Cost/weight/build-zone hard failure | Validation/simulation/review-submission are blocked when price, weight, or build-zone constraints fail, or when required handover artifacts are missing/invalid for the latest revision. Fail closed with explicit reason codes. |
| INT-020 | Simulation success/failure taxonomy | Goal-hit, forbid-hit, out-of-bounds, timeout, and instability are correctly classified in response payloads/events; reviewer handoff eligibility is granted only for the goal-hit/green-zone success path. |
| INT-021 | Runtime randomization robustness check | One admitted heavy-worker job executes one backend run with batched parallel jittered scenes (`num_scenes`) and aggregates pass/fail statistics correctly. |
| INT-022 | Motor overload + forcerange behavior | Force clamping behaves correctly; sustained overload produces `motor_overload` failure reason. |
| INT-023 | Fastener validity rules | Required fastener/joint constraints are enforced (e.g., rigid connection constraints and invalid mating rejection). |
| INT-024 | Worker benchmark validation toolchain | Benchmark `validate` catches intersecting/invalid objective setups across randomization ranges. |
| INT-188 | Validation preview backend split contract | `/benchmark/validate` generates the standard static preview package through MuJoCo by default, even when `physics.backend=GENESIS`; preview artifacts remain present and valid; MuJoCo-backed preview renders persist RGB images plus sibling `_depth.png` and `_segmentation.png` files in `renders/` by default; `renders/render_manifest.json` persists per-image metadata and segmentation legend entries with semantic labels plus unique instance identifiers; RGB preview output reflects configured material colors for differing `material_id` values; when `benchmark_definition.yaml` contains `goal_zone`, `forbid_zones`, and `build_zone`, the preview RGB images visibly include green/red/gray objective boxes; `config/agents_config.yaml render.{rgb,depth,segmentation}` can disable any one of those artifact types independently; `/benchmark/validate` does not introduce a separate Genesis load/render gate for parity. Genesis parity remains covered by simulation-backend matrix tests. |
| INT-025 | Events collection end-to-end | Worker emits `events.jsonl`, controller ingests/bulk-persists, event loss does not occur in normal path. |
| INT-026 | Mandatory event families emitted | Tool calls, simulation request/result, manufacturability checks, lint failures, plan submissions, and review decisions are emitted in real runs. |
| INT-027 | Seed/variant observability | Static variant ID + runtime seed tracked for every simulation run. |
| INT-028 | Strict API schema contract | OpenAPI is valid and runtime responses match schema for controller-worker critical endpoints. |
| INT-029 | API key enforcement | Protected endpoints reject missing/invalid key and accept valid key. |
| INT-030 | Interrupt propagation | User interrupt on controller cancels active worker job(s) and leaves consistent episode state. |
| INT-053 | Temporal workflow lifecycle logging | Starting an episode persists workflow identity and lifecycle transitions (`queued/running/completed/failed`) with timestamps; data is queryable through system persistence and events. |
| INT-054 | Temporal outage/failure logging path | If Temporal is unavailable/fails, episode must not report false success; explicit failure state/reason/event must be persisted. |
| INT-055 | S3 artifact upload logging | Successful asset uploads persist storage metadata (bucket/key/etag-or-version where available) and link to episode/asset records. |
| INT-056 | S3 upload failure + retry logging | Forced object-store failure triggers retry/failure policy; final state and failure events are consistent and queryable. |
| INT-061 | Asset serving security + session isolation contract | `GET /assets/{path}` serves only session-scoped files, returns expected MIME types for supported formats, and rejects stale/broken Python source assets with `422 Unprocessable Entity` when syntax heuristic fails. |
| INT-062 | Split-worker OpenAPI artifact contract | Generated worker API schema is either (a) merged light+heavy `worker_openapi.json` or (b) separate `worker_light_openapi.json` + `worker_heavy_openapi.json`; benchmark/simulation endpoints must not disappear from generated artifacts. |
| INT-063 | Mounted path compatibility/read-only contract | `/utils`, `/skills`, `/reviews`, `/config` mounts are present and read-only across light/heavy worker surfaces; write attempts fail while workspace root remains writable. |
| INT-070 | Mounted path traversal protection | Path traversal attempts against mounted/read-only paths are rejected deterministically (`403`) and do not allow escaping mount boundaries. |
| INT-071 | Agent filesystem policy precedence + reviewer write scope | `agents_config.yaml` enforcement matches policy precedence (`deny` > `allow`, unmatched => deny, agent override over defaults), `.manifests/**` is denied to all agent roles (read/write), and reviewer write/edit is restricted to stage-specific review decision/comments YAML pairs only. |
| INT-072 | `plan_refusal.md` validation + reviewer routing | Plan refusal requires valid `plan_refusal.md` frontmatter, role-specific reason enums, non-empty evidence body, supports multi-reason lists, and reviewer `confirm_plan_refusal` / `reject_plan_refusal` routes deterministically. |
| INT-073 | Session/episode/lineage observability linkage | Persisted records/events expose joinable linkage `user_session_id -> episode_id -> (simulation_run_id, cots_query_id, review_id)` plus `seed_id`, `seed_dataset`, `seed_match_method`, `generation_kind`, `parent_seed_id`, `is_integration_test`, and `integration_test_id` without conflating session and episode identity. |
| INT-074 | Engineering plan-reviewer DOF minimization gate | Engineering plan reviewer rejects excessive/unjustified `final_assembly.parts[*].dofs` assignments with deterministic suspicion threshold (`len(dofs) > 3` must reject unless explicit accepted mechanism-level justification exists) and re-runs reviewer-side `validate_and_price` (or equivalent wrapped validator), rejecting on mismatch/failure. |
| INT-075 | Engineering execution-reviewer over-actuation deviation gate | Engineering execution reviewer flags over-actuated plan deviations (including unjustified DOF expansion) even when a single simulation run passes, and persists reviewer evidence/events for the deviation decision. |
| INT-101 | Physics backend selection contract | Setting `physics.backend: "mujoco"` in config selects the MuJoCo backend; `"genesis"` selects Genesis. Default (`genesis`) is used when not specified. `simulation_backend_selected` event emitted. |
| INT-102 | FEM material config validation | When `fem_enabled: true`, all manufactured parts must have FEM fields (`youngs_modulus_pa`, `poissons_ratio`, `yield_stress_pa`, `ultimate_stress_pa`) in `manufacturing_config.yaml`; missing fields rejected with clear error before simulation. |
| INT-103 | Part breakage detection | Simulation with a part exceeding `ultimate_stress_pa` stops immediately with `failure_reason: PART_BREAKAGE`; result contains part label, stress value, location; `part_breakage` event emitted. |
| INT-104 | Stress reporting in simulation result | After genesis/FEM simulation, `SimulationResult.stress_summaries` populated with per-part `StressSummary` (max von Mises, safety factor, utilization %); empty list for non-FEM runs. |
| INT-105 | Fluid containment objective evaluation | Benchmark with `fluid_containment` objective passes when ≥ threshold fraction of particles remain in containment zone; fails otherwise with `FLUID_OBJECTIVE_FAILED`. |
| INT-106 | Flow rate objective evaluation | Benchmark with `flow_rate` objective passes when measured particles-per-second across gate plane is within tolerance of target; fails otherwise with `FLUID_OBJECTIVE_FAILED`. |
| INT-107 | Stress objective evaluation | Benchmark with `max_stress` objective fails simulation when max von Mises exceeds threshold with `STRESS_OBJECTIVE_EXCEEDED`; passes when below. |
| INT-108 | Tetrahedralization pipeline | STL → TetGen → `.msh` pipeline succeeds for valid geometry; emits `meshing_failure` event and retries with mesh repair for non-manifold input; hard-fails with `FAILED_ASSET_GENERATION` if unrecoverable. |
| INT-109 | Physics instability abort | If total kinetic energy exceeds threshold during simulation, simulation aborts with `failure_reason: PHYSICS_INSTABILITY`; `physics_instability` event emitted. |
| INT-110 | GPU OOM retry with particle reduction | Forced CUDA OOM triggers auto-retry at 75% particle count; `gpu_oom_retry` event emitted; result annotated `confidence: approximate`. |
| INT-111 | `validate_and_price` FEM material gate | `validate_and_price` rejects parts whose `material_id` lacks FEM fields when `fem_enabled: true` and `backend: genesis`. |
| INT-112 | Genesis rigid-body mode: backend ignores FEM/fluid config | Running with default backend (`genesis`) ignores `fluids`, `fluid_objectives`, `stress_objectives`, and `fem_enabled` if not specified; existing benchmarks pass unchanged. |
| INT-113 | Electronics planner explicit submission gate | Electronics planner must emit explicit `submit_plan` (`TOOL_START`) with `node_type=electronics_planner`; after submission, episode must reach `PLANNED` (and must not transition to `FAILED`). Missing submission fails closed. |
| INT-114 | Benchmark planner explicit submission gate | Benchmark planner must emit explicit `submit_plan` (`TOOL_START`) with `node_type=benchmark_planner`; successful submission must materialize `.manifests/benchmark_plan_review_manifest.json`, unblock `Benchmark Plan Reviewer`, and only after benchmark plan-review approval may the episode reach `PLANNED` (and must not transition to `FAILED`). Missing submission fails closed. |
| INT-184 | Node-entry validation fail-fast + reroute metadata contract | Invalid node entry in integration mode must fail closed in one turn with `FAILED`, emit `node_entry_validation_failed` evidence, and skip target-node execution. Persisted metadata must include `node`, `disposition`, `reason_code`, and structured `errors`; when a deterministic previous-node mapping exists, `reroute_target` must be populated (for non-integration reroute parity) even when integration disposition is `fail_fast`. |
| INT-120 | Circuit validation pre-gate | `validate_circuit()` must pass (no short circuits, no floating nodes, total draw ≤ PSU rating) before physics simulation proceeds; simulation rejected otherwise. |
| INT-121 | Short circuit detection | Circuit with near-zero resistance path across supply triggers `FAILED_SHORT_CIRCUIT` with branch current in result. |
| INT-122 | Overcurrent supply detection | Circuit total draw exceeding `max_current_a` triggers `FAILED_OVERCURRENT_SUPPLY`; validation reports total draw vs PSU rating. |
| INT-123 | Overcurrent wire detection | Wire carrying current exceeding gauge rating triggers `FAILED_OVERCURRENT_WIRE` with wire ID and measured current. |
| INT-124 | Open circuit / floating node detection | Unconnected circuit node triggers `FAILED_OPEN_CIRCUIT`; validation reports floating node identifier. |
| INT-125 | Motor power gating in simulation | Motor with valid controller function but no circuit power (`is_powered = 0`) produces zero effective torque; motor with power produces expected torque. |
| INT-126 | Wire tear during simulation | Wire tension exceeding rated tensile triggers `FAILED_WIRE_TORN`; affected motor stops (`is_powered` drops to 0); event emitted. |
| INT-127 | Legacy implicit-power compatibility | Episodes without `electronics` section implicitly set `is_powered = 1.0` for all motors; existing benchmarks pass unchanged. |
| INT-128 | `benchmark_definition.yaml` electronics schema gate | `electronics_requirements` section validates `power_supply_available`, `wiring_constraints`, and `circuit_validation_required` fields; malformed entries rejected. |

### P1: Full architecture workflow coverage

| ID | Test | Required assertions |
|---|---|---|
| INT-031 | Benchmark planner -> plan reviewer -> CAD -> reviewer path | Full benchmark-generation flow validates artifacts, benchmark plan-review loop, accepted handoff object integrity, benchmark-planner `submit_plan` trace presence before planner handoff, `Benchmark Plan Reviewer` start only after valid latest-revision `.manifests/benchmark_plan_review_manifest.json` exists, and post-coder reviewer start only after valid latest-revision coder handover artifact (`.manifests/benchmark_review_manifest.json`) exists. |
| INT-032 | Benchmark-to-engineer handoff package | Engineer receives expected bundle (`benchmark_definition.yaml`, benchmark-owned fixture metadata including `benchmark_parts`, environment geometry metadata, 24-view renders, moving-parts DOFs, runtime jitter metadata); downstream reviewer gate must require latest-revision handover artifacts, not tool-trace presence alone. |
| INT-033 | Engineering full loop (planner/coder/reviewer) | Planner sets realistic budgets and emits `submit_plan`; coder implements and calls python `submit_for_review(Compound)` only after passing latest-revision validation/simulation gates and manifest generation; reviewer approves/rejects with typed decision and evidence. |
| INT-034 | Reviewer evidence completeness | Review decisions include expected evidence fields, valid reviewer-specific handover manifest (for example `.manifests/benchmark_plan_review_manifest.json`, `.manifests/benchmark_review_manifest.json`, `.manifests/engineering_plan_review_manifest.json`, `.manifests/engineering_execution_review_manifest.json`, or `.manifests/electronics_review_manifest.json`) tied to the latest revision and successful simulation result when that stage requires implementation evidence, reviewer-specific persisted review file path for that stage/round, and fail-closed visual-evidence enforcement when latest-revision render images exist (`inspect_media(...)`, `media_inspection`, `llm_media_attached`). |
| INT-035 | Materials config enforcement | Only materials defined in `manufacturing_config.yaml` are accepted by validation/simulation pipeline. |
| INT-036 | Supported workbench methods | CNC, injection molding, and 3D print validation/pricing each function in integrated runs. |
| INT-037 | Joint mapping to MJCF correctness | Build123d joint definitions map to expected MJCF constraints/actuators in integrated export/simulate flow. |
| INT-038 | Controller function family coverage | Constant/sinusoidal/square/trapezoidal (and position controllers where supported) execute via runtime config without schema/tool failures. |
| INT-039 | Render artifact generation policy | On-demand render/video behavior matches policy and artifacts are discoverable by reviewer/consumer paths; persisted simulation video/image artifacts visually retain benchmark objective boxes (goal green, forbid red, build gray) when the benchmark defines them. |
| INT-040 | Asset persistence linkage | Final scripts/renders/mjcf/video are stored in S3 and linked from DB records. |
| INT-041 | Container preemption recovery path | Long task interruption records termination reason and resumes/retries through Temporal strategy. |
| INT-042 | Async callbacks/webhook completion path | Long-running simulation/manufacturing checks complete via async callback path with durable state transitions. |
| INT-043 | Batch-first execution path | Batch job submission across multiple episodes executes asynchronously with correct per-episode isolation. |
| INT-044 | Schemathesis fuzzing integration | Fuzz critical endpoints for strict API behavior; no schema drift/crashers on core paths. |
| INT-045 | Skills sync lifecycle | Worker pulls expected skills at run start; skill read events captured; skill version metadata recorded. |
| INT-057 | Backup-to-S3 logging flow | Backup endpoint writes expected snapshot/object(s) to S3 and persists backup status metadata (size, duration, key). |
| INT-058 | Cross-system correlation IDs | A single episode/trace can be correlated across controller logs/events, Temporal records, and S3 asset metadata. |
| INT-059 | Langfuse trace linkage in live runs | With valid `LANGFUSE_PUBLIC_KEY`/`LANGFUSE_SECRET_KEY`, live episode execution emits trace records linked by non-empty `langfuse_trace_id`; tool/LLM/event traces are correlated to the same run-level trace identity. |
| INT-060 | Langfuse feedback forwarding contract | `POST /episodes/{episode_id}/traces/{trace_id}/feedback` forwards score/comment to Langfuse and persists local feedback fields; missing Langfuse client returns `503`, missing `langfuse_trace_id` returns `400`. |
| INT-064 | COTS reproducibility metadata persistence | COTS queries/selection persist reproducibility metadata (`catalog_version`, `bd_warehouse_commit`, `generated_at`, `catalog_snapshot_id`) plus normalized query snapshot, ordered candidates, and final selected `part_id`s; all are exposed in downstream artifacts/events used for replayable evaluation, and handoff is invalid when selected parts exist without this metadata. |
| INT-065 | Skill safety toggle enforcement | Skill-writer flow blocks or reverts sessions that overwrite/delete more than configured threshold lines (15), records safety event, and preserves prior skill content when guard trips. |
| INT-066 | Fluid-on-electronics failure coupling | In electromechanical simulations with fluids enabled, fluid contact with powered electrical components triggers electrical failure state and benchmark failure/penalty path. |
| INT-067 | Steerability exact-pointing + mention payload contract | Face/edge/vertex/part/subassembly selections and `@`-mentions are accepted over API/UI boundary, preserved in prompt payload, and observable in run traces/events used by the agent. |
| INT-068 | Line-targeted steering contract | `@path/file.py:start-end` style references resolve and provide the exact requested code span to the agent context in the majority path; invalid ranges fail with explicit user-visible validation errors. |
| INT-069 | Frontend delivery visibility contract | End-to-end UI flow exposes simulation outputs, schematics, and macro wire views backed by real API assets for completed episodes (not placeholder/test fixtures). |
| INT-131 | Full fluid benchmark workflow (planner → engineer → reviewer) | Benchmark planner creates fluid-based benchmark; engineer designs solution with `define_fluid()` and `get_stress_report()`; reviewer verifies fluid containment metrics pass and stress results are reasonable. |
| INT-132 | Full electromechanical workflow (split planning, unified implementation, specialist review) | `Engineering Planner` and `Electronics Planner` produce a combined handoff; `Engineering Plan Reviewer` approves it; one `Engineering Coder` implements mechanics, electronics, and wire routing in one revision; `Electronics Reviewer` and `Engineering Execution Reviewer` validate the unified result; simulation runs with circuit validation and power gating. |
| INT-133 | Unified electromechanical conflict iteration loop | Wire-routing or packaging conflict (for example wire intersects moving-part sweep or connector/PSU clearance fails) routes back to the unified implementation/planning loop without inventing a separate late electrical implementer; reviewer and execution gates stay blocked until the latest revision resolves the conflict. |
| INT-134 | Stress heatmap render artifact | `preview_stress()` produces stress heatmap images stored in S3 and discoverable by reviewer. |
| INT-135 | Wire routing clearance validation | `check_wire_clearance()` rejects routes that intersect solid parts (except at attachment points) and wire bend radius violations. |
| INT-136 | Power budget validation | `calculate_power_budget()` computes total draw vs PSU capacity; warns on over-provisioning (>200%) and rejects under-provisioning. |
| INT-137 | COTS electrical component search | COTS search returns valid PSU, relay, connector, wire components from `parts.db`; required fields (`cots_part_id`, price, specs) present. |
| INT-138 | Smoke-test mode for Genesis | `smoke_test_mode: true` caps particles to 5000, labels result `confidence: approximate`, and rejects use in final validation. |
| INT-139 | Fluid data storage policy | Raw particle data stays on worker `/tmp`; only MP4 video, JSON summary metrics, and stress summaries uploaded to S3; raw cache wiped after upload. |
| INT-140 | Wire and electrical component costing | `validate_and_price()` includes wire cost (per-meter × length) and COTS electrical component costs in total. |
| INT-141 | Circuit transient simulation | `simulate_circuit_transient()` produces motor ON/OFF states over time; results match expected switching sequence from netlist. |

### P2: Multi-episode and evaluation architecture tests

| ID | Test | Required assertions |
|---|---|---|
| INT-046 | Plan-to-CAD fidelity regression | Reconstruct-from-plan cycles preserve geometry fidelity within configured tolerance. |
| INT-047 | Cross-seed transfer improvement eval | Solving one seed in a batch improves success over related seeds relative to baseline. |
| INT-048 | Reviewer optimality regression | Cases where reviewer marked "optimal" are later checked against materially cheaper alternatives. |
| INT-049 | Evaluation metric materialization | Architecture metrics are computable from persisted events/artifacts without missing fields. |
| INT-050 | Dataset readiness completeness | A completed episode has all mandatory artifacts/traces/validation markers for training readiness. |
| INT-051 | Journal quality integration checks | Journal entries are linked to observation IDs and satisfy required structure at ingestion time. |
| INT-052 | Skill effectiveness tracking | Performance delta before/after skill version updates is measurable from stored metadata. |
| INT-151 | Breakage prevention eval | Engineer solutions do not cause `PART_BREAKAGE` failure in 90% of cases across runtime-randomization batches of jittered scenes. |
| INT-152 | Safety factor range eval | Average safety factor across all parts is between 1.5 and 5.0 in 80% of solutions (no overdesign, no under-design). |
| INT-153 | End-to-end fluid benchmark eval | Planner designs fluid challenge → engineer solves → passes containment metric — 50% success rate target. |
| INT-154 | Electromechanical electrical-design success rate | Given a mechanism with motors, the combined engineering planning plus unified implementation flow produces a valid circuit in 80% on the first attempt and 95% after one retry. |
| INT-155 | Wire routing survival under jitter | Wire routing survives runtime jitter (no tears across 5 seeds) in 70% of successful solutions. |
| INT-156 | Circuit-gates-motor correctness | Circuit state correctly gates motor behaviour in 95% of simulations — motors don't spin without power. |

### Agent category: orchestration/trace contract (overlay suite)

These tests verify agent behavior contracts (turn handling, tool-loop semantics, and session isolation) while still running as true integration tests over HTTP boundaries.
This category is an overlay on priorities (`P0/P1/P2`), not a replacement.
Tag these tests with both a priority marker and `integration_agent`.

Determinism rule for this category:
- Use `tests/integration/mock_responses/` scenarios to force multi-turn/tool-call paths.
- Assertions must target HTTP responses, persisted traces/events/assets, and service logs only.
- Do not assert by importing agent internals.

| ID | Priority | Test | Required assertions |
|---|---|---|---|
| INT-181 | P1 | Tool-loop ordering + termination contract | Trace/event order is consistent (`LLM` -> `TOOL_START` -> tool result -> next `LLM` ... -> finish), and the run terminates cleanly once scripted tools are exhausted. |
| INT-182 | P1 | Concurrent agent-run isolation (files + traces + context) | Parallel runs with different `X-Session-ID` do not leak files, steering context, or traces across sessions. |
| INT-183 | P1 | Steerability queue single-consumption contract | Queued steering prompt is consumed once, affects subsequent node context in-run, and does not replay unexpectedly in later turns. |
| INT-185 | P1 | Agent-failed tool error routing contract | Trigger a deterministic agent-caused tool failure (for example invalid write/edit arguments) through live APIs; assert tool error is surfaced back to the LM as an observation and the run continues with subsequent LM turn(s) under normal LM budgets. Assert no Temporal retry fan-out for the same failed tool request. |
| INT-186 | P1 | System-failed tool retry cap + infra terminalization contract | Trigger infrastructure/transport unavailability for a live tool request; assert Temporal retries are capped at 3 attempts for the same tool request/stage, then fail closed. Assert terminal metadata reports `terminal_reason=SYSTEM_TOOL_RETRY_EXHAUSTED` and `failure_class=INFRA_DEVOPS_FAILURE`. Assert retry attempts are infra-level and do not inflate LM tool-call budget accounting. |

INT-186 exception note: a deterministic mock scenario in `tests/integration/mock_responses/` is acceptable for this case because the target is a devops/infra regression contract (system-tool transport failure and retry exhaustion), not product/business logic. This exception is scoped to INT-186 only and does not relax the no-mock rule for other integration tests.

### Frontend category: UI integration and delivery contract

These are end-to-end frontend integration tests (browser + real APIs + real artifacts). They must run against the same compose stack and must not mock controller/worker API responses.
This category is functional-only: do not add pixel-perfect or visual-style assertions.
To ensure stability and prevent Hot Module Replacement (HMR) reloads from interfering with tests, the frontend must be built and served as a static distribution on port **15173** (using `npx serve -s dist -p 15173`). All frontend service integration tests must standardize on this port.
Additionally, we use test selectors for robust and easy frontend visibility testing like `data-testid="sidebar-resizer"`.
Frontend integration tests run in strict browser-error mode: any significant unexpected `console.error` or `pageerror` event fails the test run. Known benign noise must be explicitly allowlisted (for example with test markers or configured regex allowlists), not ignored by default.
Backend integration tests run in strict backend-log mode as well:

- Unexpected backend exceptions/error-level signals during a test fail that test.
- The suite uses dedicated per-service machine-readable error JSON logs (`logs/integration_tests/current/json/{controller,worker_light,worker_heavy,temporal_worker}_errors.json`, with compatibility symlinks in `logs/integration_tests/json/`) to avoid false positives from normal info/debug output.
- Structured backend `ERROR` lines are required to include `session_id` or `episode_id`; strict teardown attributes by either field in integration mode.
- The check is controlled by `STRICT_BACKEND_ERRORS` (default `1`) and supports explicit noise control via `@pytest.mark.allow_backend_errors` or `BACKEND_ERROR_ALLOWLIST_REGEXES` (regex patterns separated by `;;`).
- Early-stop behavior is controlled by `INTEGRATION_EARLY_STOP_ON_BACKEND_ERRORS` (default `1` from the wrapper) and detects non-allowlisted backend error events from `logs/integration_tests/current/json/*_errors.json` (or compatibility symlinks at `logs/integration_tests/json/*_errors.json`); allowlisted events are filtered using global + per-test rules. Detection is surfaced by the runner, and the active test is failed by pytest fixture logic (not by runner SIGINT). Use `INTEGRATION_EARLY_STOP_ON_BACKEND_ERRORS=0` when intentionally collecting full-run logs despite known backend errors.
- `@pytest.mark.allow_backend_errors` requires explicit regex patterns.
- Accepted forms include `@pytest.mark.allow_backend_errors("fs failure")` and `@pytest.mark.allow_backend_errors(regexes=["fs failure"])`.
- The backend-log teardown gate is skipped for tests already failing in call phase to avoid masking the primary failure.
Logging-level policy for integration observability: conditions that represent true contract/logic failures must be emitted at error level so strict backend-log checks can detect them; deviations from expected logic must not be logged as warnings. In particular, fallback paths that bypass or weaken intended behavior are failure signals and must be logged at error level. Warnings should be reserved for non-failing, recoverable, or advisory conditions.

| ID | Priority | Test | Required assertions |
|---|---|---|---|
| INT-157 | P0 | Session history + workflow entry | Sidebar lists sessions from live API; selecting a session opens the correct benchmark/solution workflow with matching episode context. |
| INT-158 | P0 | Benchmark vs solution workflow parity | In both workflows, prompt submission, streamed assistant output, and artifact refresh behave consistently through real backend events. |
| INT-159 | P0 | Plan approval + comment contract | After planner output, approve/disapprove controls appear; action posts decision to API; optional user comment is persisted and visible in run history. |
| INT-160 | P1 | Reasoning traces default-hidden + expandable | Reasoning traces are hidden by default, expand on user action, and render from backend-queryable live traces (not static placeholders or phase-label stubs only). |
| INT-161 | P1 | Tool-call activity rendering | Every backend `TOOL_START` trace in a run is executable and represented in the UI activity feed with typed rows (`Edited`, `Viewed file`, `Viewed directory`); failed tool calls render failure notice. |
| INT-162 | P0 | Interrupt UX to backend propagation | During active generation, `Send` is replaced by `Stop`; stop action calls interrupt API and run transitions to interrupted/cancelled state with no further streamed tokens. |
| INT-163 | P0 | Steerability context cards + multi-select | Ctrl multi-select adds multiple context cards (CAD/code/etc.), cards are removable, and outgoing prompt payload contains structured selected elements only. |
| INT-164 | P0 | Code viewer + line-target mention contract | File tree, syntax highlighting, and line numbers render from live files; selecting lines adds `@path:start-end` mention payload and invalid ranges return explicit validation errors. |
| INT-165 | P0 | CAD topology selection modes | Part/primitive/subassembly selection modes work; face-selection mode activation is visually indicated; clicking geometry adds context cards; topology overlay is on by default and toggleable. |
| INT-166 | P1 | Simulation viewer time navigation | Time controls (play/pause/seek/rewind) load corresponding simulation frames from real assets for both rigid-body and deformable playback paths. |
| INT-167 | P0 | Controller-proxied CAD asset fetch contract | GLB/OBJ fetches for viewer use controller proxy endpoints (e.g. `/api/episodes/{id}/assets/{path}`); non-GET methods are rejected; UI does not depend on mocked local geometry fixtures. |
| INT-168 | P1 | Circuit viewer integration | Circuit data renders through frontend circuit viewer from real backend outputs; selecting a circuit component can add it to steering context. |
| INT-169 | P2 | Theme toggle persistence | Light/dark mode toggle updates UI theme and persists across reload/session restore without breaking core workflow readability. |
| INT-170 | P0 | Post-run feedback UX + API persistence | Thumbs up/down appears only after model output completion; modal supports topic selection + text; submission is persisted via feedback API. |
| INT-171 | P1 | 3-column layout + resize persistence | Session/chat/viewer columns load in default 3:3:6 layout, are user-resizable, and retain user-adjusted split on reload. |
| INT-172 | P0 | Plan-approval control placement + gating | Approve/disapprove controls are available in both expected UI locations (chat-bottom and file-explorer/top-right) when planning is complete; controls are hidden/disabled before planner output is ready. |
| INT-173 | P0 | Exact-pointing payload contract (CAD entities) | Selecting face/edge/vertex/part/subassembly (including face click in FACE mode) produces typed context payloads with stable entity IDs, center/normal metadata, and source asset reference; payload reaches backend unchanged. |
| INT-174 | P0 | CAD show/hide behavior during design and simulation | Users can hide/show selected parts both in design and simulation views; visibility toggles do not corrupt selection state or context-card creation for visible entities. |
| INT-175 | P1 | Controller-first API boundary for frontend | Browser network traffic for chat/files/assets/sessions uses controller endpoints only; frontend never calls worker host directly in normal operation. |
| INT-176 | P1 | Tool-call failure recovery path in chat | Failed tool call message is rendered with failure reason, and subsequent successful tool calls/messages continue streaming without UI deadlock. |
| INT-177 | P0 | Feedback modal edit/recall + persistence contract | After output completion, user can open feedback modal, change thumbs direction before submit, select topic(s), add text, and persisted feedback reflects final edited state. |
| INT-178 | P1 | Session restore continuity (functional) | Reloading an active episode restores workflow mode, chat transcript, and artifact panel state from live APIs without requiring manual re-selection. |
| INT-179 | P1 | Manual `@` mention contract in chat input | Typed `@` mentions for supported targets (CAD entities and code ranges) are accepted and serialized as structured steering inputs; invalid mentions return explicit user-visible validation errors. |

## Per-test Unit->Integration Implementation Map (mandatory)

This section exists to force implementation as true integration tests, not unit tests.

| ID | How to implement as integration | Reject as unit-test anti-pattern |
|---|---|---|
| INT-001 | Bring up compose stack and hit `/health` endpoints over HTTP. | Importing FastAPI app/TestClient only. |
| INT-002 | Trigger real run via API; verify worker-side execution evidence and controller non-execution. | Patching remote FS client or executor calls. |
| INT-003 | Use two real session IDs via HTTP file APIs and assert isolation. | Calling router/helper methods directly in-process. |
| INT-004 | Send parallel simulate requests over HTTP to the same heavy-worker instance; assert one request is admitted, `/ready` reports not-ready while that job is active, and concurrent request(s) receive deterministic busy responses (`503` + `WORKER_BUSY`) with no in-worker queueing/buffering. Do not assert cross-worker load distribution or cluster fan-out in this test. Ensure build scripts use `PartMetadata` class. | Mocking in-worker busy-gate behavior (instead of exercising live HTTP admission) or asserting horizontal scaling behavior from a single-instance test. |
| INT-187 | Trigger a deterministic fatal simulation-child failure through live heavy execution; assert failed response, `worker-heavy /health` remains healthy, and a subsequent heavy request still succeeds without restarting the service. | Unit-testing exception handlers around simulation helpers without live subprocess/service/process-boundary verification. |
| INT-005 | Submit with missing engineer-planner artifacts through API and assert rejection, then run engineer planner flow over controller APIs and assert episode traces include `TOOL_START` with `name=submit_plan` and `node_type=engineer_planner`; after submission, assert episode reaches `PLANNED` and not `FAILED`. Assert missing submission trace cannot transition to success-like statuses. | Calling artifact validator function directly or asserting only final `PLANNED/COMPLETED` status without planner tool-call evidence. |
| INT-006 | Submit malformed `plan.md` through real flow and assert heading gate failure. | Unit-testing markdown parser in isolation only. |
| INT-007 | Edit `todo.md` through tool APIs and assert integrity rejection on bad structure. | Directly invoking TODO validator function. |
| INT-008 | Upload invalid `benchmark_definition.yaml` via API and assert logic/bounds failure. | Constructing model objects without API path. |
| INT-009 | Submit malformed `assembly_definition.yaml` in run flow and assert blocked handoff. | Pydantic-schema-only unit checks. |
| INT-010 | Execute planner submission over HTTP and verify pricing script gate behavior. | Mocking script call result. |
| INT-011 | Provide planner caps above benchmark caps via real artifacts and assert refusal. | Comparing dicts in unit-only test. |
| INT-012 | Run COTS query through runtime interface and assert read-only behavior. | Mocking DB/search client end-to-end. |
| INT-013 | Assert required COTS response fields from live API/subagent result. | Asserting a mocked tool return fixture. |
| INT-014 | Run planning flow and verify COTS IDs/prices persisted into produced artifacts. | Checking handcrafted artifact strings only. |
| INT-015 | Attempt environment mutation in engineering flow and assert immutability rejection. | Unit-testing hash helper only. |
| INT-016 | Submit invalid review decision YAML via API and assert strict decision rejection. | Parsing YAML schema in isolation only. |
| INT-017 | Exercise refusal + reviewer confirm/reject branch with real API transitions. | State-machine branch unit test with mocks only. |
| INT-018 | Call `simulate` and coder review-submission endpoints without latest successful `validate_and_price` artifacts and assert deterministic hard block; then provide valid artifacts and assert unblock only when `submit_for_review(Compound)` emits a valid latest-revision stage-specific manifest (`.manifests/engineering_execution_review_manifest.json` or `.manifests/benchmark_review_manifest.json`, as applicable). | Directly testing function precondition checks only. |
| INT-019 | Submit overweight/overbudget/out-of-zone or missing-artifact latest revision via API and assert fail-closed reason codes at validation/simulation/review-submission boundaries. | Testing only local numeric comparison helpers. |
| INT-020 | Execute scenarios over HTTP and assert taxonomy in response + events, and assert reviewer handoff remains blocked for all non-goal-hit outcomes. Build scripts must include `PartMetadata` for all parts. | Mocking simulation result enums. |
| INT-021 | Run runtime-randomization verification via API and assert one admitted heavy job produces one backend run with batched jittered scenes plus aggregated robustness output. | Single mocked seed result assertion. |
| INT-022 | Run overload scenario in real simulation path and assert `motor_overload` behavior. | Synthetic return object with overload flag. |
| INT-023 | Submit invalid fastener/joint setup via run flow and assert validation failure. Verify `PartMetadata` is used for joint definitions. | Unit-test of fastener rule function only. |
| INT-024 | Run benchmark validation endpoint on conflicting geometry/objectives and assert failure. | Calling validation module directly in process. |
| INT-188 | Call `/benchmark/validate` through the live heavy-worker path with `physics.backend=GENESIS`; assert MuJoCo-backed preview artifacts, `renders/render_manifest.json`, objective-box overlays, modality-toggle behavior, and no extra Genesis parity gate in the validation path. | Static artifact-file snapshot checks without executing the live validation path or asserting only backend-selection helpers. |
| INT-025 | Execute real episode; verify worker events ingestion/persistence end-to-end. | Reading only local mock event list. |
| INT-026 | Verify required event families emitted from a real run, not fabricated payloads. | Event model unit tests only. |
| INT-027 | Run simulation and assert persisted static variant/runtime seed fields. | Unit assertion against seeded fixture object. |
| INT-028 | Fetch live OpenAPI from running service and validate real responses. | Static schema file lint without live calls. |
| INT-029 | Use live endpoints with missing/invalid/valid API key headers. | Unit-test auth dependency in isolation only. |
| INT-030 | Start long run and interrupt through API; assert worker cancellation and final state. | Mocking interrupt handler methods. |
| INT-031 | Execute full benchmark planner->plan-reviewer->CAD->reviewer workflow over APIs; assert planner `submit_plan` trace, benchmark plan-reviewer start blocked until valid latest-revision `.manifests/benchmark_plan_review_manifest.json` exists, and benchmark reviewer start blocked until valid latest-revision `.manifests/benchmark_review_manifest.json` exists. | Patching graph nodes in process. |
| INT-032 | Execute handoff and verify produced package artifacts from real storage paths, including latest-revision review manifest validity checks used by reviewer gate (no trace-only fallback, no model-side manifest reads). | Handcrafted dict payload assertions. |
| INT-033 | Run engineering planner/coder/reviewer loop end-to-end through services; assert planner/coder submission semantics (`submit_plan` vs python `submit_for_review(Compound)`), strict latest-revision preconditions, and fail-closed behavior on stale artifacts. | Node-level unit tests with mocked agent outputs. |
| INT-034 | Submit real reviews and assert evidence completeness plus persisted reviewer-specific manifest linkage (`.manifests/benchmark_plan_review_manifest.json`, `.manifests/benchmark_review_manifest.json`, `.manifests/engineering_plan_review_manifest.json`, `.manifests/engineering_execution_review_manifest.json`, or `.manifests/electronics_review_manifest.json`) for the accepted/rejected latest revision, reviewer-specific persisted review decision/comments YAML paths for the stage, fail-closed rejection when latest-revision renders exist but `inspect_media(...)` was not used, and `media_inspection`/`llm_media_attached` evidence for the satisfied path. | Unit-test of review schema only. |
| INT-035 | Use disallowed material in run and assert pipeline rejection from integrated config. | Local config parser unit test only. |
| INT-036 | Exercise CNC/injection/3DP through validation/pricing APIs with real artifacts. | Mocked workbench method returns. |
| INT-037 | Produce assembly and assert exported MJCF joint/actuator mapping from run outputs. | Unit-test mapper function only. |
| INT-038 | Execute controller function modes via runtime config in real simulation runs. | Direct function math unit tests only. |
| INT-039 | Trigger render/video via APIs and assert artifact policy behavior in storage. | Mocking renderer outputs. |
| INT-040 | Verify assets stored in S3 + DB links after real episode completion. | Fake storage client + call-count assertions. |
| INT-041 | Simulate container interruption in compose and validate Temporal-based recovery path. | Unit tests of retry helpers only. |
| INT-042 | Validate async callback/webhook completion by observing real state transitions. | Mock callback invocation sequence only. |
| INT-043 | Submit batch episodes through API and assert parallel/isolated outcomes. | Looping local function calls with fake inputs. |
| INT-044 | Run schemathesis against live endpoints in compose environment. | Static schema-only checks with no server. |
| INT-045 | Verify skill pull/read/versioning via live run and persisted metadata/events. | Unit tests of git/skills helper functions only. |
| INT-046 | Run multi-episode plan->CAD reconstruction cycle and compare resulting fidelity metrics. | Single-run geometry unit check. |
| INT-047 | Execute seed-batch episodes and compute transfer uplift from persisted outcomes. | Metric formula unit tests only. |
| INT-048 | Replay reviewer-optimal cases against follow-up cheaper candidates in real pipeline. | Offline CSV analysis-only tests. |
| INT-049 | Compute metrics from persisted integration artifacts/events, not synthetic fixtures. | Mocked metric input tables. |
| INT-050 | Validate dataset readiness from complete real episode artifacts and trace persistence. | Checking static checklist file only. |
| INT-051 | Ingest real journals from runs and assert link/structure constraints at persistence boundary. | Markdown parser unit tests only. |
| INT-052 | Compare pre/post skill versions using real run history and measured deltas. | Mocked before/after metric values. |
| INT-053 | Start real episode and assert workflow IDs/status transitions persisted from Temporal-integrated path. | Fake workflow objects in unit tests. |
| INT-054 | Disable Temporal service (or break connectivity) in compose and assert failure logging path. | Mocking Temporal client exceptions only. |
| INT-055 | Complete real upload and assert object metadata persisted with episode linkage. | Storage adapter unit test with fake client only. |
| INT-056 | Force real S3/MinIO upload failure and assert retry + terminal logging behavior. | Mocked upload error branch only. |
| INT-057 | Call backup endpoint against live stack and verify object creation + backup metadata logs. | Unit test of backup serializer only. |
| INT-058 | For one episode, correlate IDs across events, Temporal records, and S3 metadata from real persistence. | Asserting hardcoded correlation IDs in fixtures. |
| INT-059 | Run live episode with Langfuse configured and assert persisted trace linkage (`langfuse_trace_id`) across emitted traces. | Unit-testing callback wiring or mocking Langfuse handler calls only. |
| INT-060 | Call live feedback endpoint and assert both remote Langfuse scoring effect and local DB feedback persistence + error-path status codes. | Directly invoking feedback route function with mocked DB/Langfuse client only. |
| INT-061 | Request assets via live `GET /assets/{path}` with `X-Session-ID`; assert MIME behavior, cross-session denial, and `422` rejection for syntactically broken Python source. | Calling asset-serving helper directly or checking filesystem paths without HTTP boundary. |
| INT-062 | Generate/fetch worker OpenAPI artifact(s) in CI/integration environment and assert light+heavy endpoint coverage is present. | Linting a stale committed schema file without runtime generation. |
| INT-063 | Attempt writes to mounted paths and writes to workspace root via live file APIs; assert read-only mounts and writable workspace behavior across worker surfaces. | Asserting config constants for mount paths without exercising container mounts. |
| INT-064 | Execute COTS lookup and artifact handoff via APIs; assert persisted `catalog_version`, `bd_warehouse_commit`, `generated_at`, `catalog_snapshot_id`, normalized query snapshot, ordered candidates, and selected `part_id`s in events/records/artifacts. | Unit-testing metadata dataclass construction only. |
| INT-070 | Attempt mounted-path traversal via live file APIs (e.g., `/utils/../...`); assert deterministic `403` and no cross-boundary access. | Path-normalization unit checks without exercising worker HTTP/file boundary. |
| INT-071 | Execute per-agent file operations via live file APIs and assert `agents_config.yaml` precedence (`deny` > `allow`, unmatched deny, agent override), strict deny of `.manifests/**` to all agent roles, and reviewer-only stage-specific write scopes for the review decision/comments YAML pairs. | In-process path policy matcher and precedence helpers only. |
| INT-072 | Submit refusal artifacts through real planner/reviewer flow; assert invalid/missing `plan_refusal.md`, invalid role reasons, or empty evidence are rejected; assert `confirm_plan_refusal`/`reject_plan_refusal` transitions. | Frontmatter parser-only tests without exercising orchestration route/state transitions. |
| INT-073 | Execute real episode runs and assert persisted traces/events expose `user_session_id`, `episode_id`, `simulation_run_id`, `cots_query_id`, `review_id`, `seed_id`, `seed_dataset`, `seed_match_method`, `generation_kind`, `parent_seed_id`, `is_integration_test`, and `integration_test_id` with joinable linkage and no session/episode conflation. | Checking schema fields exist without runtime persistence assertions. |
| INT-074 | Run planner->plan-reviewer flow over HTTP with seeded over-actuated plans; assert plan reviewer rejects unjustified DOFs, enforces deterministic threshold (`len(dofs) > 3` reject unless explicitly accepted justification), and re-runs reviewer-side `validate_and_price` (or equivalent) with fail-closed rejection on mismatch/failure. | Unit tests that only compare parsed DOF lists or helper outputs without real orchestration/handover boundaries. |
| INT-075 | Run coder->execution-reviewer flow over HTTP with seeded plan-deviation over-actuation; assert execution reviewer flags deviation with persisted review evidence/event even when a single simulation run passes. | Unit tests that only compare parsed DOF lists or helper outputs without real orchestration/handover boundaries. |
| INT-065 | Run skill-update path through live workflow; attempt >15-line overwrite/delete and assert safety guard (block/revert) + persisted safety event. | Git diff unit test with mocked repository state. |
| INT-066 | Run fluid-enabled electromechanical scenario via API where fluid contacts powered electronics; assert electrical failure reason and benchmark fail/penalty output. | Manually setting electrical failure enum without simulation path. |
| INT-067 | Submit steerability request with topology selections and `@` part mentions through public API/UI contract; assert serialized payload reaches run trace/events and is consumed in agent prompt context. | Unit-testing prompt formatter with handcrafted input only. |
| INT-068 | Submit chat prompt with code-line reference (`@file:line-line`) through API; assert resolved snippet in agent context and explicit validation error for invalid spans. | Parsing line references in isolation without executing run path. |
| INT-069 | Execute UI e2e flow on completed episode and verify simulation media/schematic/wire views render from real backend assets. | Component tests with mocked API payloads only. |
| INT-101 | Set `physics.backend` in config and hit simulation endpoint; assert backend-selected event and correct engine used. | Importing backend factory and calling it directly. |
| INT-102 | Submit parts with missing FEM fields (using `PartMetadata`) via API when `fem_enabled: true`; assert rejection before simulation. | Calling Pydantic validator on material dict directly. |
| INT-103 | Run simulation via API with a part designed to break; assert `PART_BREAKAGE` in result and `part_breakage` event in event stream. Script must use `PartMetadata`. | Constructing `SimulationResult` manually with breakage flag. |
| INT-104 | Run FEM simulation via API; assert `stress_summaries` populated in HTTP response with expected fields. Requires `PartMetadata` on parts. | Calling stress computation function in-process. |
| INT-105 | Upload objectives with `fluid_containment` and run simulation via API; assert pass/fail based on particle distribution in result. | Unit-testing particle counting function only. |
| INT-106 | Upload objectives with `flow_rate` and run simulation; assert measured rate in result matches expected behavior. | Mocking particle gate crossings. |
| INT-107 | Upload objectives with `max_stress` and run simulation; assert `STRESS_OBJECTIVE_EXCEEDED` on overloaded part. Requires `PartMetadata`. | Testing stress threshold comparison in isolation. |
| INT-108 | Submit non-manifold geometry via API and assert mesh repair retry + eventual success or `FAILED_ASSET_GENERATION` with `meshing_failure` event. | Calling TetGen wrapper function directly. |
| INT-109 | Run simulation designed to produce runaway energy via API; assert `PHYSICS_INSTABILITY` result and event. | Setting kinetic energy variable directly. |
| INT-110 | (Requires GPU env) Force CUDA OOM conditions; assert retry at reduced particle count and `gpu_oom_retry` event. | Mocking CUDA allocator only. |
| INT-111 | Submit part with FEM-missing `PartMetadata` via `validate_and_price` API when genesis+FEM; assert rejection. | Calling validation function directly with dict. |
| INT-112 | Run existing rigid-body benchmark with default configuration; assert Genesis is used and success is unchanged despite fluids/FEM config being present in system but not active for the benchmark. | Importing backend and toggling flags in unit test. |
| INT-113 | Run electronics-planner flow over controller APIs and assert traces include `TOOL_START submit_plan` with `node_type=electronics_planner`; after submission, episode must reach `PLANNED` and not `FAILED`; missing submission must not reach success-like status. | Mocking planner node returns or checking only status without trace-level `submit_plan` evidence. |
| INT-114 | Run benchmark-planner flow over controller APIs and assert traces include `TOOL_START submit_plan` with `node_type=benchmark_planner`; assert `.manifests/benchmark_plan_review_manifest.json` is created and benchmark plan reviewer entry is unblocked only for the latest planner revision; after plan-review approval, episode must reach `PLANNED` and not `FAILED`; missing submission must not reach success-like status. | Mocking benchmark planner internals or asserting only terminal status without planner submission trace evidence. |
| INT-184 | Trigger deterministic node-entry rejection over HTTP (engineer + benchmark paths), assert terminal fail-fast, target-node non-execution, and persisted metadata schema (`node`, `disposition`, `reason_code`, `errors`, `reroute_target` when applicable). | Calling `evaluate_node_entry_contract()` directly in-process or asserting only log strings without episode metadata/traces. |
| INT-120 | Submit circuit via API; call `validate_circuit` endpoint; assert pass/fail controls whether simulate endpoint accepts the run. | Importing `validate_circuit()` and calling in-process. |
| INT-121 | Submit circuit with near-zero-ohm path across supply via API; assert `FAILED_SHORT_CIRCUIT` and branch current in response. | Constructing PySpice result object manually. |
| INT-122 | Submit circuit exceeding PSU `max_current_a` via API; assert `FAILED_OVERCURRENT_SUPPLY` with total draw reported. | Comparing current values in unit test. |
| INT-123 | Submit circuit with overloaded wire gauge via API; assert `FAILED_OVERCURRENT_WIRE` with wire ID. | Mocking wire current lookup. |
| INT-124 | Submit circuit with floating node via API; assert `FAILED_OPEN_CIRCUIT` and node ID in response. | Running Ngspice locally without API. |
| INT-125 | Run simulation via API with motor that has controller function but no power; assert zero torque output. Run again with power; assert expected torque. | Calling `is_powered()` helper directly. |
| INT-126 | Run simulation via API where wire tension exceeds rated tensile mid-run; assert `FAILED_WIRE_TORN` and motor stops in result. | Setting tendon tension variable directly. |
| INT-127 | Run legacy benchmark (no electronics section) via API; assert all motors produce expected torque (implicit power=1.0). | Patching `is_powered` return value. |
| INT-128 | Submit malformed `electronics_requirements` in objectives via API; assert schema rejection before simulation. | Validating Pydantic model constructor only. |
| INT-131 | Execute full fluid benchmark planner→engineer→reviewer pipeline through APIs with real tool calls. | Patching agent graph nodes in-process. |
| INT-132 | Execute the split-planning, unified-implementation electromechanical pipeline through APIs with real tool calls, circuit validation, wire routing, electronics review, and execution review. | Mocking LangGraph node transitions or splitting implementation ownership into staged in-process fake nodes. |
| INT-133 | Trigger wire-routing or electrical-packaging conflict via API and assert routing returns to the unified implementation/planning loop while reviewer/execution gates remain blocked until a new latest revision resolves the conflict. | Unit-testing handover state machine only. |
| INT-134 | Call `preview_stress` via API after simulation; assert images stored in S3 and accessible via asset endpoint. | Mocking renderer output files. |
| INT-135 | Submit wire routes via API that intersect solid parts; assert `check_wire_clearance` rejection with specific failure details. | Calling clearance check function directly in-process. |
| INT-136 | Submit circuit with known motor specs via `calculate_power_budget` API; assert correct total draw and over/under-provisioning warnings. | Unit-testing arithmetic helper only. |
| INT-137 | Query COTS catalog for electrical components (PSU, relay, wire) via API; assert valid results with required fields. | Mocking catalog search response. |
| INT-138 | Run genesis simulation with `smoke_test_mode: true` via API; assert particle cap applied and result labelled `approximate`. | Setting config flag in unit test. |
| INT-139 | Run fluid simulation via API; assert only MP4/JSON/stress uploaded to S3; assert raw particle data absent from S3. | Checking local filesystem directly. |
| INT-140 | Call `validate_and_price` on assembly with wires and electrical COTS parts; assert wire and elec costs included in total. | Calling pricing helper function directly. |
| INT-141 | Run `simulate_circuit_transient` via API; assert motor ON/OFF timeline matches expected switching sequence. | Importing transient solver directly. |
| INT-151 | Run engineer episodes via API with runtime-randomization batches; assert <10% of jittered scenes produce `PART_BREAKAGE`. | Single mock seed assertion. |
| INT-152 | Collect safety factors from multi-episode runs via API; assert average 1.5–5.0 in ≥80%. | Manual safety factor calculation. |
| INT-153 | Execute planner→engineer fluid benchmark end-to-end via APIs; assert ≥50% pass containment metric. | Offline metric analysis only. |
| INT-154 | Execute the combined engineering planning plus unified implementation flow on a motor-mechanism benchmark via APIs; assert ≥80% valid circuit first attempt. | Mocking circuit validation result. |
| INT-155 | Execute wire-routed assemblies across 5 seeds via API; assert ≥70% survive without `FAILED_WIRE_TORN`. | Single-seed unit test only. |
| INT-156 | Run electromechanical simulations via API; assert ≥95% correctly gate motor on/off based on circuit state. | Asserting `is_powered` return values only. |
| INT-157 | Open frontend against live stack; load sessions via real API and navigate into both benchmark/solution pages with correct episode IDs. | Component test with mocked session list JSON only. |
| INT-158 | Execute benchmark and solution prompt flows end-to-end in browser with live streaming/events. | UI-only snapshot tests with fake websocket payloads. |
| INT-159 | Trigger approve/disapprove controls in live run; assert decision/comment persisted via API and reflected in follow-up state. | Unit-test of plan approval button state only. |
| INT-160 | Verify reasoning panel hidden by default and populated only after expand from live trace payload; with `View reasoning` ON, content must come from persisted backend traces (`/api/episodes/{id}`), not synthetic labels only. | Rendering static reasoning markdown fixture or asserting only `Starting task phase...` strings. |
| INT-161 | Run real tool calls and assert activity feed entries (`Edited`/`Viewed`) originate from backend `TOOL_START` traces; verify UI row count aligns with backend trace count for the run. | Hardcoded activity card fixtures in component tests. |
| INT-162 | Start real run, click stop in UI, and assert interrupt endpoint + terminal interrupted state + stream halt. | Mocking interrupt action creator without backend. |
| INT-163 | Use multi-select in live CAD/code UI; assert selected context objects are posted in steering payload and removable before send. | Testing local React state reducer only. |
| INT-164 | Select lines in live code viewer and submit mention; assert backend receives resolved range and rejects invalid spans with surfaced error. | Parsing mention syntax in a unit helper test. |
| INT-165 | Use live CAD assets to switch selection modes, click geometry in FACE mode, and assert visible mode activation + context-card creation + topology browser toggle behavior over real UI. | Mocking three.js selection handlers only. |
| INT-166 | Scrub simulation timeline in browser against real simulation assets; assert frame/time sync and seek boundaries. | Video-player unit tests with local MP4 only. |
| INT-167 | Inspect live network calls during CAD load; assert controller `GET /episodes/{id}/assets/{path}` endpoints are used; verify worker is reached via proxy and disallowed methods fail. | Asserting URL builder function output only. |
| INT-168 | Render circuit view from live episode outputs and assert selectable components propagate into context payload. | Static circuit SVG snapshot tests only. |
| INT-169 | Toggle theme in live app, reload browser, assert persisted preference in real runtime behavior. | Unit test of theme store/localStorage adapter only. |
| INT-170 | Submit post-run feedback in live UI and assert API persistence + retrieval in episode trace metadata. | Modal component unit test with mocked submit handler. |
| INT-171 | Resize live 3-column layout, reload, and assert persisted split ratios and working panes. | CSS layout unit snapshot without runtime persistence. |
| INT-172 | Drive planner run to completion in live UI and assert approve/disapprove controls in both required locations; before planner completion, assert controls absent or disabled. | Asserting only conditional rendering flags in component props. |
| INT-173 | In live CAD viewer, select face/edge/vertex/part/subassembly and submit prompt; assert outbound payload includes typed entity schema (`level`, `target_id`, `center`/`normal` where available), stable IDs, and backend receives same structure unchanged. | Unit-testing selection-to-payload mapper with static fixtures only. |
| INT-174 | Use live CAD/simulation view to hide/show parts and then select remaining visible entities; assert visibility changes and intact context-card behavior end-to-end. | Toggling local visibility state without real assets or backend context submission. |
| INT-175 | Capture browser network in live run and assert all API calls target controller origin (sessions/chat/files/assets), with no direct worker-origin calls. | Checking frontend base URL constants only. |
| INT-176 | Force a real tool-call failure during run and assert failure row appears with reason; then assert later successful calls and tokens continue to render. | Rendering hardcoded failed/success event fixtures. |
| INT-177 | Submit feedback in live UI after editing score/topics/comment before final submit; assert persisted record equals final edited values, not intermediate draft. | Unit-testing modal form reducer only. |
| INT-178 | Reload browser mid-episode in live stack; assert same episode/workflow opens and chat/artifact panes repopulate from API state. | Snapshot-testing initial page layout without backend state restoration. |
| INT-179 | Type valid and invalid `@` mentions directly in live chat input; assert valid structured payload creation and explicit validation errors for invalid mention syntax/ranges. | Parsing `@` tokens in an isolated helper test only. |
| INT-181 | Execute a scripted multi-tool scenario through live APIs and assert persisted trace/event ordering and clean finish once tool list is exhausted. | Asserting mocked node transitions/tool arrays without runtime orchestration. |
| INT-182 | Start parallel live agent runs with distinct sessions and assert no cross-session reads/writes/traces/context leakage. | Unit-testing session-keyed maps/locks without HTTP/system boundaries. |
| INT-183 | Enqueue steering via live steerability endpoints during active run; assert single dequeue/consumption and downstream trace evidence in same episode. | Isolated queue unit test with mocked state transitions. |
| INT-185 | Force a deterministic LM-caused invalid tool invocation over HTTP (for example invalid path/args policy violation), assert tool error observation reaches subsequent LM turn, and verify no Temporal retry loop is created for that request. | Calling runtime retry classifiers/helpers in-process without live controller-worker boundaries. |
| INT-186 | Induce worker/API unavailability for a live tool request, assert exactly up-to-3 Temporal retries then fail-closed terminalization with `SYSTEM_TOOL_RETRY_EXHAUSTED` + `INFRA_DEVOPS_FAILURE`, and verify retries are infra-level (not LM-budget increments). | Unit tests that simulate retry counters without real Temporal/controller-worker execution traces. |

## Recommended suite organization

- `tests/integration/smoke/`: INT-001..INT-004 (fast baseline).
- `tests/integration/architecture_p0/`: INT-005..INT-030, INT-053..INT-056, INT-061..INT-063, INT-070..INT-075, INT-101..INT-114, INT-120..INT-128, INT-184, INT-187.
- `tests/integration/architecture_p1/`: INT-031..INT-045, INT-057..INT-060, INT-064..INT-069, INT-131..INT-141.
- `tests/integration/evals_p2/`: INT-046..INT-052, INT-151..INT-156.
- `tests/integration/agent/p1/`: INT-181, INT-182, INT-183, INT-185, INT-186.
- `tests/integration/frontend/p0/`: INT-157, INT-158, INT-159, INT-162, INT-163, INT-164, INT-165, INT-167, INT-170, INT-172, INT-173, INT-174, INT-177.
- `tests/integration/frontend/p1/`: INT-160, INT-161, INT-166, INT-168, INT-171, INT-175, INT-176, INT-178, INT-179.
- `tests/integration/frontend/p2/`: INT-169.

Marker recommendation:

- `@pytest.mark.integration_p0`
- `@pytest.mark.integration_p1`
- `@pytest.mark.integration_p2`
- `@pytest.mark.integration_agent`
- `@pytest.mark.integration_frontend`

CI gates recommendation:

- PR gate: run `integration_p0`.
- Optional PR fast-regression slice: run `integration_agent and integration_p0`.
- Nightly: run `integration_p0 or integration_p1`.
- Nightly agent slice: run `integration_agent and integration_p1`.
- Weekly or pre-release: run full `integration_p0 or integration_p1 or integration_p2`.

## Notes

- This spec intentionally treats architecture statements as test requirements, including expected fail paths.
- Existing unit tests for observability/workbench/COTS are useful, but they do not replace integration-level verification across controller-worker-db-storage boundaries.
- If an implementation PR adds or changes integration tests, it should include the mapped `INT-xxx` IDs in test names or docstrings.
