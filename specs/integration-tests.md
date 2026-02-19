# Integration Test Specification

## Why this document exists

This document defines the integration test coverage required to match `kitty-specs/desired_architecture.md`, `kitty-specs/desired_architecture_WP2_fluids.md`, `kitty-specs/desired_architecture_WP3_electronics.md`, and implemented roadmap features in `roadmap.md` as of February 19, 2026.

Short answer to the audit question: **No, current integration tests do not accurately reflect the current desired architecture.**

## **CRITICAL NOTE TO IMPLEMENTATION AGENTS**

These workflows are integration workflows. Do **not** satisfy them with unit tests disguised as integration tests.
Use mocks only when they are strictly unavoidable, and document each mock with a justification in the test docstring.

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

*Exception to the importing rules*: you can import python models and enums to use appropriate schema to avoid using pure json which will need to be manually updated later.
Commonly, these models and enums would be in `shared/` folder.

## Audit snapshot

- Integration runner last updated: Feb 17, 2026 (`scripts/run_integration_tests.sh`).
- `desired_architecture.md` changes since that commit: **25 commits**.
- Net architecture diff since runner update: **+628 / -55 lines**.
- Major post-runner additions include:
  - Engineering planner workflow and mandatory planner artifacts.
  - COTS subagent architecture and read-only catalog requirements.
  - `assembly_definition.yaml` + `validate_costing_and_price` gate.
  - Formal event taxonomy, metrics, and seed tracking.
  - Multi-tier evaluations and multi-episode integration/post-processing evals.

## Current integration coverage (what `scripts/run_integration_tests.sh` executes)

Current marker scope (`pytest -m integration`) includes 6 tests:

1. `tests/test_integration_docker.py::test_services_health`
2. `tests/test_integration_docker.py::test_controller_to_worker_agent_run`
3. `tests/integration/test_worker_concurrency.py::test_worker_concurrency`
4. `tests/integration/test_simulation_concurrency.py::test_simulation_concurrency`
5. `tests/integration/test_full_workflow.py::test_full_workflow_end_to_end`
6. `tests/integration/test_agent_real_llm.py::test_agent_run_real_llm`

This is useful smoke coverage for health, controller->worker plumbing, and concurrency, but it does not cover most architecture-critical flows.

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
| INT-004 | Simulation serialization | Multiple agents may run, but only one simulation runs at a time (queue/lock behavior enforced). |
| INT-005 | Engineer planner mandatory artifact gate | Planner handoff blocked unless `plan.md`, `todo.md`, `objectives.yaml`, `assembly_definition.yaml` are present and valid. |
| INT-006 | `plan.md` structure validation | Exact required engineering plan headings enforced (`1..5` sections). |
| INT-007 | `todo.md` checkbox integrity | Required checkbox format is enforced; deleted mandatory checklist entries are rejected. |
| INT-008 | `objectives.yaml` logic validation | Build/goal/forbid constraints validated: bounds checks, no illegal intersections, valid moving-parts definitions. |
| INT-009 | `assembly_definition.yaml` schema gate | Required fields and numeric types enforced per method; malformed/template-like files rejected. |
| INT-010 | Planner pricing script integration | `validate_costing_and_price` runs, computes totals, and blocks handoff when over caps. |
| INT-011 | Planner caps under benchmark caps | Planner-owned `max_unit_cost`/`max_weight` are <= benchmark/customer limits. |
| INT-012 | COTS search read-only behavior | COTS search path can query catalog, cannot mutate DB/files beyond allowed journal logging. |
| INT-013 | COTS output contract | Output contains required candidate fields (`part_id`, manufacturer, specs, price, source, fit rationale) or explicit no-match rationale. |
| INT-014 | COTS propagation into planning artifacts | Selected COTS part IDs/prices propagate into plan and cost-estimation artifacts. |
| INT-015 | Engineer handover immutability checks | Engineer cannot modify benchmark environment geometry (hash/checksum immutability checks across handover). |
| INT-016 | Review decision schema gate | Reviewer output frontmatter supports only allowed decision values and required evidence fields. |
| INT-017 | Plan refusal decision loop | Refusal requires proof; reviewer confirm/reject branches route correctly. |
| INT-018 | `validate_and_price` integration gate | `simulate` and `submit_for_review` require manufacturability + pricing validation first. |
| INT-019 | Cost/weight/build-zone hard failure | Submission/simulation blocked when design exceeds price, weight, or build-zone constraints. |
| INT-020 | Simulation success/failure taxonomy | Goal hit, forbid hit, out-of-bounds, timeout, and instability are correctly classified in result payloads and events. |
| INT-021 | Runtime randomization robustness check | Multi-seed runtime jitter execution occurs and aggregates pass/fail statistics correctly. |
| INT-022 | Motor overload + forcerange behavior | Force clamping behaves correctly; sustained overload produces `motor_overload` failure reason. |
| INT-023 | Fastener validity rules | Required fastener/joint constraints are enforced (e.g., rigid connection constraints and invalid mating rejection). |
| INT-024 | Worker benchmark validation toolchain | Benchmark `validate` catches intersecting/invalid objective setups across randomization ranges. |
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
| INT-120 | Circuit validation pre-gate | `validate_circuit()` must pass (no short circuits, no floating nodes, total draw ≤ PSU rating) before physics simulation proceeds; simulation rejected otherwise. |
| INT-121 | Short circuit detection | Circuit with near-zero resistance path across supply triggers `FAILED_SHORT_CIRCUIT` with branch current in result. |
| INT-122 | Overcurrent supply detection | Circuit total draw exceeding `max_current_a` triggers `FAILED_OVERCURRENT_SUPPLY`; validation reports total draw vs PSU rating. |
| INT-123 | Overcurrent wire detection | Wire carrying current exceeding gauge rating triggers `FAILED_OVERCURRENT_WIRE` with wire ID and measured current. |
| INT-124 | Open circuit / floating node detection | Unconnected circuit node triggers `FAILED_OPEN_CIRCUIT`; validation reports floating node identifier. |
| INT-125 | Motor power gating in simulation | Motor with valid controller function but no circuit power (`is_powered = 0`) produces zero effective torque; motor with power produces expected torque. |
| INT-126 | Wire tear during simulation | Wire tension exceeding rated tensile triggers `FAILED_WIRE_TORN`; affected motor stops (`is_powered` drops to 0); event emitted. |
| INT-127 | Pre-WP3 backward compat: implicit power | Episodes without `electronics` section implicitly set `is_powered = 1.0` for all motors; existing benchmarks pass unchanged. |
| INT-128 | `objectives.yaml` electronics schema gate | `electronics_requirements` section validates `power_supply_available`, `wiring_constraints`, and `circuit_validation_required` fields; malformed entries rejected. |

### P1: Full architecture workflow coverage

| ID | Test | Required assertions |
|---|---|---|
| INT-031 | Benchmark planner -> CAD -> reviewer path | Full benchmark-generation flow validates artifacts, review loop, and accepted handoff object integrity. |
| INT-032 | Benchmark-to-engineer handoff package | Engineer receives expected bundle (`objectives.yaml`, environment geometry metadata, 24-view renders, moving-parts DOFs, runtime jitter metadata). |
| INT-033 | Engineering full loop (planner/coder/reviewer) | Planner sets realistic budgets, coder implements, reviewer approves/rejects with typed decision and evidence. |
| INT-034 | Reviewer evidence completeness | Review decisions include expected evidence fields (images viewed, files checked, etc.). |
| INT-035 | Materials config enforcement | Only materials defined in `manufacturing_config.yaml` are accepted by validation/simulation pipeline. |
| INT-036 | Supported workbench methods | CNC, injection molding, and 3D print validation/pricing each function in integrated runs. |
| INT-037 | Joint mapping to MJCF correctness | Build123d joint definitions map to expected MJCF constraints/actuators in integrated export/simulate flow. |
| INT-038 | Controller function family coverage | Constant/sinusoidal/square/trapezoidal (and position controllers where supported) execute via runtime config without schema/tool failures. |
| INT-039 | Render artifact generation policy | On-demand render/video behavior matches policy and artifacts are discoverable by reviewer/consumer paths. |
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
| INT-064 | COTS reproducibility metadata persistence | COTS queries/selection persist reproducibility metadata (`catalog_version`, `bd_warehouse_commit`, `generated_at`) and expose it in downstream artifacts/events used for replayable evaluation. |
| INT-065 | Skill safety toggle enforcement | Skill-writer flow blocks or reverts sessions that overwrite/delete more than configured threshold lines (15), records safety event, and preserves prior skill content when guard trips. |
| INT-066 | Fluid-on-electronics failure coupling | In electromechanical simulations with fluids enabled, fluid contact with powered electrical components triggers electrical failure state and benchmark failure/penalty path. |
| INT-067 | Steerability exact-pointing + mention payload contract | Face/edge/vertex/part/subassembly selections and `@`-mentions are accepted over API/UI boundary, preserved in prompt payload, and observable in run traces/events used by the agent. |
| INT-068 | Line-targeted steering contract | `@path/file.py:start-end` style references resolve and provide the exact requested code span to the agent context in the majority path; invalid ranges fail with explicit user-visible validation errors. |
| INT-069 | Frontend delivery visibility contract | End-to-end UI flow exposes simulation outputs, schematics, and macro wire views backed by real API assets for completed episodes (not placeholder/test fixtures). |
| INT-131 | Full fluid benchmark workflow (planner → engineer → reviewer) | Benchmark planner creates fluid-based benchmark; engineer designs solution with `define_fluid()` and `get_stress_report()`; reviewer verifies fluid containment metrics pass and stress results are reasonable. |
| INT-132 | Full electromechanical workflow (mech → elec → reviewer) | Mech engineer designs assembly with motors; electronics engineer wires circuit, routes wires, passes `validate_circuit()`; reviewer approves; unified simulation runs with power gating. |
| INT-133 | Elec → Mech conflict iteration loop | Wire routing conflict (wire intersects moving part sweep) triggers Elec→Mech handover; Mech modifies assembly; iteration count tracked via `elec_agent_handover` event. |
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
| INT-151 | Breakage prevention eval | Engineer solutions do not cause `PART_BREAKAGE` failure in 90% of cases across multi-seed runs. |
| INT-152 | Safety factor range eval | Average safety factor across all parts is between 1.5 and 5.0 in 80% of solutions (no overdesign, no under-design). |
| INT-153 | End-to-end fluid benchmark eval | Planner designs fluid challenge → engineer solves → passes containment metric — 50% success rate target. |
| INT-154 | Elec agent circuit success rate | Given a mechanism with motors, the Elec Engineer produces a valid circuit in 80% (first attempt), 95% after one retry. |
| INT-155 | Wire routing survival under jitter | Wire routing survives runtime jitter (no tears across 5 seeds) in 70% of successful solutions. |
| INT-156 | Circuit-gates-motor correctness | Circuit state correctly gates motor behaviour in 95% of simulations — motors don't spin without power. |

## Per-test Unit->Integration Implementation Map (mandatory)

This section exists to force implementation as true integration tests, not unit tests.

| ID | How to implement as integration | Reject as unit-test anti-pattern |
|---|---|---|
| INT-001 | Bring up compose stack and hit `/health` endpoints over HTTP. | Importing FastAPI app/TestClient only. |
| INT-002 | Trigger real run via API; verify worker-side execution evidence and controller non-execution. | Patching remote FS client or executor calls. |
| INT-003 | Use two real session IDs via HTTP file APIs and assert isolation. | Calling router/helper methods directly in-process. |
| INT-004 | Send parallel simulate requests over HTTP; assert serialized execution from timings/logs. Ensure build scripts use `PartMetadata` class. | Mocking simulation lock/semaphore logic. |
| INT-005 | Submit with missing artifacts through API and assert rejection. | Calling artifact validator function directly. |
| INT-006 | Submit malformed `plan.md` through real flow and assert heading gate failure. | Unit-testing markdown parser in isolation only. |
| INT-007 | Edit `todo.md` through tool APIs and assert integrity rejection on bad structure. | Directly invoking TODO validator function. |
| INT-008 | Upload invalid `objectives.yaml` via API and assert logic/bounds failure. | Constructing model objects without API path. |
| INT-009 | Submit malformed `assembly_definition.yaml` in run flow and assert blocked handoff. | Pydantic-schema-only unit checks. |
| INT-010 | Execute planner submission over HTTP and verify pricing script gate behavior. | Mocking script call result. |
| INT-011 | Provide planner caps above benchmark caps via real artifacts and assert refusal. | Comparing dicts in unit-only test. |
| INT-012 | Run COTS query through runtime interface and assert read-only behavior. | Mocking DB/search client end-to-end. |
| INT-013 | Assert required COTS response fields from live API/subagent result. | Asserting a mocked tool return fixture. |
| INT-014 | Run planning flow and verify COTS IDs/prices persisted into produced artifacts. | Checking handcrafted artifact strings only. |
| INT-015 | Attempt environment mutation in engineering flow and assert immutability rejection. | Unit-testing hash helper only. |
| INT-016 | Submit invalid review frontmatter via API and assert strict decision rejection. | Parsing markdown frontmatter in isolation only. |
| INT-017 | Exercise refusal + reviewer confirm/reject branch with real API transitions. | State-machine branch unit test with mocks only. |
| INT-018 | Call simulate/submit endpoints without prior valid pricing and assert hard block. | Directly testing function precondition checks only. |
| INT-019 | Submit overweight/overbudget/out-of-zone design through API and assert fail reasons. | Testing only local numeric comparison helpers. |
| INT-020 | Execute scenarios over HTTP and assert failure taxonomy in response + events. Build scripts must include `PartMetadata` for all parts. | Mocking simulation result enums. |
| INT-021 | Run multi-seed runtime jitter simulations via API and assert aggregated robustness output. | Single mocked seed result assertion. |
| INT-022 | Run overload scenario in real simulation path and assert `motor_overload` behavior. | Synthetic return object with overload flag. |
| INT-023 | Submit invalid fastener/joint setup via run flow and assert validation failure. Verify `PartMetadata` is used for joint definitions. | Unit-test of fastener rule function only. |
| INT-024 | Run benchmark validation endpoint on conflicting geometry/objectives and assert failure. | Calling validation module directly in process. |
| INT-025 | Execute real episode; verify worker events ingestion/persistence end-to-end. | Reading only local mock event list. |
| INT-026 | Verify required event families emitted from a real run, not fabricated payloads. | Event model unit tests only. |
| INT-027 | Run simulation and assert persisted static variant/runtime seed fields. | Unit assertion against seeded fixture object. |
| INT-028 | Fetch live OpenAPI from running service and validate real responses. | Static schema file lint without live calls. |
| INT-029 | Use live endpoints with missing/invalid/valid API key headers. | Unit-test auth dependency in isolation only. |
| INT-030 | Start long run and interrupt through API; assert worker cancellation and final state. | Mocking interrupt handler methods. |
| INT-031 | Execute full benchmark planner->CAD->reviewer workflow over APIs. | Patching graph nodes in process. |
| INT-032 | Execute handoff and verify produced package artifacts from real storage paths. | Handcrafted dict payload assertions. |
| INT-033 | Run engineering planner/coder/reviewer loop end-to-end through services. | Node-level unit tests with mocked agent outputs. |
| INT-034 | Submit real reviews and assert evidence completeness persisted in decision records. | Unit-test of review schema only. |
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
| INT-064 | Execute COTS lookup and artifact handoff via APIs; assert persisted `catalog_version`, `bd_warehouse_commit`, and `generated_at` in events/records/artifacts. | Unit-testing metadata dataclass construction only. |
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
| INT-120 | Submit circuit via API; call `validate_circuit` endpoint; assert pass/fail controls whether simulate endpoint accepts the run. | Importing `validate_circuit()` and calling in-process. |
| INT-121 | Submit circuit with near-zero-ohm path across supply via API; assert `FAILED_SHORT_CIRCUIT` and branch current in response. | Constructing PySpice result object manually. |
| INT-122 | Submit circuit exceeding PSU `max_current_a` via API; assert `FAILED_OVERCURRENT_SUPPLY` with total draw reported. | Comparing current values in unit test. |
| INT-123 | Submit circuit with overloaded wire gauge via API; assert `FAILED_OVERCURRENT_WIRE` with wire ID. | Mocking wire current lookup. |
| INT-124 | Submit circuit with floating node via API; assert `FAILED_OPEN_CIRCUIT` and node ID in response. | Running Ngspice locally without API. |
| INT-125 | Run simulation via API with motor that has controller function but no power; assert zero torque output. Run again with power; assert expected torque. | Calling `is_powered()` helper directly. |
| INT-126 | Run simulation via API where wire tension exceeds rated tensile mid-run; assert `FAILED_WIRE_TORN` and motor stops in result. | Setting tendon tension variable directly. |
| INT-127 | Run pre-WP3 benchmark (no electronics section) via API; assert all motors produce expected torque (implicit power=1.0). | Patching `is_powered` return value. |
| INT-128 | Submit malformed `electronics_requirements` in objectives via API; assert schema rejection before simulation. | Validating Pydantic model constructor only. |
| INT-131 | Execute full fluid benchmark planner→engineer→reviewer pipeline through APIs with real tool calls. | Patching agent graph nodes in-process. |
| INT-132 | Execute full mech→elec→reviewer pipeline through APIs with real tool calls including circuit validation and wire routing. | Mocking LangGraph node transitions. |
| INT-133 | Trigger wire routing conflict via API and observe Elec→Mech iteration with `elec_agent_handover` events in stream. | Unit-testing handover state machine only. |
| INT-134 | Call `preview_stress` via API after simulation; assert images stored in S3 and accessible via asset endpoint. | Mocking renderer output files. |
| INT-135 | Submit wire routes via API that intersect solid parts; assert `check_wire_clearance` rejection with specific failure details. | Calling clearance check function directly in-process. |
| INT-136 | Submit circuit with known motor specs via `calculate_power_budget` API; assert correct total draw and over/under-provisioning warnings. | Unit-testing arithmetic helper only. |
| INT-137 | Query COTS catalog for electrical components (PSU, relay, wire) via API; assert valid results with required fields. | Mocking catalog search response. |
| INT-138 | Run genesis simulation with `smoke_test_mode: true` via API; assert particle cap applied and result labelled `approximate`. | Setting config flag in unit test. |
| INT-139 | Run fluid simulation via API; assert only MP4/JSON/stress uploaded to S3; assert raw particle data absent from S3. | Checking local filesystem directly. |
| INT-140 | Call `validate_and_price` on assembly with wires and electrical COTS parts; assert wire and elec costs included in total. | Calling pricing helper function directly. |
| INT-141 | Run `simulate_circuit_transient` via API; assert motor ON/OFF timeline matches expected switching sequence. | Importing transient solver directly. |
| INT-151 | Run multi-seed engineer episodes via API; assert <10% produce `PART_BREAKAGE`. | Single mock seed assertion. |
| INT-152 | Collect safety factors from multi-episode runs via API; assert average 1.5–5.0 in ≥80%. | Manual safety factor calculation. |
| INT-153 | Execute planner→engineer fluid benchmark end-to-end via APIs; assert ≥50% pass containment metric. | Offline metric analysis only. |
| INT-154 | Execute Elec Engineer on motor-mechanism benchmark via APIs; assert ≥80% valid circuit first attempt. | Mocking circuit validation result. |
| INT-155 | Execute wire-routed assemblies across 5 seeds via API; assert ≥70% survive without `FAILED_WIRE_TORN`. | Single-seed unit test only. |
| INT-156 | Run electromechanical simulations via API; assert ≥95% correctly gate motor on/off based on circuit state. | Asserting `is_powered` return values only. |

## Coverage map: current vs required

- Covered partially feb 14:
  - INT-001, INT-002, INT-003, INT-004 (basic smoke/plumbing/concurrency).
- Not covered or only weakly covered:
  - INT-005 through INT-060 (planner gating, COTS, artifact validation, observability completeness, Langfuse logging guarantees, Temporal/S3 logging guarantees, strict schema fuzzing, multi-episode eval architecture, etc.).
  - INT-061 through INT-069 (asset-serving security/session boundaries, split-worker OpenAPI artifacts, mounted path contracts, COTS reproducibility metadata, skill safety toggle, fluid-electronics coupling, steerability contracts, and UI delivery visibility).
  - INT-101 through INT-112 (WP2: physics backend abstraction, FEM/deformable materials, fluid simulation, stress reporting, breakage detection, meshing pipeline, fluid/stress metrics, GPU OOM handling).
  - INT-120 through INT-128 (WP3: circuit validation, electrical failure modes, motor power gating, wire tear, backward compat, electronics schema).
  - INT-131 through INT-141 (WP2/WP3 P1: full fluid and electromechanical workflows, agent handovers, stress rendering, wire routing, power budget, COTS electrical, smoke-test mode, data storage policy, circuit transient).
  - INT-151 through INT-156 (WP2/WP3 P2: breakage prevention evals, safety factor range, fluid benchmark evals, circuit success rate, wire survival, motor gating correctness).

## Recommended suite organization

- `tests/integration/smoke/`: INT-001..INT-004 (fast baseline).
- `tests/integration/architecture_p0/`: INT-005..INT-030, INT-053..INT-056, INT-061..INT-063, INT-101..INT-112, INT-120..INT-128.
- `tests/integration/architecture_p1/`: INT-031..INT-045, INT-057..INT-060, INT-064..INT-069, INT-131..INT-141.
- `tests/integration/evals_p2/`: INT-046..INT-052, INT-151..INT-156.

Marker recommendation:

- `@pytest.mark.integration_p0`
- `@pytest.mark.integration_p1`
- `@pytest.mark.integration_p2`

CI gates recommendation:

- PR gate: run `integration_p0`.
- Nightly: run `integration_p0 or integration_p1`.
- Weekly or pre-release: run full `integration_p0 or integration_p1 or integration_p2`.

## Notes

- This spec intentionally treats architecture statements as test requirements, including expected fail paths.
- Existing unit tests for observability/workbench/COTS are useful, but they do not replace integration-level verification across controller-worker-db-storage boundaries.
- If an implementation PR adds or changes integration tests, it should include the mapped `INT-xxx` IDs in test names or docstrings.
