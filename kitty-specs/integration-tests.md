# Integration Test Specification

## Why this document exists

This document defines the integration test coverage required to match `kitty-specs/desired_architecture.md` as of February 11, 2026.

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

*Exception to the importing rules*: you can import python models and enums to use appropriate schema to avoid using pure json which will need to be manually updated later.
Commonly, these models and enums would be in `shared/` folder.

## Audit snapshot

- Integration runner last updated: `ec6f964` on **2026-02-09 18:44:47 +0000** (`scripts/run_integration_tests.sh`).
- `desired_architecture.md` changes since that commit: **25 commits**.
- Net architecture diff since runner update: **+628 / -55 lines**.
- Major post-runner additions include:
  - Engineering planner workflow and mandatory planner artifacts.
  - COTS subagent architecture and read-only catalog requirements.
  - `preliminary_cost_estimation.yaml` + `validate_costing_and_price` gate.
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
| INT-005 | Engineer planner mandatory artifact gate | Planner handoff blocked unless `plan.md`, `todo.md`, `objectives.yaml`, `preliminary_cost_estimation.yaml` are present and valid. |
| INT-006 | `plan.md` structure validation | Exact required engineering plan headings enforced (`1..5` sections). |
| INT-007 | `todo.md` checkbox integrity | Required checkbox format is enforced; deleted mandatory checklist entries are rejected. |
| INT-008 | `objectives.yaml` logic validation | Build/goal/forbid constraints validated: bounds checks, no illegal intersections, valid moving-parts definitions. |
| INT-009 | `preliminary_cost_estimation.yaml` schema gate | Required fields and numeric types enforced per method; malformed/template-like files rejected. |
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

## Per-test Unit->Integration Implementation Map (mandatory)

This section exists to force implementation as true integration tests, not unit tests.

| ID | How to implement as integration | Reject as unit-test anti-pattern |
|---|---|---|
| INT-001 | Bring up compose stack and hit `/health` endpoints over HTTP. | Importing FastAPI app/TestClient only. |
| INT-002 | Trigger real run via API; verify worker-side execution evidence and controller non-execution. | Patching remote FS client or executor calls. |
| INT-003 | Use two real session IDs via HTTP file APIs and assert isolation. | Calling router/helper methods directly in-process. |
| INT-004 | Send parallel simulate requests over HTTP; assert serialized execution from timings/logs. | Mocking simulation lock/semaphore logic. |
| INT-005 | Submit with missing artifacts through API and assert rejection. | Calling artifact validator function directly. |
| INT-006 | Submit malformed `plan.md` through real flow and assert heading gate failure. | Unit-testing markdown parser in isolation only. |
| INT-007 | Edit `todo.md` through tool APIs and assert integrity rejection on bad structure. | Directly invoking TODO validator function. |
| INT-008 | Upload invalid `objectives.yaml` via API and assert logic/bounds failure. | Constructing model objects without API path. |
| INT-009 | Submit malformed `preliminary_cost_estimation.yaml` in run flow and assert blocked handoff. | Pydantic-schema-only unit checks. |
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
| INT-020 | Execute scenarios over HTTP and assert failure taxonomy in response + events. | Mocking simulation result enums. |
| INT-021 | Run multi-seed runtime jitter simulations via API and assert aggregated robustness output. | Single mocked seed result assertion. |
| INT-022 | Run overload scenario in real simulation path and assert `motor_overload` behavior. | Synthetic return object with overload flag. |
| INT-023 | Submit invalid fastener/joint setup via run flow and assert validation failure. | Unit-test of fastener rule function only. |
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

## Coverage map: current vs required

- Covered partially today:
  - INT-001, INT-002, INT-003, INT-004 (basic smoke/plumbing/concurrency).
- Not covered or only weakly covered:
  - INT-005 through INT-060 (planner gating, COTS, artifact validation, observability completeness, Langfuse logging guarantees, Temporal/S3 logging guarantees, strict schema fuzzing, multi-episode eval architecture, etc.).

## Recommended suite organization

- `tests/integration/smoke/`: INT-001..INT-004 (fast baseline).
- `tests/integration/architecture_p0/`: INT-005..INT-030, INT-053..INT-056.
- `tests/integration/architecture_p1/`: INT-031..INT-045, INT-057..INT-060.
- `tests/integration/evals_p2/`: INT-046..INT-052.

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
