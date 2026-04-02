# Simulation Optimization Attempts

## Scope summary

- Primary focus: persistent log of simulation-runtime optimization attempts, measurements, and decisions.
- This document stores dated architecture snapshots, experiment runs, benchmark commands, findings, and follow-up decisions.
- Use this file when evaluating whether a simulation-runtime optimization was already tried, what the measured result was, and what behavior changes were considered acceptable or risky.

## Purpose

We keep this document as the durable memory for simulation performance work.

The simulation stack is one of the most expensive and failure-prone parts of the system. Performance changes here can easily alter crash-containment, state-isolation, readiness behavior, or observability semantics. We therefore record:

1. The architecture state that was in effect when an attempt was made.
2. The exact benchmark or experiment command that was executed.
3. The measured result.
4. The interpretation and caution notes.
5. The accepted next step or explicit non-decision.

This document is append-oriented. New optimization work should add a new dated entry instead of overwriting old reasoning.

## Logging rules

The following rules apply to every future entry in this file:

1. Every architecture snapshot entry must include a date and timezone.
2. Every benchmark attempt entry must include:
   - the exact command,
   - the backend mode,
   - the environment caveats,
   - the measured timings,
   - the interpretation,
   - the proposed or rejected next step.
3. If a benchmark was run in a constrained environment such as a sandbox, the limitation must be written explicitly.
4. If a proposed optimization changes runtime semantics, the behavior delta must be written explicitly.
5. If an optimization is rejected, the reason must be persisted here instead of being kept only in chat history.

## Current architecture snapshot log

### 2026-03-09 morning GMT - runtime path in effect

This snapshot records the heavy-worker simulation architecture that was live when the optimization discussion and benchmark happened on 2026-03-09.

#### Topology

- `controller` handles orchestration and user-facing API work.
- `worker-light` handles filesystem and light execution work.
- `worker-heavy` handles simulation, validation, preview, rendering, and related heavy operations.
- `controller-temporal-worker` / Temporal handles durable workflow and activity execution.

This matches the split-worker contract in `specs/architecture/distributed-execution.md`.

#### External behavior

- `worker-heavy` is single-flight.
- The worker accepts one active heavy job at a time.
- While a heavy job is running, `/ready` returns busy and the load balancer should stop routing new heavy jobs to that worker.
- The direct `/benchmark/*` endpoints are an adapter/debug surface, not a separate production queuing model.

#### HTTP adapter path for simulation

The direct path is:

01. Client or test sends `POST /benchmark/simulate`.
02. `worker-heavy` enters `heavy_operation_admission`.
03. The request bundle and session root are resolved.
04. `run_simulation_task(...)` calls `run_simulation_in_isolated_process(...)`.
05. The runtime creates a fresh `ProcessPoolExecutor(max_workers=1, max_tasks_per_child=1, initializer=init_genesis_worker)`.
06. A fresh child process starts.
07. The child runs `init_genesis_worker()`.
08. The child imports Genesis, resolves CPU or GPU, calls `gs.init(...)`, builds a tiny scene, and then runs the simulation subprocess function.
09. The child returns the simulation result.
10. The parent process collects artifacts and returns the HTTP response.
11. The executor scope ends and the child process goes away.

#### Temporal activity path for simulation

The production-oriented path is:

1. Controller dispatches a heavy simulation through Temporal.
2. Temporal activity `worker_run_simulation` extracts the session bundle into a temp root.
3. The activity calls `run_simulation_in_isolated_process(...)`.
4. The same fresh-child-process behavior happens as in the direct HTTP path.

The key point is that both paths pay the same fresh-child startup behavior because they share the same simulation runner.

#### Validation path

`/benchmark/validate` uses the same pattern:

1. Heavy admission begins.
2. Session root is resolved.
3. `run_validation_in_isolated_process(...)` is called.
4. The runtime creates a fresh child executor with the same Genesis initializer.
5. The child runs validation and exits when the request completes.

#### Genesis initialization behavior

There are two relevant initialization sites in the current runtime:

1. `worker_heavy/runtime/simulation_runner.py::init_genesis_worker()`

   - Sets headless defaults.
   - Imports Genesis and Torch.
   - Calls `gs.init(...)`.
   - Builds a tiny plane scene as a best-effort prewarm.

2. `worker_heavy/simulation/genesis_backend.py::_ensure_initialized()`

   - Checks `gs._initialized`.
   - Calls `gs.init(...)` only if Genesis is not already initialized in that process.

This means Genesis is treated as process-global inside a given process, but the current architecture creates a new process for each simulation or validation request, so the warm state is discarded after each request.

#### Cleanup behavior in effect on 2026-03-09

Integration teardown calls `/internal/simulation/cleanup` after each integration test.

At the time of this snapshot, that cleanup endpoint:

- closes cached session backends in the parent `worker-heavy` process,
- runs `gc.collect()`,
- does not keep a persistent simulation child alive,
- does not materially control the fresh child used by `/benchmark/simulate` and `/benchmark/validate` because that child already exits at the end of each request.

This is an important caution note for future refactors: if the child becomes persistent, cleanup semantics must be redefined explicitly.

#### Performance characteristic observed before optimization

The architecture provides strong crash containment by using a fresh child process per heavy request.

The tradeoff is that it repeatedly pays:

- Python process startup,
- Genesis import/runtime startup,
- first-use backend setup,
- tiny prewarm scene cost,
- and any first-build compilation cost.

The optimization discussion on 2026-03-09 was triggered by the observation that this repeated cold-start work dominates integration runtime more than the literal `gs.init()` call itself.

## Optimization candidate log

### Candidate recorded on 2026-03-09 GMT - persistent dedicated simulation child

The currently favored candidate is a narrow runtime refactor:

- keep `worker-heavy` single-flight,
- keep a process boundary between the FastAPI parent and the physics runtime,
- replace one-fresh-child-per-request with one-long-lived dedicated simulation child process per worker instance,
- keep work single-threaded inside that child,
- clear session-local backend state between requests,
- recycle the child on crash or on explicit bounded-reuse policy.

The intended benefit is to preserve crash containment while eliminating repeated cold-start costs across requests.

The intended non-goals are:

- no change to public `/benchmark/*` request/response contracts,
- no change to load-balancer `/ready` semantics,
- no introduction of multi-job in-process scheduling,
- no move of Genesis into the FastAPI parent process,
- no thread-sharing of one Genesis runtime across arbitrary request threads.

The main caution points are:

1. stale backend state leaking between requests,
2. memory growth across many requests,
3. cleanup semantics changing,
4. recovery policy after a child crash,
5. the need for stronger executor lifecycle observability.

## Optimization attempt log

### 2026-03-09 14:12:29 GMT - Genesis init and reuse benchmark

#### Goal

The goal of this run was to answer two questions before changing runtime semantics:

1. What is the actual cost of paying `gs.init()` on every request?
2. Can Genesis realistically be initialized once and reused in-process, especially with thread boundaries in mind?

#### Script

The benchmark script was created at:

- `scripts/throwaway/genesis_init_benchmark.py`

The script measures:

- fresh spawned process with `gs.init()` only,
- fresh spawned process with `gs.init()` plus a tiny scene build,
- same-process reuse after one init,
- thread-affinity behavior when initialization and scene build happen on different threads.

#### Command

The benchmark was executed with:

```bash
GENESIS_FORCE_CPU=1 .venv/bin/python scripts/throwaway/genesis_init_benchmark.py --rounds 2 --backend auto
```

#### Environment caveats

- The run was CPU-mode only.
- `GENESIS_FORCE_CPU=1` was set to match the current integration-runner behavior.
- Cache writes were redirected into `/tmp` inside the benchmark harness because the sandboxed environment would otherwise block default cache writes under `~/.cache`.
- GPU timing was not measured in this run.

#### Measurements

Fresh spawned process, `gs.init()` only:

- `gs.init()` inside the child: mean `0.1804s`
- child total work: mean `3.3668s`
- wall process roundtrip: mean `4.1724s`

Fresh spawned process, `gs.init()` plus tiny scene:

- `gs.init()` inside the child: mean `0.1749s`
- child scene create: mean `2.2560s`
- child scene build: mean `2.0373s`
- child scene step: mean `0.1018s`
- child total work: mean `7.7303s`
- wall process roundtrip: mean `8.7341s`

Same-process reuse:

- first `gs.init()`: `0.1817s`
- second `gs.init()`: failed with `GenesisException: Genesis already initialized.`
- tiny scene build reuse timings:
  - first recorded build: `2.1257s`
  - second recorded build: `0.6400s`

Thread-affinity probe:

- `main_thread_init_s`: `0.1956s`
- fresh process where worker thread did both init and build:
  - `thread_init_s`: `0.1840s`
  - `scene_create_s`: `2.2410s`
  - `scene_build_s`: `1.9636s`
  - result: success
- process where init happened first and the worker-thread scene build happened later:
  - result: failure
  - failure family: Quadrants / LLVM thread assertion
  - representative message: `std::this_thread::get_id() == main_thread_id_`

#### Interpretation

The benchmark does not support the claim that literal `gs.init()` is the dominant cost.

The measurements support a different conclusion:

1. `gs.init()` itself is relatively small in CPU mode in this environment, roughly `0.18s`.
2. The cold-request cost is dominated by the broader fresh-process and first-scene path.
3. Reusing the same process materially reduces repeated scene-build cost after the first initialization.
4. Genesis behaves as process-global enough that a second `gs.init()` in the same process is not the intended pattern.
5. Cross-thread reuse is unsafe enough that a design based on arbitrary thread handoff should be rejected.

The practical conclusion is:

- do not optimize around "calling `gs.init()` less often" in isolation,
- optimize around "creating a fresh child process less often",
- keep Genesis work pinned to one dedicated process/thread ownership model.

#### Decision state after the run

No production code was changed as part of this benchmark.

The benchmark increased confidence in the persistent-dedicated-child design and decreased confidence in any thread-based sharing design.

The tentative next implementation step remains:

1. keep the API parent process isolated from simulation,
2. keep `worker-heavy` single-flight,
3. make the simulation child persistent across requests,
4. redefine cleanup to clear session state without tearing down the whole warm process,
5. add lifecycle observability for executor start, reuse, cleanup, recycle, and crash.

#### Caution notes preserved from discussion

- The external behavior must remain stable even if the child becomes persistent.
- `/ready` behavior must not change.
- A child crash must still fail closed.
- Cleanup must become explicit child-side cleanup, not implicit process exit.
- Validation should be included in the same design if it shares the same expensive runtime path.
- `/benchmark/verify` should not be widened into the first refactor unless necessary; it has a distinct execution path and should stay out of scope for the first change.

### 2026-03-09 15:04:19 GMT - same-scene vs different-scene warm-process benchmark

#### Goal

The goal of this run was to test the user challenge against the earlier benchmark:

1. Was the observed warm-process speedup only an artifact of repeating the exact same scene?
2. Does switching to a different object in the same warm process still force the full original cold cost?

This run was explicitly designed to separate:

- process-global warmup cost,
- same-scene repeat cost in one warm process,
- different-scene-next cost in one warm process.

#### Script

The experiment assets were persisted in:

- `scripts/experiments/genesis-scene-reuse/README.md`
- `scripts/experiments/genesis-scene-reuse/benchmark_same_vs_different_scenes.py`
- `scripts/experiments/genesis-scene-reuse/genesis_init_benchmark_baseline.py`
- `scripts/experiments/genesis-scene-reuse/results-20260309T150419Z.json`

The experiment keeps one process alive after a single `gs.init(...)` and then runs:

1. primitive box scene,
2. same primitive box scene again,
3. primitive sphere scene,
4. mesh box scene,
5. same mesh box scene again,
6. mesh sphere scene.

#### Command

The benchmark was executed with:

```bash
GENESIS_FORCE_CPU=1 .venv/bin/python scripts/experiments/genesis-scene-reuse/benchmark_same_vs_different_scenes.py --backend cpu
```

#### Environment caveats

- The run was CPU-mode only.
- `GENESIS_FORCE_CPU=1` was set.
- The experiment was intentionally not the full heavy-worker path. It isolates Genesis scene construction and step cost inside one process after one init.
- The experiment uses minimal primitive and OBJ mesh scenes. It does not prove final timings for large CAD/FEM benchmark scenes.

#### Measurements

Process init:

- `init_s`: `1.2405s`

Primitive sequence in one warm process:

- `primitive_box_1`
  - `scene_create_s`: `2.4544s`
  - `scene_build_s`: `60.1440s`
- `primitive_box_2_same`
  - `scene_create_s`: `0.0308s`
  - `scene_build_s`: `0.8258s`
- `primitive_sphere_1_diff`
  - `scene_create_s`: `0.0396s`
  - `scene_build_s`: `11.6634s`

Primitive analysis:

- same-vs-first build ratio: `0.0137`
- diff-vs-first build ratio: `0.1939`
- diff-vs-same build ratio: `14.1232`

Mesh sequence in one warm process:

- `mesh_box_1`
  - `scene_create_s`: `0.0326s`
  - `scene_build_s`: `0.8740s`
- `mesh_box_2_same`
  - `scene_create_s`: `0.0385s`
  - `scene_build_s`: `0.8773s`
- `mesh_sphere_1_diff`
  - `scene_create_s`: `0.0373s`
  - `scene_build_s`: `0.8648s`

Mesh analysis:

- same-vs-first build ratio: `1.0038`
- diff-vs-first build ratio: `0.9894`
- diff-vs-same build ratio: `0.9857`

#### Interpretation

This run materially changes the confidence level of the earlier discussion.

The earlier warm-process benchmark was not sufficient to prove what portion of the speedup came from same-scene repetition versus process-global warm state. This follow-up benchmark corrected that gap.

The results support the following conclusions:

1. Object or scene change does matter. The primitive sphere case did not drop to the same cost as the repeated primitive box case.
2. Object or scene change does not automatically reset the process to the original fully cold cost. The primitive sphere build at `11.6634s` remained far below the first primitive box build at `60.1440s`.
3. There is a large reusable process-global warmup component. The first primitive build paid a very large one-time cost that the later primitive scenes did not repeat.
4. There is also a scene-specific rebuild component. The primitive sphere remained much more expensive than repeating the same primitive box.
5. For the minimal OBJ mesh scenes used here, same-versus-different object cost was nearly identical once the process was warm. This suggests that the mesh case in this minimal benchmark is not dominated by object-family switching.

The practical correction to the architecture discussion is:

- a persistent dedicated child process would remove only the reusable process-global cold component,
- it would not remove scene-specific rebuild cost,
- any optimization that aims to reduce the remaining scene-specific cost needs a separate compiled-scene, mesh, or geometry-hash cache strategy.

#### Decision state after the run

The runtime optimization candidate remains valid, but its scope is now better defined.

What a persistent child is expected to do:

- remove repeated process-global cold-start work,
- preserve Genesis runtime warm state across requests,
- reduce repeated first-build cost that is process-global.

What a persistent child is not expected to do:

- eliminate object-specific rebuild cost for materially different scenes,
- replace the need for scene or geometry caching if those costs dominate production workloads.

#### Follow-up

The next benchmark tier, if needed, should be closer to the real heavy-worker path:

1. run two different `simulate_subprocess(...)` workloads in the same warm process,
2. compare same-input-repeat versus different-input-next,
3. decide whether compiled-scene caching is worth pursuing after the persistent-child refactor.

### 2026-03-09 16:04:36 GMT - MuJoCo vs Genesis 24-view validation-prerender benchmark

#### Goal

The goal of this run was to test the concrete product question behind `/benchmark/validate`:

1. Is the validate-time 24-view static render path materially faster in MuJoCo than in Genesis?
2. Is the speedup large enough to justify rendering validation previews in MuJoCo while leaving Genesis as the simulation backend?
3. Does the measured result support the stronger hypothesis that validation would become "near-instant" or approximately `100x` faster?

This run was intended to benchmark the exact static-render behavior used by validation, not a separate toy renderer.

#### Script or code path

The experiment assets were persisted in:

- `scripts/experiments/mujoco-vs-genesis-prerender/README.md`
- `scripts/experiments/mujoco-vs-genesis-prerender/benchmark_validate_prerender_backends.py`
- `scripts/experiments/mujoco-vs-genesis-prerender/results-20260309T160007Z.json`
- `scripts/experiments/mujoco-vs-genesis-prerender/results-20260309T160436Z.json`

The benchmark exercises two paths:

1. endpoint-equivalent validation path
   - `validate_subprocess(...)`
   - which reaches `validate(...)`
   - which reaches `prerender_24_views(...)`
2. direct static render path
   - `prerender_24_views(...)`

Each child run preserves the parent-request script snapshot and benchmark-definition snapshot, selects either `MUJOCO` or `GENESIS` in `physics.backend`, and executes in a fresh subprocess to preserve cold-request behavior.

#### Command

The final full-run commands that produced the clean comparable artifacts were:

```bash
UV_CACHE_DIR=/tmp/uv-cache uv run python scripts/experiments/mujoco-vs-genesis-prerender/benchmark_validate_prerender_backends.py --scenario all --mode validate --backend all --repetitions 1 --smoke-test-mode false --isolate-cache --force-cpu --mujoco-gl egl --timeout-sec 180
```

```bash
UV_CACHE_DIR=/tmp/uv-cache uv run python scripts/experiments/mujoco-vs-genesis-prerender/benchmark_validate_prerender_backends.py --scenario all --mode prerender --backend all --repetitions 1 --smoke-test-mode false --isolate-cache --force-cpu --mujoco-gl egl --timeout-sec 180
```

An additional smoke sanity run was also executed:

```bash
UV_CACHE_DIR=/tmp/uv-cache uv run python scripts/experiments/mujoco-vs-genesis-prerender/benchmark_validate_prerender_backends.py --scenario int101_like --mode validate --backend all --repetitions 1 --smoke-test-mode true --isolate-cache --force-cpu --timeout-sec 120
```

#### Environment caveats

- The run was CPU-mode only.
- `GENESIS_FORCE_CPU=1` was set in the benchmark runner.
- Cache directories were isolated per child run to reduce cross-run cache pollution.
- The benchmark ran in a constrained headless environment.
- MuJoCo renderer backend selection mattered in this environment:
  - `MUJOCO_GL=osmesa` produced renderer failures here,
  - `MUJOCO_GL=egl` produced stable comparable render outputs.
- The benchmark does not measure full simulation. It measures the static 24-view render path that validation currently performs.
- The benchmark is still a local experiment harness, not an HTTP-level integration test.

#### Measurements

Full 24-view render runs with `smoke-test-mode=false` produced valid artifacts for both backends.

Validation path results from `results-20260309T160007Z.json`:

- `int101_like`
  - Genesis: `70.4728s`
  - MuJoCo: `5.4807s`
  - MuJoCo speedup over Genesis: `12.86x`
- `int133_like`
  - Genesis: `70.4668s`
  - MuJoCo: `5.6497s`
  - MuJoCo speedup over Genesis: `12.47x`
- `multi_part_like`
  - Genesis: `72.1559s`
  - MuJoCo: `5.6620s`
  - MuJoCo speedup over Genesis: `12.74x`

Validation-path aggregate:

- average MuJoCo speedup over Genesis: `12.69x`
- Genesis wall time range: `70.47s` to `72.16s`
- MuJoCo wall time range: `5.48s` to `5.66s`

Direct prerender results from `results-20260309T160436Z.json`:

- `int101_like`
  - Genesis: `72.3167s`
  - MuJoCo: `5.6523s`
  - MuJoCo speedup over Genesis: `12.79x`
- `int133_like`
  - Genesis: `71.4315s`
  - MuJoCo: `5.8329s`
  - MuJoCo speedup over Genesis: `12.25x`
- `multi_part_like`
  - Genesis: `69.6513s`
  - MuJoCo: `5.7433s`
  - MuJoCo speedup over Genesis: `12.13x`

Direct-prerender aggregate:

- average MuJoCo speedup over Genesis: `12.39x`
- Genesis wall time range: `69.65s` to `72.32s`
- MuJoCo wall time range: `5.65s` to `5.83s`

Smoke-mode sanity result from `results-20260309T155533Z.json`:

- `int101_like` validate path
  - Genesis: `68.5799s`
  - MuJoCo: `0.8636s`
  - MuJoCo speedup over Genesis: `79.41x`

The smoke result is not representative of the real 24-view path because smoke mode reduces the render-view set.

#### Interpretation

This run supports the core optimization claim, but not the stronger numerical claim.

The measurements support the following conclusions:

1. MuJoCo is materially faster than Genesis for static 24-view validation prerendering.
2. The speedup is large and consistent across the tested scenarios.
3. The speedup is not approximately `100x` for the real 24-view path in this environment. The measured factor is closer to `12x`.
4. The "near-instant" claim is also not supported for the real 24-view path. MuJoCo reduced the path to roughly `5.5s`, not to sub-second latency.
5. The smoke-mode result can create an overly optimistic impression. It should not be used to justify product-facing expectations for the full validation render path.

The practical product interpretation is:

- switching validation preview rendering from Genesis to MuJoCo is still a strong optimization,
- the main benefit is moving validation from roughly `70s` to roughly `5-6s` in this benchmark,
- this is large enough to justify a backend split for validation preview if the implementation risk remains low,
- but additional work would still be required if the actual requirement is "near-instant" validation.

#### Decision state

The benchmark supports a narrowed architecture decision:

1. static preview rendering for `/benchmark/validate` is a good candidate to run in MuJoCo,
2. Genesis should remain the simulation backend where Genesis-only behavior is required,
3. product expectations should be updated to "order-of-magnitude faster" rather than "`100x` faster" or "near-instant" based on this evidence alone.

No production runtime behavior was changed as part of this benchmark entry.

#### Follow-up

The next concrete step, if we decide to implement this optimization, is:

1. narrow `/benchmark/validate` preview rendering to MuJoCo static prerendering,
2. keep the rest of validation semantics unchanged,
3. rerun the relevant integration coverage under both backend defaults to confirm no contract regressions,
4. only then re-measure end-to-end `/benchmark/validate` latency through the real heavy-worker boundary.

### 2026-03-09 16:24:07 GMT - fresh-child versus reused-child `simulate_subprocess(...)` benchmark

#### Goal

The goal of this run was to benchmark the actual optimization target directly:

1. Does reusing the simulation child process materially speed up the real `simulate_subprocess(...)` path?
2. Is the gain still present when using distinct bundles and distinct `session_id` values, so the result cannot be dismissed as same-session backend reuse?
3. Does reversing bundle order change the conclusion?

This run was intended to replace inference with direct evidence on the heavy-worker subprocess path.

#### Script or code path

The experiment assets were persisted in:

- `scripts/experiments/simulate-subprocess-reuse/README.md`
- `scripts/experiments/simulate-subprocess-reuse/benchmark_fresh_vs_warm_simulate_subprocess.py`
- `scripts/experiments/simulate-subprocess-reuse/results-fresh-vs-warm-simulate-subprocess-20260309T162407Z.json`

The benchmark drives the real `simulate_subprocess(...)` function inside spawned child executors.

It compares two modes:

1. current runtime behavior
   - one fresh `ProcessPoolExecutor(max_workers=1, max_tasks_per_child=1, initializer=init_genesis_worker)` per request
2. candidate runtime behavior
   - one reused `ProcessPoolExecutor(max_workers=1, initializer=init_genesis_worker)` handling two requests sequentially in one child

It runs two successful bundles in both orders:

- `A_then_B`
- `B_then_A`

It also assigns a distinct `session_id` to every bundle execution, including warm mode, so the measurement is not relying on same-session backend reuse.

#### Command

The benchmark was executed with:

```bash
GENESIS_FORCE_CPU=1 .venv/bin/python scripts/experiments/simulate-subprocess-reuse/benchmark_fresh_vs_warm_simulate_subprocess.py --force-cpu --repetitions 1
```

#### Environment caveats

- The run was CPU-mode only.
- `GENESIS_FORCE_CPU=1` was set.
- Cache roots (`HOME`, `XDG_CACHE_HOME`, Taichi cache paths) were isolated per child benchmark run to reduce cross-run disk-cache contamination.
- The benchmark is a subprocess harness, not an HTTP-level integration test.
- The two bundles are intentionally simple successful workloads, but they do use the real `simulate_subprocess(...)` path and its real heavy-worker runtime plumbing.

#### Measurements

`A_then_B`:

- fresh-child total wall time: `125.8094s`
- reused-child total wall time: `80.1602s`
- total speedup: `1.5695x`

Second request in `A_then_B`:

- fresh-child second request wall time: `32.0604s`
- reused-child second request wall time: `2.4374s`
- second-request speedup: `13.1537x`

`B_then_A`:

- fresh-child total wall time: `127.8717s`
- reused-child total wall time: `80.9276s`
- total speedup: `1.5801x`

Second request in `B_then_A`:

- fresh-child second request wall time: `31.4093s`
- reused-child second request wall time: `2.4990s`
- second-request speedup: `12.5686x`

Per-request decomposition for `A_then_B`:

- fresh-child first request
  - inner `simulate_subprocess(...)` elapsed: `45.8744s`
  - wall-clock request time: `89.8403s`
- fresh-child second request
  - inner `simulate_subprocess(...)` elapsed: `3.3729s`
  - wall-clock request time: `32.0604s`
- reused-child first request
  - inner `simulate_subprocess(...)` elapsed: `46.0931s`
  - wall-clock request time: `72.5194s`
- reused-child second request
  - inner `simulate_subprocess(...)` elapsed: `2.4369s`
  - wall-clock request time: `2.4374s`

Per-request decomposition for `B_then_A`:

- fresh-child first request
  - inner `simulate_subprocess(...)` elapsed: `49.1911s`
  - wall-clock request time: `92.9810s`
- fresh-child second request
  - inner `simulate_subprocess(...)` elapsed: `3.2789s`
  - wall-clock request time: `31.4093s`
- reused-child first request
  - inner `simulate_subprocess(...)` elapsed: `46.7417s`
  - wall-clock request time: `73.2532s`
- reused-child second request
  - inner `simulate_subprocess(...)` elapsed: `2.4985s`
  - wall-clock request time: `2.4990s`

#### Interpretation

This run is the strongest direct evidence gathered so far for the persistent-child design.

The results support the following conclusions:

1. Reusing the simulation child process materially improves the real heavy-worker subprocess path.
2. The effect survives bundle-order reversal. This is not a one-bundle artifact.
3. The effect also survives distinct `session_id` values. This is not same-session backend-cache cheating.
4. The second-request gain is the cleanest signal:
   - fresh-child mode still spends about `31s` to `32s` of wall time on the second request,
   - reused-child mode reduces that second request to about `2.4s` to `2.5s`.
5. The inner `simulate_subprocess(...)` elapsed time for the second request is already small in both modes, roughly `2.4s` to `3.4s`. The repeated penalty is therefore mostly outside the useful simulation work and inside process lifecycle and warm-runtime loss.

The practical conclusion is explicit:

- the current repeated cost is not explained by the literal `gs.init()` call,
- and it is not explained by same-session backend reuse,
- it is primarily explained by repeatedly creating and discarding the simulation child process and its warm runtime state.

#### Decision state

This benchmark is treated as sufficient evidence to proceed with the persistent-child `simulation_runner.py` refactor planning.

The previously open question, "does child reuse matter on the real subprocess path?", is now answered in the affirmative.

No further cleanup-variant benchmark is required before planning the refactor.

#### Follow-up

The next concrete step is to write the refactor plan and then implement the persistent-child runtime change under the existing single-flight worker contract.

### 2026-03-09 16:30:38 GMT - implementation plan for persistent-child `simulation_runner.py` refactor

#### Goal

The goal of this plan is to translate the benchmark evidence into a bounded runtime refactor that improves performance without breaking the current worker contract.

The refactor target is narrow:

- keep `worker-heavy` single-flight,
- keep the process boundary between FastAPI and the physics runtime,
- change only the lifecycle of the simulation child from per-request to persistent-per-worker-instance.

#### Script or code path

This plan targets the following runtime files and code paths:

- `worker_heavy/runtime/simulation_runner.py`
- `worker_heavy/api/routes.py`
- `worker_heavy/app.py`
- `worker_heavy/simulation/factory.py`
- direct HTTP simulation and validation adapter paths,
- Temporal heavy-activity simulation and validation paths that already share the same runner.

#### Command

No command was executed for this entry. This is a design and planning record derived from the benchmark evidence above.

#### Environment caveats

- The benchmark evidence used for this plan was CPU-mode only.
- The plan is still valid because the runtime topology problem is process lifecycle, not a CPU-only artifact.
- The plan intentionally does not include scene-cache or geometry-cache work in the same change.

#### Measurements

The plan is justified by the benchmark immediately above it.

The decisive numbers are:

- fresh-child totals: `125.8s` to `127.9s`
- reused-child totals: `80.2s` to `80.9s`
- fresh-child second request: `31.4s` to `32.1s`
- reused-child second request: `2.4s` to `2.5s`

#### Interpretation

The refactor target is now clear.

The runtime change must optimize process reuse, not thread reuse and not same-session cache reuse.

The first implementation should preserve external behavior while changing only child lifecycle.

#### Decision state

This plan is accepted as the implementation direction for the first persistent-child refactor.

#### Follow-up

The implementation order is:

1. Introduce a persistent executor manager in `worker_heavy/runtime/simulation_runner.py`.
2. Submit `simulate_subprocess(...)` and `validate_subprocess(...)` work through that manager instead of per-request `with ProcessPoolExecutor(...)` scopes.
3. Add explicit child-callable cleanup helpers that clear cached session backends without killing the warm process.
4. Route `/internal/simulation/cleanup` into that child-side cleanup path.
5. Add shutdown-time executor disposal in `worker_heavy/app.py`.
6. Add fail-closed executor recreation on `BrokenProcessPool`.
7. Add executor lifecycle observability events.
8. Rerun integration coverage and the direct subprocess benchmark after the refactor.

#### Detailed implementation plan

##### Invariants to preserve

The following behavior must remain unchanged:

1. public `/benchmark/simulate` and `/benchmark/validate` request and response contracts,
2. `/ready` and `heavy_operation_admission` single-flight semantics,
3. fail-closed behavior on child crash,
4. session-root filesystem contract,
5. external artifact collection behavior.

The following behavior will intentionally change:

1. the simulation child remains alive after a request completes,
2. Genesis process-global warm state survives across requests on the same worker instance,
3. cleanup becomes an explicit child-side operation instead of mostly relying on child process exit.

##### Scope of phase 1

Phase 1 scope is:

1. `worker_heavy/runtime/simulation_runner.py`,
2. `/benchmark/simulate`,
3. `/benchmark/validate`,
4. `/internal/simulation/cleanup`,
5. worker shutdown handling.

Phase 1 explicitly excludes:

1. `/benchmark/verify`,
2. changes to HTTP request or response models,
3. changes to single-flight admission logic,
4. scene-geometry caching,
5. compiled-scene caching.

##### Planned runtime design

Phase 1 runtime design is:

1. Create one persistent child executor per worker instance.
2. Submit `simulate_subprocess(...)` and `validate_subprocess(...)` work into that executor sequentially.
3. Reuse the same child process across heavy requests until crash, explicit shutdown, or explicit recycle policy.
4. Use distinct `session_id` values as today. Do not collapse requests into one fake shared session.

The implementation should introduce a small dedicated manager, for example `SimulationExecutorManager`, responsible for:

1. lazy creation of the persistent executor,
2. serialized submission of subprocess tasks,
3. child-health tracking,
4. cleanup submission,
5. executor shutdown and recreation.

##### File-level implementation plan

`worker_heavy/runtime/simulation_runner.py`

1. Remove per-request `with ProcessPoolExecutor(...)` creation for simulate and validate.
2. Replace it with access through a persistent executor manager.
3. Keep `init_genesis_worker()` as the child-process initializer.
4. Add explicit child-callable cleanup functions, for example:
   - cleanup cached session backends,
   - optionally `gc.collect()`,
   - but do not tear down Genesis or exit the child.

`worker_heavy/simulation/factory.py`

1. Keep backend caching behavior explicit.
2. Expose child-side cleanup helpers that can safely clear session-local backend state.
3. Do not rely on the parent-process cleanup route for child cleanup any more.

`worker_heavy/api/routes.py`

1. Keep heavy admission behavior unchanged.
2. Route `/internal/simulation/cleanup` into the persistent-child cleanup path.
3. Preserve current response semantics.

`worker_heavy/app.py`

1. Add shutdown-time executor disposal.
2. Ensure worker exit terminates the persistent child cleanly.

##### Cleanup plan

Phase 1 cleanup should be conservative.

The first implementation should prefer safety over maximum caching:

1. keep the child process alive,
2. clear child-side session backend cache explicitly when cleanup is requested,
3. avoid re-calling `gs.init()` or rebuilding the entire process unless the executor is recycled.

This means phase 1 cleanup preserves:

- process-global Genesis warm state,
- child interpreter warm state,
- child process lifetime.

But it clears:

- cached session backends,
- session-local runtime objects that could leak across requests.

##### Crash and recycle policy

The runtime must fail closed:

1. If task submission raises `BrokenProcessPool`, the current request fails deterministically.
2. The manager marks the executor as broken and discards it.
3. The next heavy request lazily creates a new child.

Optional bounded recycling is allowed after phase 1, but it is not required to land the initial refactor.

If recycling is added later, it should be based on explicit policy such as:

1. maximum requests served per child,
2. RSS threshold,
3. known bad-state detection.

##### Observability additions

The refactor should add explicit executor lifecycle events, for example:

1. `simulation_executor_started`
2. `simulation_executor_reused`
3. `simulation_executor_cleanup`
4. `simulation_executor_shutdown`
5. `simulation_executor_crashed`
6. `simulation_executor_recreated`

These events should include at minimum:

- worker session or request identifier,
- executor generation identifier,
- operation type (`simulate` or `validate`),
- reason (`startup`, `cleanup`, `crash`, `shutdown`, `recycle`).

##### Acceptance criteria for the refactor

The refactor is considered acceptable only if all of the following hold:

1. Existing integration behavior for `simulate` and `validate` still passes.
2. Single-flight admission behavior is unchanged.
3. Child crash behavior remains fail-closed.
4. `/internal/simulation/cleanup` still succeeds and now cleans the persistent child state explicitly.
5. A rerun of the direct `simulate_subprocess(...)` benchmark still shows the second-request collapse relative to fresh-child mode.

##### Rejected alternatives for phase 1

The following alternatives are explicitly rejected for this first refactor:

1. Initializing Genesis in the FastAPI parent process.
2. Sharing Genesis across arbitrary request threads.
3. Expanding worker-heavy beyond single-flight.
4. Bundling scene-cache or geometry-cache work into the same change.

The reason is scope control:

- the benchmark evidence already justifies process reuse,
- while thread-sharing and scene-caching introduce different risks and should not be conflated with the first runtime lifecycle fix.

### 2026-03-09 21:15:49 GMT - integration cache-path stability benchmark

#### Goal

The goal of this benchmark was to answer the cache-identity question raised by integration-test reruns:

1. whether stable `INT-###` session identifiers materially improve repeated Genesis reuse across fresh-child requests,
2. whether stable workspace and bundle extraction paths matter more than the `session_id`,
3. whether both dimensions are required before on-disk cache reuse appears.

This benchmark was intentionally narrower than a full integration run.

It is meant to separate cache-key hypotheses before changing the real integration runner or heavy-worker request plumbing.

#### Script or code path

The experiment assets are:

- `scripts/experiments/integration-cache-path-stability/README.md`
- `scripts/experiments/integration-cache-path-stability/benchmark_integration_cache_path_stability.py`
- `scripts/experiments/integration-cache-path-stability/results-int101-like-20260309T210926Z.json`
- `scripts/experiments/integration-cache-path-stability/results-int133-like-20260309T211549Z.json`

The benchmark drives the real `simulate_subprocess(...)` path in fresh child processes.

It compares four stability modes:

1. random workspace path plus random `INT-###-uuid8` session id,
2. stable workspace path plus random `INT-###-uuid8` session id,
3. random workspace path plus stable `INT-###` session id,
4. stable workspace path plus stable `INT-###` session id.

The benchmark keeps the cache root stable within each scenario and mode while clearing the workspace contents between repetitions. This is intended to preserve path identity without allowing stale output files to become an accidental shortcut.

#### Command

The executed commands were:

```bash
GENESIS_FORCE_CPU=1 .venv/bin/python scripts/experiments/integration-cache-path-stability/benchmark_integration_cache_path_stability.py --force-cpu --backend GENESIS --scenario int101_like --repetitions 2 --output /tmp/int101-cache-path-stability.json

GENESIS_FORCE_CPU=1 .venv/bin/python scripts/experiments/integration-cache-path-stability/benchmark_integration_cache_path_stability.py --force-cpu --backend GENESIS --scenario int133_like --repetitions 2 --output /tmp/int133-cache-path-stability.json
```

#### Environment caveats

- The run was CPU-mode only.
- The benchmark is a subprocess harness, not an HTTP-level integration test.
- The benchmark intentionally uses small successful smoke-style scenarios so repeated runs are feasible.
- The benchmark isolates cache roots per scenario and mode to avoid cross-mode contamination while still allowing repeated-run reuse inside one scenario-mode lane.
- The benchmark exercises the real `simulate_subprocess(...)` path, not the full controller -> bundle -> heavy-worker HTTP path.
- The benchmark explicitly set deterministic `HOME`, `XDG_CACHE_HOME`, Taichi cache variables, and temp directories inside each scenario-mode lane. This means the benchmark did control the Genesis/Taichi disk-cache root explicitly.

#### Measurements

`int101_like`:

- random workspace + random session
  - first run: `70.347s`
  - repeat run: `12.142s`
- stable workspace + random session
  - first run: `68.585s`
  - repeat run: `13.134s`
  - repeat speedup vs baseline: `0.924x`
- random workspace + stable session
  - first run: `68.433s`
  - repeat run: `11.508s`
  - repeat speedup vs baseline: `1.055x`
- stable workspace + stable session
  - first run: `67.637s`
  - repeat run: `12.624s`
  - repeat speedup vs baseline: `0.962x`

`int133_like`:

- random workspace + random session
  - first run: `64.928s`
  - repeat run: `11.293s`
- stable workspace + random session
  - first run: `68.105s`
  - repeat run: `12.129s`
  - repeat speedup vs baseline: `0.931x`
- random workspace + stable session
  - first run: `69.655s`
  - repeat run: `11.929s`
  - repeat speedup vs baseline: `0.947x`
- stable workspace + stable session
  - first run: `69.681s`
  - repeat run: `12.082s`
  - repeat speedup vs baseline: `0.935x`

#### Interpretation

The benchmark answers one architectural question clearly enough for the current subprocess path:

1. repeated fresh-child runs already collapse from roughly `65s` to `70s` down to roughly `11s` to `13s` on the second run even in the all-random baseline,
2. neither stable workspace path nor stable `INT-###` session id provides a material repeat-run improvement over that baseline on these two scenarios,
3. the measured differences between modes are small enough to treat as noise for the present decision.

The current hypothesis therefore narrows to:

- random `session_id` values are not the dominant blocker for cache reuse on this direct `simulate_subprocess(...)` path,
- stable workspace paths are also not showing a useful win on this direct path for these scenarios,
- the remaining integration slowness is more likely explained by another layer such as process lifecycle, a different runtime path, or a cache key dimension not exercised by these small smoke scenarios.

The runtime-plumbing clarification from the follow-up review is:

1. integration tests now do preserve heavy-worker process reuse because the persistent simulation child stays alive across requests and across tests unless it crashes,
2. integration teardown still clears cached backend instances after each test, so integration tests do not preserve same-session backend-object reuse across tests,
3. this benchmark therefore rules out session-id and workspace-path identity as the dominant factor on the direct subprocess path, but it does not answer the full integration-path question where bundle extraction, worker session-root routing, and other HTTP-layer plumbing are involved,
4. this benchmark also does not prove that ten materially different Genesis scenes will all receive the same benefit; it only shows that identical repeated smoke workloads already reuse enough lower-level cache even in the all-random baseline.

#### Decision state

The hypothesis "integration rerun slowness is primarily caused by random `INT-###-uuid8` session ids or random workspace paths on the direct `simulate_subprocess(...)` path" is not supported by this benchmark.

The benchmark supports a narrower decision:

1. do not implement stable `INT-###` session-id canonicalization solely on the basis of this cache hypothesis,
2. do not implement stable workspace-path canonicalization solely on the basis of this benchmark.

This decision is specific to the direct `simulate_subprocess(...)` path with controlled cache roots. It is not a blanket statement about every higher-level integration path.

#### Follow-up

The next concrete step is:

1. if integration reruns are still observably slow, benchmark the full HTTP integration path where bundle extraction and worker session-root routing are involved,
2. separately continue prioritizing the persistent-child runtime work, because the earlier direct subprocess benchmark already showed that process lifecycle dominates repeated cost,
3. if the unresolved question is "does the tenth materially different Genesis scene still benefit after the first one", run a separate benchmark over many distinct randomized scenes instead of repeating one smoke scenario,
4. only revisit path/session canonicalization if a higher-level benchmark shows a regression that this direct subprocess benchmark does not capture.

#### Cache configuration note

The current local Genesis `0.4.1` cache lookup in this environment is:

1. `GS_CACHE_FILE_PATH` when set,
2. on Linux, `XDG_CACHE_HOME/genesis` when `XDG_CACHE_HOME` is set,
3. otherwise `~/.cache/genesis`.

The `gsd` cache directory is `<cache_dir>/gsd`.

This matters for runtime diagnosis because:

1. the experiment did set cache-root environment variables explicitly,
2. the current integration runner does not set `GS_CACHE_FILE_PATH` or `XDG_CACHE_HOME`,
3. therefore real integration runs currently inherit the Genesis disk-cache root from the ambient process environment unless a wrapper or shell setup overrides it.

## Template for future entries

Use the following structure for future simulation optimization records.

### YYYY-MM-DD HH:MM:SS TZ - short title

#### Goal

State the question or performance issue being tested.

#### Script or code path

State the benchmark script, integration test, or runtime path used.

#### Command

Write the exact command.

#### Environment caveats

List backend mode, GPU/CPU mode, sandbox limitations, and any other relevant caveats.

#### Measurements

Write the raw or summarized measured output.

#### Interpretation

State what the measurement means and what it does not prove.

#### Decision state

Write whether a change was accepted, rejected, deferred, or narrowed.

#### Follow-up

Write the next concrete step or the explicit reason for no next step.
