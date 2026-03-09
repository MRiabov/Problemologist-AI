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
- `controller-worker` / Temporal handles durable workflow and activity execution.

This matches the split-worker contract in `specs/architecture/distributed-execution.md`.

#### External behavior

- `worker-heavy` is single-flight.
- The worker accepts one active heavy job at a time.
- While a heavy job is running, `/ready` returns busy and the load balancer should stop routing new heavy jobs to that worker.
- The direct `/benchmark/*` endpoints are an adapter/debug surface, not a separate production queuing model.

#### HTTP adapter path for simulation

The direct path is:

1. Client or test sends `POST /benchmark/simulate`.
2. `worker-heavy` enters `heavy_operation_admission`.
3. The request bundle and session root are resolved.
4. `run_simulation_task(...)` calls `run_simulation_in_isolated_process(...)`.
5. The runtime creates a fresh `ProcessPoolExecutor(max_workers=1, max_tasks_per_child=1, initializer=init_genesis_worker)`.
6. A fresh child process starts.
7. The child runs `init_genesis_worker()`.
8. The child imports Genesis, resolves CPU or GPU, calls `gs.init(...)`, builds a tiny scene, and then runs the simulation subprocess function.
9. The child returns the simulation result.
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
