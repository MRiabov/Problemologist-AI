# Distributed execution

## Scope summary

- Primary focus: controller and split-worker deployment architecture.
- Defines `worker-light` vs `worker-heavy` responsibilities, worker APIs, and routing contract.
- Covers persistence/storage expectations, concurrency model, and Temporal orchestration boundary.
- Use this file when changing infra topology or controller-to-worker integration behavior.

There is a controller node which runs the LLM and tool calls, and a split worker plane:

1. `worker-light` for filesystem + execution tooling,
2. `worker-heavy` for simulation + validation + rendering,
3. `controller-worker` (Temporal worker) for durable orchestration.

For both safety and performance reasons, it desirable that the LLM-generated scripts are never executed on the controller machine.

In the future we may well refactor to run on distributed nodes, perhaps even IPv6.

## Current service topology (main)

- `controller` (FastAPI): public API, LLM/tool orchestration, business logic.
- `controller-worker` (Temporal worker): durable execution for long-running workflows/retries and heavy-task queue consumption/dispatch.
- `worker-light` (FastAPI): session filesystem, git, linting, runtime execution, static asset serving.
- `worker-heavy` (FastAPI single-flight executor): simulation, validation, rendering, manufacturability analysis, review handover.
- Shared dependencies: `temporal`, `postgres`, `minio`.

The split is intentional: keep fast, high-throughput operations on light infra while isolating heavy physics/render workloads onto dedicated nodes.

## Worker API

The worker API is physically split into two specialized services to optimize resource allocation and separate concerns: **Worker Light** and **Worker Heavy**.

### Worker Light

- **Purpose**: Handles lightweight, synchronous operations and filesystem management.
- **Responsibilities**:
  - Filesystem CRUD operations (`/fs/*`).
  - Git repository management (`/git/*`).
  - Python code execution (short-lived) (`/runtime/execute`).
  - Asset serving (`/assets/*`).
  - Code linting (`/lint`).
  - Topology inspection (`/topology/inspect`).
- **HTTP Boundary**: Typically exposed on port 18001.
- **Operational profile**: High request-rate, short-lived operations, no heavy physics kernels.

### Worker Heavy

- **Purpose**: Handles compute-intensive, long-running tasks.
- **Responsibilities**:
  - Physics simulation (`/benchmark/simulate`).
  - Geometric validation plus static preview generation (`/benchmark/validate`).
  - Design handover and DFM checks (`/benchmark/submit`).
  - Manufacturing analysis (`/benchmark/analyze`).
  - Rendering and preview generation (`/benchmark/preview`).
  - Asset building (`/benchmark/build`).
  - Single-flight execution gate: one active heavy job per worker instance.
- **HTTP Boundary**: Typically exposed on port 18002.
- **Operational profile**: one active external heavy job at a time per instance, no in-process multi-request scheduler.

### Routing Contract (Controller -> Workers)

- Controller routes light operations to light worker, (default light worker endpoint). The light worker executes scripts, which can ping the load balancer handling heavy workers.
- Controller routes heavy operations through Temporal workflows, not directly to `WORKER_HEAVY_URL`.
- All worker calls are session-scoped with `X-Session-ID`.
- The `WorkerClient` is the single boundary adapter; agents do not know about service split.
- Heavy-worker admission is fail-closed: busy workers return deterministic busy responses; they do not enqueue additional work internally.

### Heavy Execution Path Contract

Heavy compute execution has one production path:

- Controller tools call Temporal workflows for heavy operations.
- Temporal workflows dispatch heavy activities on `heavy-tasks-queue`.
- Heavy activity execution runs simulation/validation/render in isolated child process scope (crash containment boundary).

Backend responsibility is split by operation purpose:

1. `/benchmark/simulate` uses the selected physics backend.
2. `/benchmark/validate` performs fast validation plus static preview generation.
3. Static preview generation for `/benchmark/validate` uses MuJoCo by default even when `physics.backend=genesis`.
4. `/benchmark/validate` does not add a separate Genesis load/render gate solely for parity checking; Genesis-specific runtime behavior is established by actual Genesis simulation runs where Genesis behavior is required.

Direct `worker-heavy` HTTP endpoints (`/benchmark/*`) are reserved for integration tests that verify worker-level boundaries, not an alternate orchestration model with independent queueing semantics.

Temporal task delivery remains worker-poll based (long-poll task queues), while workflow completion is returned through Temporal result APIs. We do not define a Temporal-server-push webhook delivery mode for activity task dispatch in this architecture.

### Shared Worker Modules

To avoid drift between services, common models and logic are in shared modules:

- `shared/workers/schema.py` - request/response schemas.
- `shared/workers/filesystem/*` - session filesystem backend/router.
- `shared/workers/workbench_models.py` - manufacturing/workbench contracts.
- `shared/workers/loader.py`, `persistence.py`, `markdown_validator.py` - shared execution helpers.

### Worker logic split

Worker-specific logic stays in:

- `worker_light/*` for lightweight operations and design validation (e.g. intersection checks) that normally would be executed \<1s.:
  - FS
  - runtime
  - git
  - build123d validation (and maybe, lightweight rendering *in the future*);
  - DFM validation
  - linting
- `worker_heavy/*` for:
  - heavy simulation
  - validation
  - rendering.
  - any GPU work, if necessary.

Validation is still part of `worker-heavy`, but the static preview part of validation is intentionally lighter than Genesis simulation. The point of the split is to avoid paying unnecessary Genesis render cost on validation-only requests.

Note: we want to offload work from `worker_heavy` as much as possible because:

- Network communication
- Boot time is slow, if need to up more workers it takes time.
- We don't want to load CPU-bound processes with lots of small requests
- If there are GPUs, they are more expensive.

### Worker filesystem and communication

Upon requesting simulation or rendering, worker light prepares the session bundle and git state needed for heavy execution. The controller-worker Temporal path owns the actual heavy dispatch, retry, and completion tracking.

Predominantly, worker light communicates with controller-worker orchestration for heavy dispatch; direct worker-heavy contact is reserved for integration tests that verify worker-level boundaries and for termination signals from the controller. There is a load balancer pinging `/ready` status in worker. Ideally, the worker-heavy is hidden behind Temporal/controller-worker orchestration, which also acts as the admission and retry boundary.

### OpenAPI Artifacts

- `controller_openapi.json` documents the controller surface.
- Worker OpenAPI generation must include both worker surfaces (light and heavy) or provide separate specs (`worker_light_openapi.json`, `worker_heavy_openapi.json`) to avoid missing benchmark endpoints from generated artifacts.
- If a single `worker_openapi.json` is kept, it must be a merged schema, not just `worker_light` endpoints.

### Concurrency and Parallelism

Heavy-worker concurrency is a single-flight contract:

- A heavy worker instance can execute exactly one active job at a time.
- The heavy worker app does not own internal multi-job management (no in-process queue/semaphore/scheduler for multiple external jobs).
- If a request arrives while a job is active, the heavy worker returns a deterministic busy response (`503` + machine-readable reason such as `WORKER_BUSY`) instead of buffering jobs.
- `/ready` and equivalent readiness signals must report `not ready` while a job is active, so load balancers route only to idle instances.
- Any queueing, retries, and fan-out for concurrent demand are owned outside `worker-heavy` (Temporal/controller-worker orchestration and infrastructure load balancing), not by heavy-worker app internals.
- Cross-worker fan-out/scaling policy is intentionally out of scope for this phase; the enforced contract here is per-instance single-flight admission plus deterministic busy behavior.
- Runtime randomization executes within one admitted heavy-worker job.
- That admitted job corresponds to one backend simulation run over a batched set of parallel scenes/environments, not to N serialized full simulation replays.
- The heavy worker must compile/build/load the simulation scene once per admitted runtime-randomization request (or reuse an equivalent compiled-scene cache hit), then instantiate `num_scenes` jittered scene variants from that compiled state.
- `num_scenes` is batch width inside one backend run. It is not permission to run multiple heavy jobs on one worker instance.
- The jittered scene variants execute in parallel inside that one admitted job when enough RAM is available. Parallel batched execution is mandatory when backend-supported reuse is available.
- Recompiling or replaying the entire scene serially per jitter seed is non-compliant with this architecture.
- Temporal heavy-task dispatch is worker-poll task queue delivery. Webhook-style triggers, if used, are external orchestration inputs and not Temporal server task-delivery mode.
- FastAPI API serving and Temporal heavy-activity polling/execution must be process-isolated (separate process/container boundary) so a simulation child-process crash cannot terminate the API process.
- Optional startup pre-warming is still allowed per worker instance to reduce first-run backend/JIT latency.

## Persistent state and durable execution

To simplify app logic and avoid writing retry and other "safety" logic, we will deploy a small `temporal` instance running on a separate small machine next to the main app.

## Hyperscaler of choice

We are deploying to Railway. (In production, we may deploy workers to inexpensive, *batch* IPv6 nodes - SaladCloud, or use a on-prem infra that somebody will give to me...)

### Deployment specifics

Railway supports docker-compose import and we will start with the docker-compose. This is the easiest choice and I have familiarity with it (k8s too bloated for this purpose)

### Podman containers

We decided to run in Podman containers because they are leaner than Docker and serve all same purposes. Worker containers have an attached volume.

#### `uv` base containers

We will use astral-sh/uv as base containers and not standard python ones (for quality).

## Persisting files

The files are written directly to the worker container. We don't store it on controller. However, we upload final results ("Assets") to the Railway bucket S3.

The worker's filesystem is implemented as a "disposable sandbox" via `SandboxFilesystemBackend`.

The "main app" essentially serves as a business logic layer that also forwards requests to observability layer, but the actual execution - from linting to simulation - happens in the worker container.

### Workers' filesystem

`FilesystemMiddleware` (which previously was a dependency from some architecture, now removed and migrated to a standalone class) supports a "sandbox" filesystem backend. This is handy, and we will expose the workers' filesystem as sandbox - something that can easily be removed.

Notably, the files can be created locally (e.g. video, image, MJCF outputs), and something should be done about it.

Each `X-Session-ID` maps to its own isolated temp workspace root. Light and heavy services resolve the same session root contract for cross-service continuity.

Read/write split:

- Workspace files (`/`) are read-write.
- Mounted paths (`/utils`, `/skills`, `/reviews`, `/config`) are read-only.

Mount paths must be defined explicitly for split workers (light/heavy/shared) and tested to avoid dead mounts after refactors.

Skills sync is startup-configurable; in integration tests we can use local deterministic skills paths.

For videos and large files, we will also use a `CompositeBackend`. It will route the `/render/` folder to the s3; we will need internal plumbing to make this happen (presumably, any python function that will render, will upload to s3).

### Videos

Videos are the largest artifact that we will need to generate and store.
To ensure consistency, we will upload them to Railway buckets.
This is OK as we already work on Railway and internal networking both fast, easy to set up and inexpensive.
Storing videos on buckets is also cheaper than in volumes.

I suppose the videos will be automatically deleted after a short period, e.g. a day to avoid storage costs. <!--(why not store them in ephemeral storage then?...)-->

Videos are rendered on-demand and only if the model requests it (I'll need to double-check if is preferable or not)
Why: models don't strictly always need to view videos; they might want to view only coordinates in the end (videos can be rendered by a variable in `simulate(component, render=True)`) this is in part to make video.

## Database

We do persistence via SQLAlchemy and Alembic migrations to avoid issues with hand-written SQL.

<!-- If something is a long-running process, prefer to persist it to a database and receive a callback. -->

<!-- Solved via Temporal.  -->

All important updates must be persisted into the DB (for observability, as below.)

## Agent and Worker boundary

### Separation

The agent runtime (DSPy.ReAct modules/signatures inside LangGraph-managed orchestration) never "knows" about distributed workers. It only calls an async Python function (a tool). It is the job of the tool to dispatch a job and handle retries. In this case, durable retries and persistence are handled by Temporal, while LLM call retries are handled in our runtime wrappers.

### Temporal

Temporal is used to orchestrate the workers. It is not used to run or retry the agent.

Temporal needs a database and we will use the Postgres database used by temporal, except under the different `DATABASE` partition.

Because tasks like simulation can be long-running, completion is tracked through Temporal workflow/activity result state (and associated persisted events), not worker-local ad-hoc queue state.

### Failed tool calls

We classify tool-call failures into two types: agent-failed and system-failed.

- Agent-failed: deterministic tool-contract or generated-artifact errors caused by LM output (for example invalid tool arguments, writing a file without required overwrite semantics, generated build123d code that fails to compile).
- System-failed: infrastructure/transport failure where execution could not be attempted reliably (for example worker filesystem/API unreachable, transport timeout, transient worker/service 5xx).

On agent-failed, we return a tool error observation to the LM so it can revise and continue within normal LM turn/token/time budgets. On system-failed, we retry with Temporal up to 3 times for all agents. If the same system-failed condition persists across 3 retries for the same tool request in the same stage, we log a `CRITICAL` error and terminate the episode.

System-failed retries do not count toward LM tool-call budget because they are independent of LM quality. They still count toward episode wall-clock timeout.
