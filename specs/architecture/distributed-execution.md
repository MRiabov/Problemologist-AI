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
- `controller-worker` (Temporal worker): durable execution for long-running workflows and retries.
- `worker-light` (FastAPI): session filesystem, git, linting, runtime execution, static asset serving.
- `worker-heavy` (FastAPI + process pool): simulation, validation, rendering, manufacturability analysis, review handover.
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
  - Geometric validation (`/benchmark/validate`).
  - Design handover and DFM checks (`/benchmark/submit`).
  - Manufacturing analysis (`/benchmark/analyze`).
  - Rendering and preview generation (`/benchmark/preview`).
  - Asset building (`/benchmark/build`).
  - Temporal Activity Worker (consumes from `heavy-tasks-queue`).
- **HTTP Boundary**: Typically exposed on port 18002.
- **Operational profile**: low parallelism by design (`max_workers=1` for heavy simulation process pool on CPU-bound infra).

### Routing Contract (Controller -> Workers)

- Controller routes light operations to light worker, (default light worker endpoint). The light worker executes scripts, which can ping the load balancer handling heavy workers.
- Controller routes heavy operations to `WORKER_HEAVY_URL`.
- All worker calls are session-scoped with `X-Session-ID`.
- The `WorkerClient` is the single boundary adapter; agents do not know about service split.

### Shared Worker Modules

To avoid drift between services, common models and logic are in shared modules:

- `shared/workers/schema.py` - request/response schemas.
- `shared/workers/filesystem/*` - session filesystem backend/router.
- `shared/workers/workbench_models.py` - manufacturing/workbench contracts.
- `shared/workers/loader.py`, `persistence.py`, `markdown_validator.py` - shared execution helpers.

### Worker logic split

Worker-specific logic stays in:

- `worker_light/*` for lightweight operations and design validation (e.g. intersection checks) that normally would be executed <1s.:
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

Note: we want to offload work from `worker_heavy` as much as possible because:

- Network communication
- Boot time is slow, if need to up more workers it takes time.
- We don't want to load CPU-bound processes with lots of small requests
- If there are GPUs, they are more expensive.

### Worker filesystem and communication

Upon requesting simulation or rendering, worker light will send a gzip file of all necessary code to worker. The worker light will also git commit when sending a heavy simulation.

Predominantly, worker light communicates with worker heavy directly; with exceptions of termination signals from controller. There is a load balancer pinging `/ready` status in worker. Ideally, the worker-heavy is hidden behind a Temporal setup, which also can act as the load balancer.

### OpenAPI Artifacts

- `controller_openapi.json` documents the controller surface.
- Worker OpenAPI generation must include both worker surfaces (light and heavy) or provide separate specs (`worker_light_openapi.json`, `worker_heavy_openapi.json`) to avoid missing benchmark endpoints from generated artifacts.
- If a single `worker_openapi.json` is kept, it must be a merged schema, not just `worker_light` endpoints.

### Concurrency and Parallelism

To support concurrent simulation requests (e.g., during integration testing or parallel benchmarking), the worker utilizes a **Process-level isolation** strategy:

- **ProcessPoolExecutor**: Simulations are offloaded to a `ProcessPoolExecutor`.
- **Backend Isolation**: This is strictly required for the `Genesis` physics backend, which enforces execution on the "main thread" and has global state issues. Process isolation ensures each simulation runs in a fresh environment.
- **Resource Cleanup**: We use `max_tasks_per_child=None` in the executor to allow for process reuse. This is critical for Genesis, as it enables the caching of JIT-compiled kernels across multiple simulation requests within the same process.
- **Pre-warming**: To eliminate the ~50s JIT compilation delay during the first user request, the worker pool uses an `initializer` (`_init_genesis_worker`) that builds a tiny dummy scene immediately upon process startup.
- **CPU Scaling**: On CPU-only hardware, `max_workers` is restricted to 1 for simulations to prevent resource contention during heavy kernel compilation and voxelization phases.

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

Because tasks like simulation (with involve both simulation and uploading to the database) could be long-running we are using webhooks and callbacks to report their completion.

### Failed tool calls

We classify tool-call failures into two types: agent-failed and system-failed.

- Agent-failed: deterministic tool-contract or generated-artifact errors caused by LM output (for example invalid tool arguments, writing a file without required overwrite semantics, generated build123d code that fails to compile).
- System-failed: infrastructure/transport failure where execution could not be attempted reliably (for example worker filesystem/API unreachable, transport timeout, transient worker/service 5xx).

On agent-failed, we return a tool error observation to the LM so it can revise and continue within normal LM turn/token/time budgets. On system-failed, we retry with Temporal up to 3 times for all agents. If the same system-failed condition persists across 3 retries for the same tool request in the same stage, we log a `CRITICAL` error and terminate the episode.

System-failed retries do not count toward LM tool-call budget because they are independent of LM quality. They still count toward episode wall-clock timeout.
