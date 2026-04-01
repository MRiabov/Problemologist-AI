# Distributed execution

## Scope summary

- Primary focus: controller and split-worker deployment architecture.
- Defines `worker-light`, `worker-heavy`, and `worker-renderer` responsibilities, worker APIs, and routing contract.
- Covers persistence/storage expectations, concurrency model, WebSocket control-plane transport, and Temporal orchestration boundary.
- Use this file when changing infra topology or controller-to-worker integration behavior.

The controller remains the orchestration layer. The LLM/tool substrate under that controller can be either API-backed or Codex CLI-backed, but that is a backend choice, not a change to the controller's role.

Terminology is strict:

- The controller is not a worker. It owns orchestration, tool routing, and public API behavior.
- `worker-light`, `worker-heavy`, and `worker-renderer` are worker services.
- The Temporal worker is a separate background service that executes durable workflows and activities on behalf of the controller. The compose service may still be named `controller-worker`, but that name is an implementation label, not the controller role.

There is a controller node which runs the LLM and tool calls, and a split worker plane:

1. `worker-light` for filesystem + execution tooling,
2. `worker-heavy` for simulation + validation coordination,
3. `worker-renderer` for all static and dynamic rendering,
4. Temporal worker service (`controller-worker` in compose) for durable orchestration.

For both safety and performance reasons, it is desirable that the LLM-generated scripts are never executed on the controller machine.

Frontend-to-controller and controller-to-worker-light control traffic uses WebSockets. Temporal remains the exception for workflow and activity delivery.

In the future we may well refactor to run on distributed nodes, perhaps even IPv6.

## Current service topology (main)

- `controller` (FastAPI): public API, LLM/tool orchestration, business logic.
- Temporal worker service (`controller-worker` in compose): durable execution for long-running workflows/retries and heavy-task queue consumption/dispatch.
- `worker-light` (FastAPI): session filesystem, git, linting, runtime execution, static asset serving.
- `worker-heavy` (FastAPI single-flight executor): simulation, validation coordination, manufacturability analysis, review handover.
- `worker-renderer` (FastAPI single-flight executor): headless rendering for static preview, selection snapshots, and simulation videos. Unlike `worker-light` and `worker-heavy`, it stays in the containerized renderer deployment in development, integration, eval, and production because the graphics stack must not depend on the host display session. EGL remains the desired default, but the current native EGL render probes segfault for reasons that are not yet isolated, so the current renderer image falls back to an OSMesa-backed VTK window class.
- Shared dependencies: `temporal`, `postgres`, `minio`.

The split is intentional: keep fast, high-throughput operations on light infra while isolating heavy physics/render workloads onto dedicated nodes.

## Local stack profiles

Local bootstrap uses profile-scoped host-port namespaces and compose project names so integration tests and eval runs can coexist on the same workstation.
As a convention, integration runs stay in the `10000-19999` band and eval runs stay in the `20000-29999` band, leaving the `0-10000` range for ordinary local services and reducing runner port clashes.

1. The `integration` profile keeps the existing local integration ports (`18000/18001/18002`, `15432`, `17233`, `19000/19001`, `15173`) and remains the default profile for the integration runner.
2. The `eval` profile uses a disjoint host-port namespace (`28000/28001/28002`, `25432`, `27233`, `29000/29001`) for the controller, workers, and infra so eval runs do not tear down or probe the integration stack.
3. Compose project names, worker-session directories, and service-log state are profile-scoped; the bootstrap layer uses one compose definition with profile-driven env vars instead of duplicated YAML.
4. The integration profile starts the frontend dev server on its normal local port; the eval profile leaves the frontend off because the eval runner only needs the backend services.

## Worker API

The worker API is physically split into three specialized services to optimize resource allocation and separate concerns: **Worker Light**, **Worker Heavy**, and **Worker Renderer**.

### Worker Light

- **Purpose**: Handles lightweight, synchronous operations and filesystem management.
- **Responsibilities**:
  - Filesystem CRUD operations (`/fs/*`).
  - Git repository management (`/git/*`).
  - Python code execution (short-lived) (`/runtime/execute`).
  - Asset serving (`/assets/*`).
  - Code linting (`/lint`).
  - Topology inspection (`/topology/inspect`).
- **WebSocket transport**: Frontend-to-controller and controller-to-worker-light control traffic on port 18001.
- **Operational profile**: High request-rate, short-lived operations, no heavy physics kernels.

### Worker Heavy

- **Purpose**: Handles compute-intensive, long-running tasks.
- **Responsibilities**:
  - Physics simulation (`/benchmark/simulate`).
  - Geometric validation (`/benchmark/validate`).
  - Design handover and DFM checks (`/benchmark/submit`).
  - Manufacturing analysis (`/benchmark/analyze`).
  - Render-request coordination for simulation jobs.
  - Asset building (`/benchmark/build`).
  - Single-flight execution gate: one active heavy job per worker instance.
- **HTTP boundary**: Direct benchmark endpoints are reserved for integration tests; production job dispatch stays behind Temporal.
- **Operational profile**: one active external heavy job at a time per instance, no in-process multi-request scheduler.

### Worker Renderer

- **Purpose**: Handles all headless rendering jobs.
- **Responsibilities**:
  - Explicit preview generation through build123d/VTK, including multi-view bundles and progressive status updates.
  - Simulation video generation and frame extraction.
  - Selection snapshots, depth images, segmentation images, and render-manifest persistence.
  - All render-only post-processing that does not require physics stepping.
- **HTTP boundary**: renderer endpoints are internal-only; public benchmark routes stay on the controller and worker-heavy paths.
- **Operational profile**: headless single-flight rendering; one active render job at a time per renderer instance, no internal render queue.

### Routing Contract (Controller -> Workers)

- Controller routes light operations to light worker over the WebSocket control channel. The light worker executes scripts, can stream queued/view-ready preview status back over that channel, and can ping the load balancer handling heavy workers.
- Controller routes heavy operations through Temporal workflows, not directly to `WORKER_HEAVY_URL`.
- Render jobs are dispatched to `worker-renderer`; the heavy worker does not own the graphics stack.
- All non-Temporal worker calls are session-scoped with `X-Session-ID`.
- The `WorkerClient` is the single boundary adapter; agents do not know about service split.
- Heavy-worker and renderer-worker admission are fail-closed: direct worker HTTP busy responses are allowed on the worker boundary, but controller and agent-facing product routes must not surface those raw `503 WORKER_BUSY` responses. Product routes either wait/retry through Temporal or fail closed at the orchestration layer if Temporal itself cannot complete.

### Heavy Execution Path Contract

Heavy compute execution has one production path:

- Controller tools call Temporal workflows for heavy operations.
- Temporal workflows dispatch heavy activities on `heavy-tasks-queue`.
- Heavy activity execution runs simulation/validation in isolated child process scope (crash containment boundary) and dispatches simulation render jobs to the renderer worker.

Backend responsibility is split by operation purpose:

1. `/benchmark/simulate` uses the selected physics backend and requests dynamic render/video artifacts from the renderer worker.
2. `/benchmark/validate` performs fast validation and does not generate preview artifacts.
3. Explicit preview requests use the renderer worker through the preview helper, normalize multi-view camera inputs, and remain separate from validation.
4. `/benchmark/validate` does not add a separate Genesis load/render gate solely for parity checking; Genesis-specific runtime behavior is established by actual Genesis simulation runs where Genesis behavior is required.
5. Both render-producing paths use the same renderer worker service; only the render job kind differs.

Backend choice is orthogonal to the controller and worker-plane split:

- API-backed runs execute through the controller's HTTP orchestration path.
- Codex-backed runs execute through the local Codex workspace path.
- Both paths still rely on the same worker services for filesystem, execution, validation, simulation, and the renderer worker.

Direct `worker-heavy` benchmark endpoints (`/benchmark/*`) are reserved for integration tests that verify worker-level boundaries, not an alternate orchestration model with independent queueing semantics.

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
  - simulation/validation coordination that dispatches render jobs.
  - any GPU work, if necessary.
- `worker_renderer/*` for:
  - static preview generation
  - simulation video rendering
  - selection snapshots and render-manifest generation
  - all headless rendering dependencies and post-processing

Validation is still part of `worker-heavy`, but the static preview part of validation is executed by `worker-renderer`. The point of the split is to keep validation on the build123d/VTK geometry path while isolating the graphics stack from simulation state.

Note: we want to offload work from `worker_heavy` as much as possible because:

- Network communication
- Boot time is slow, if need to up more workers it takes time.
- We don't want to load CPU-bound processes with lots of small requests
- If there are GPUs, they are more expensive.

### Worker filesystem and communication

Upon requesting simulation or rendering, worker light prepares the session bundle and git state needed for heavy execution. The Temporal orchestration path owns the actual heavy dispatch, render dispatch, retry, and completion tracking.

Predominantly, worker light communicates with Temporal orchestration for heavy dispatch through the WebSocket control plane; direct worker-heavy contact is reserved for integration tests that verify worker-level boundaries and for termination signals from the controller. There is a load balancer pinging `/ready` status in worker. Ideally, the worker-heavy and worker-renderer are hidden behind Temporal orchestration, which also acts as the admission and retry boundary.

### OpenAPI Artifacts

- `controller_openapi.json` documents the controller surface.
- Worker OpenAPI generation must include all worker surfaces (light, heavy, and renderer) or provide separate specs (`worker_light_openapi.json`, `worker_heavy_openapi.json`, `worker_renderer_openapi.json`) to avoid missing benchmark endpoints from generated artifacts.
- If a single `worker_openapi.json` is kept, it must be a merged schema, not just `worker_light` endpoints.

### Concurrency and Parallelism

Heavy-worker concurrency is a single-flight contract:

- A heavy worker instance can execute exactly one active job at a time.
- The heavy worker app does not own internal multi-job management (no in-process queue/semaphore/scheduler for multiple external jobs).
- If a request arrives while a job is active, the heavy worker returns a deterministic busy response (`503` + machine-readable reason such as `WORKER_BUSY`) instead of buffering jobs.
- That `503` is the direct heavy-worker admission/readiness signal, not the expected controller-path response.
- `/ready` and equivalent readiness signals must report `not ready` while a job is active, so load balancers route only to idle instances.
- Any queueing, retries, and fan-out for concurrent demand are owned outside `worker-heavy` (Temporal orchestration and infrastructure load balancing), not by heavy-worker app internals.
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

Renderer-worker concurrency is also a single-flight contract:

- A renderer worker instance can execute exactly one active render job at a time.
- The renderer worker app does not own internal multi-job management (no in-process queue/semaphore/scheduler for multiple external render jobs).
- If a render request arrives while a job is active, the renderer worker returns a deterministic busy response instead of buffering jobs.
- Any queueing, retries, and fan-out for concurrent render demand are owned outside `worker-renderer` (Temporal orchestration and infrastructure load balancing), not by renderer-worker internals.
- Within one admitted preview job, the renderer worker may fan out the requested views internally, but that still counts as one active render job at the service boundary.

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
