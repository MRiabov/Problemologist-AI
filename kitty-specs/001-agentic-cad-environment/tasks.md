# Tasks: Agentic CAD Environment

**Spec**: [001-agentic-cad-environment](spec.md)
**Status**: Planned

## Work Package 1: Foundation & Skeleton

**Goal**: Initialize the project structure for Controller and Worker, ensuring isolated environments and basic CI/CD.
**Priority**: Critical (Phase 1)
**Tests**: `docker-compose up` brings up both services healthy. `make test` runs basic lints.

- [x] T001: Create project directory structure for Controller (`src/controller`) and Worker (`src/worker`).
- [x] T002: Create `Dockerfile` for Controller (Python 3.10+, Railway-ready).
- [x] T003: Create `Dockerfile` for Worker (Python 3.10+, Podman-compatible, with `build123d` and `mujoco` deps).
- [x] T004: Create `docker-compose.yml` for local development setup (Controller, Worker, Postgres, MinIO).
- [x] T005: Setup `pyproject.toml` / `uv` workspace for dependency management.
- [x] T006: Configure basic strict linting (`ruff`, `pyright`) and pre-commit hooks.

**Implementation**:

1. Scaffold directories.
2. write Dockerfiles.
3. Setup `docker-compose`.
4. Configure Python deps.

**Dependencies**: None
**Risks**: Docker networking issues between Podman and main containers.

## Work Package 2: Worker Core (FS & Runtime)

**Goal**: Implement the sandboxed Hybrid filesystem (Local Sandbox + S3 Routing) and secure Python runtime execution.
**Priority**: Critical (Phase 1)
**Tests**: Unit tests for Local FS and Composite routing. Integration test running a simple Python script.

- [ ] T007: Implement `LocalFilesystemBackend` for high-frequency operations.
- [ ] T008: Implement `CompositeBackend` to route `/renders/` to MinIO/S3 and everything else to local.
- [ ] T009: Implement `runtime` module to execute arbitrary Python code in a subprocess.
- [ ] T010: Ensure the local filesystem is reset from Git on every session (start workflow).
- [ ] T011: Verify `build123d` and `mujoco` imports work in the runtime environment.

**Implementation**:

1. Local FS setup.
2. Composite backend logic.
3. Subprocess wrapper.
4. Git reset hook.

**Dependencies**: WP01
**Risks**: Performance of Composite routing; ensuring Git reset doesn't wipe required persistence.

## Work Package 3: Worker API & Client

**Goal**: Expose Worker capabilities via strict OpenAPI and generate a client for the Controller.
**Priority**: High (Phase 1)
**Tests**: `schemathesis` runs against Worker API. Controller can call Worker `ls` via client.

- [ ] T012: Setup FastAPI setup in `src/worker/app.py`.
- [ ] T013: Define Pydantic models for Tool Inputs/Outputs (`WriteFileRequest`, `ExecuteResponse`, etc.).
- [ ] T014: Implement endpoints: `/fs/ls`, `/fs/read`, `/fs/write`, `/fs/edit`.
- [ ] T015: Implement endpoint: `/runtime/execute`.
- [ ] T016: Implement endpoint: `/runtime/reset` (triggers Git reset of sandbox).

**Implementation**:

1. FastAPI routes.
2. Pydantic schema.
3. Client generation.

**Dependencies**: WP02
**Risks**: Schema drift (mitigated by strict generation).

## Work Package 4: Controller & DeepAgents Integration

**Goal**: Integrate the Worker Client into the Controller's `deepagents` middleware.
**Priority**: High (Phase 2)
**Tests**: Controller unit tests mocking the Worker Client.

- [ ] T017: Setup Controller application (FastAPI + LangGraph skeleton).
- [ ] T018: internalize generated Worker functionality.
- [ ] T019: Implement `RemoteFilesystem` middleware in Controller that calls Worker Client.
- [ ] T020: Define LangChain Tools (`ls`, `write_file`, `execute`) wrapping the middleware.
- [ ] T021: Setup basic Agent graph (skeleton) to verify tool availability.

**Implementation**:

1. Controller app setup.
2. Middleware integration.
3. Tool definition.

**Dependencies**: WP03
**Risks**: Latency in tool calls.

## Work Package 5: Domain Utils (Worker Side)

**Goal**: Implement the "OS-level" utilities that agents will import and usage.
**Priority**: Medium (Phase 2)
**Tests**: Unit tests for `utils` package.

- [ ] T022: Create `src/worker/utils/` package.
- [ ] T023: Implement `validate_and_price(component)` stub (mock workbench).
- [ ] T024: Implement `simulate(component)` stub (mock simulation loop: git commit -> upload -> pass).
- [ ] T025: Implement `submit_for_review(component)` stub.
- [ ] T026: Ensure these utils are importable by the `runtime` execution (PYTHONPATH).

**Implementation**:

1. Python package creation.
2. Logic implementation.
3. Path configuration.

**Dependencies**: WP02
**Risks**: `build123d` complexity in validation.

## Work Package 6: Temporal Orchestration

**Goal**: Move long-running tasks (Simulation) to Temporal workflows.
**Priority**: Medium (Phase 2)
**Tests**: Run a "long" task and restart Worker; verify task completes.

- [ ] T027: Add Temporal service to `docker-compose`.
- [ ] T028: Implement Temporal Worker in `src/controller` (or separate service?). A: Controller initiates.
- [ ] T029: Define `SimulationWorkflow` and `SimulationActivity`.
- [ ] T030: Update `simulate` util (or API) to trigger Temporal workflow instead of sync execution.
- [ ] T031: Implement callback/webhook logic to notify Agent when simulation is done (if async) or wait (if sync). Note: Spec says Temporal handles it.

**Implementation**:

1. Temporal setup.
2. Workflow definition.
3. Integration.

**Dependencies**: WP05, WP04
**Risks**: Complexity of async agent flow.

## Work Package 7: Observability & Persistence

**Goal**: set up structured logging, Postgres persistence, and LangFuse.
**Priority**: Medium (Phase 3)
**Tests**: Verify logs appear in structlog format. Verify traces in LangFuse.

- [ ] T032: Configure `structlog` for JSON logging on both nodes.
- [ ] T033: Setup Postgres DB with SQLAlchemy and Alembic.
- [ ] T034: Define schema for `Episodes`, `Traces`, `Assets`.
- [ ] T035: Integrate `langfuse` in Controller Agent.

**Implementation**:

1. Logging config.
2. DB setup.
3. LangFuse integration.

**Dependencies**: WP04
**Risks**: Database connection management.
