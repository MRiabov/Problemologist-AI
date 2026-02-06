# Research: Agentic CAD Environment

## Decisions

### 1. Geometry Pipeline (CAD -> Physics)

**Decision**: Use `build123d` -> STL -> `trimesh` -> `mujoco` (MJCF).
**Rationale**:

- `build123d` exports high-quality STLs.
- `trimesh` handles inertia and convex hull generation for stable MuJoCo collisions.
- MJCF is generated programmatically from the geometry data.

### 2. Framework & Security

**Decision**: Use **`deepagents`** with `FilesystemMiddleware` and `SandboxFilesystemBackend`.
**Rationale**:

- Provides native support for distributed workers and sandboxed execution.
- `FilesystemMiddleware` allows agents to work directly in a familiar file-based environment while maintaining safety.
- Podman-based worker nodes provide robust isolation for untrusted code execution.

### 3. Documentation & Learning

**Decision**: Use **`SKILL.md`** files and a dedicated **Learner Agent**.
**Rationale**:

- Agents acquire skills (e.g., `build123d` best practices) which are persisted as `SKILL.md` artifacts.
- A sidecar Learner Agent asynchronously updates skills based on successful reasoning traces in the `journal.md`.
- Documentation is accessible through `deepagents` memory and the filesystem.

### 4. Orchestration

**Decision**: Use **Temporal** for worker orchestration.
**Rationale**:

- Simplifies long-running tasks like simulation and renders.
- Handles retries and state persistence across worker failures.
