---
work_package_id: "WP01"
title: "Foundation & Skeleton"
lane: "planned"
dependencies: []
subtasks: ["T001", "T002", "T003", "T004", "T005", "T006"]
---

# WP01: Foundation & Skeleton

**Goal**: Initialize the project structure for Controller and Worker, ensuring isolated environments and basic CI/CD.

## Context

We are building a distributed system using `deepagents`.

- **Controller**: Manages agents (Railway deployment).
- **Worker**: Executes code (Podman container).

This WP sets up the rigid directory structure and build environment.

## Subtasks

### T001: Create project directory structure

Create the following folder structure:

```
src/
├── controller/
│   ├── api/
│   ├── graph/
│   └── __init__.py
├── worker/
│   ├── filesystem/
│   ├── runtime/
│   ├── utils/
│   ├── app.py           # (Empty for now)
│   └── __init__.py
└── deepagents/          # (Shared/Vendor code)
```

**Files**:

- Create all `__init__.py` files to make them packages.

### T002: Create Dockerfile for Controller

Location: `src/controller/Dockerfile`
Base: `python:3.10-slim`

**Requirements**:

- Install `uv` or `pip`.
- Copy `pyproject.toml` and lock file.
- Install dependencies (including `fastapi`, `uvicorn`, `langchain`, `deepagents`).
- Copy `src/controller`.
- Entrypoint: `uvicorn src.controller.api.main:app --host 0.0.0.0 --port 8000`

### T003: Create Dockerfile for Worker

Location: `src/worker/Dockerfile`
Base: `python:3.10-slim` OR a base image with `build123d` pre-compiled if needed (usually pip is fine).

**Requirements**:

- Install system deps for `build123d` (OCP) and `mujoco` (if needed for rendering, e.g. `libgl1-mesa-glx`).
- Install `build123d`, `mujoco`, `fastapi`, `uvicorn`, `s3fs`.
- Setup a non-root user `agent` for security.
- Entrypoint: `uvicorn src.worker.app:app --host 0.0.0.0 --port 8001`

### T004: Create docker-compose.yml

Location: `docker-compose.yml` (root)

**Services**:

- **controller**: Build context `src/controller`. Ports `8000:8000`. Env vars for DB/S3.
- **worker**: Build context `src/worker` (or root with file mapping?). Better to build from root so it can access shared code if any. *Correction*: Use root context and specify Dockerfile.
- **postgres**: Standard PG setup.
- **minio**: Standard MinIO setup (ports `9000:9000`, `9001:9001`). Create default bucket `problemologist`.

### T005: Setup Python Dependencies

Using `pyproject.toml` (or `uv init`):

- Define workspace or shared dependencies.
- Add `fastapi`, `uvicorn`, `build123d`, `mujoco`, `langchain`, `structlog`.
- Generate lock file.

### T006: Configure Linting

Create `ruff.toml` and `pyrightconfig.json` at root.

- **Rules**: Enforce strict typing (`reportMissingTypeStubs: false` initially if libs missing).
- Set up `pre-commit` config.

## Verification

1. `docker-compose build` should succeed for both images.
2. `docker-compose up -d` should start services.
3. `uv run ruff check .` should clean (or empty).
