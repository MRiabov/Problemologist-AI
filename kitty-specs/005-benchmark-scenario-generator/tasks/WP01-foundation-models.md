---
work_package_id: WP01
title: Foundation & Data Models
lane: "doing"
dependencies: []
base_branch: main
base_commit: 08d44c45aaa09786e450e9a322e7f98d2b2f3498
created_at: '2026-02-06T14:32:20.498478+00:00'
subtasks: [T001, T002, T003, T004]
shell_pid: "649118"
agent: "Gemini"
---

# WP01: Foundation & Data Models

**Goal**: Establish the core data structures and project scaffolding for the Benchmark Scenario Generator.

## Subtasks

### T001: Implement Pydantic Models

**Purpose**: Define the data exchange format for benchmarks and session state.
**Reference**: `data-model.md`.

**Instructions**:

1. Create `src/generators/benchmark/models.py`.
2. Implement `BenchmarkAsset` Pydantic model:
   - `benchmark_id`: UUID
   - `mjcf_url`, `build123d_url`, `preview_bundle_url`: HttpUrl
   - `random_variants`: List[UUID]
   - `difficulty_score`: float
   - `metadata`: Dict[str, Any] (flexible for Tier, Cost Targets)
3. Implement `GenerationSession` Pydantic model:
   - `session_id`: UUID
   - `prompt`: str
   - `status`: Enum (planning, executing, validating, accepted, rejected)
   - `validation_logs`: List[str]

### T002: Database Schema & Migrations

**Purpose**: Persist the models in Postgres.

**Instructions**:

1. If using SQLAlchemy, define the ORM mapping for `BenchmarkAsset` and `GenerationSession` in `src/generators/benchmark/schema.py`.
2. Generate an alembic migration script to create these tables.
3. Ensure proper indices on `benchmark_id` and `session_id`.

### T003: Define Graph State

**Purpose**: Define the state object passed between agents in the `deepagents` (LangGraph) graph.

**Instructions**:

1. Create `src/generators/benchmark/state.py`.
2. Define `BenchmarkGeneratorState` (TypedDict or Pydantic model, check `deepagents` conventions):
   - `session`: GenerationSession
   - `current_script`: str (The Python code being generated)
   - `simulation_result`: Optional[ValidationResult] (Result of the last check)
   - `review_feedback`: Optional[str] (Comments from reviewer)
   - `plan`: Optional[Dict] (The randomization strategy)
   - `messages`: List[BaseMessage] (Chat history)

### T004: Scaffold Directory & Templates

**Purpose**: Create the necessary folder structure and initial agent files.

**Instructions**:

1. Ensure the following exist:
   - `src/generators/benchmark/`
   - `src/generators/benchmark/templates/`
   - `src/generators/benchmark/utils/`
   - `src/worker/agent_files/`
2. Create baseline Markdown templates in `src/worker/agent_files/`:
   - `journal.md`: Minimal structured header.
   - `todo.md`: Placeholder for Planner's output.
   - `script.py`: Template with `build(seed, scale)` boilerplate.

## Verification

- [ ] Models import correctly and validate sample data.
- [ ] Migration script is generated and looks correct (no need to run against prod DB, but verify SQL).
- [ ] `BenchmarkGeneratorState` covers all necessary fields for the pipeline.

## Activity Log

- 2026-02-06T14:32:20Z – Gemini – shell_pid=649118 – lane=doing – Assigned agent via workflow command
