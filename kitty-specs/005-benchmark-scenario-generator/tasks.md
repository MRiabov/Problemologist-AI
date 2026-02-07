# Tasks: Benchmark Scenario Generator (Spec 005)

**Feature Directory**: `/home/maksym/Work/proj/Problemologist/Problemologist-AI/kitty-specs/005-benchmark-scenario-generator`

## Work Packages

### WP01: Foundation & Data Models

**Goal**: Set up the data structures and foundational state management.

- [x] T001: Implement Pydantic models `BenchmarkAsset` and `GenerationSession` in `src/generators/benchmark/models.py`.
- [x] T002: Create database schema/migrations for these models.
- [x] T003: Define `BenchmarkGeneratorState` for the `deepagents` graph.
- [x] T004: Scaffold directory structure and standard agent template files (`journal.md`, `todo.md`, etc.).

~250 lines

### WP02: Validation & Worker Utilities

**Goal**: Implement the physics validation logic that runs on the worker.

- [x] T005: Implement `simulate(Compound) -> SimulationResult` in `src/worker/utils/validation.py`.
- [x] T006: Implement `validate(Compound) -> bool` for geometric/randomization verification.
- [x] T007: Implement `submit_for_review(Compound)` to handover results to Reviewer.
- [x] T008: Expose these tools to the Agent environment via `deepagents` tools.

~300 lines

### WP03: Coder Agent (Generation Logic)

**Goal**: Implement the core agent that writes the `build123d` scripts.

- [x] T009: Create Coder prompt template with `build(seed, scale)` requirements.
- [x] T010: Implement Coder node in `graph.py` with access to validation tools.
- [x] T011: Implement feedback loop: Parse `ValidationResult` and re-prompt Coder on failure.
- [x] T012: Verify Coder can generate a simple valid script.

~350 lines

### WP04: Planner & Reviewer Agents

**Goal**: Implement the guiding Planner and the quality-assuring Reviewer.

- [x] T013: Implement Planner node (Theme -> Randomization Strategy).
- [x] T014: Implement Reviewer node (Image analysis -> Approve/Reject).
- [x] T015: Connect nodes in `graph.py` (Planner -> Coder -> Reviewer).
- [x] T016: Define graph entry point.

~350 lines

### WP05: Persistence & Integration

**Goal**: Persist results and provide a CLI for running the generator.

- [x] T017: Implement `save_asset` logic (Save images to `/renders/` for S3 routing, DB insert for metadata).
- [x] T018: Implement `save_session_state` checkpointing.
- [x] T019: Create CLI entry point/script to trigger generation sessions.
- [x] T020: End-to-End integration test.

~300 lines
