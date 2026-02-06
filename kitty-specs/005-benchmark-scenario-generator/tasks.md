# Tasks: Benchmark Scenario Generator (Spec 005)

**Feature Directory**: `/home/maksym/Work/proj/Problemologist/Problemologist-AI/kitty-specs/005-benchmark-scenario-generator`

## Work Packages

### WP01: Foundation & Data Models

**Goal**: Set up the data structures and foundational state management.

- [ ] **T001**: Implement Pydantic models `BenchmarkAsset` and `GenerationSession` in `src/generators/benchmark/models.py`.
- [ ] **T002**: Create database schema/migrations for these models.
- [ ] **T003**: Define `BenchmarkGeneratorState` for the `deepagents` graph.
- [ ] **T004**: Scaffold directory structure `src/generators/benchmark`.

~250 lines

### WP02: Validation & Worker Utilities

**Goal**: Implement the physics validation logic that runs on the worker.

- [ ] **T005**: Implement `simulate_stability(mjcf_content) -> ValidationResult` in `src/worker/utils/validation.py`.
- [ ] **T006**: Implement headless rendering utility to produce 24-view preview bundle.
- [ ] **T007**: Unit test validation logic with dummy valid/invalid MJCFs.
- [ ] **T008**: Expose validation tools to the Agent environment.

~300 lines

### WP03: Coder Agent (Generation Logic)

**Goal**: Implement the core agent that writes the `build123d` scripts.

- [ ] **T009**: Create Coder prompt template with `build(seed, scale)` requirements.
- [ ] **T010**: Implement Coder node in `graph.py` with access to validation tools.
- [ ] **T011**: Implement feedback loop: Parse `ValidationResult` and re-prompt Coder on failure.
- [ ] **T012**: Verify Coder can generate a simple valid script.

~350 lines

### WP04: Planner & Reviewer Agents

**Goal**: Implement the guiding Planner and the quality-assuring Reviewer.

- [ ] **T013**: Implement Planner node (Theme -> Randomization Strategy).
- [ ] **T014**: Implement Reviewer node (Image analysis -> Approve/Reject).
- [ ] **T015**: Connect nodes in `graph.py` (Planner -> Coder -> Reviewer).
- [ ] **T016**: Define graph entry point.

~350 lines

### WP05: Persistence & Integration

**Goal**: Persist results and provide a CLI for running the generator.

- [ ] **T017**: Implement `save_asset` logic (S3 upload for assets, DB insert for metadata).
- [ ] **T018**: Implement `save_session_state` checkpointing.
- [ ] **T019**: Create CLI entry point/script to trigger generation sessions.
- [ ] **T020**: End-to-End integration test.

~300 lines
