# Tasks: Benchmark Scenario Generator

**Feature**: 005-benchmark-scenario-generator
**Status**: Planned

## Work Package 1: Foundation & Validator
**Goal**: Establish the project structure and build the "Judge" (Validator) that ensures generated scenarios are valid.
**Priority**: Critical (Blocker for WP02)

- [ ] **T001**: Scaffold `src/generators/benchmark` package structure.
- [ ] **T002**: Implement `AssetManager` for file paths and STL/MJCF export logic.
- [ ] **T003**: Implement `Validator` class (Mesh checks + MuJoCo stability simulation).
- [ ] **T004**: Create "Golden" and "Broken" test fixtures to verify Validator.
- [ ] **T005**: Write unit tests for Validator and AssetManager.

**Implementation Sketch**:
Create the directory structure. Build a utility to standardize where STLs and XMLs are saved (`scenarios/staging/{run_id}/`). Implement the `Validator` using `mujoco` python bindings to load the XML and step it for 1 second. If `data.qvel` > 100, fail.

**Dependencies**: None.
**Prompt Estimate**: ~300 lines.

## Work Package 2: Generator Agent (Core Logic)
**Goal**: Build the LangGraph agent that can author, compile, and refine a scenario script.
**Priority**: High

- [ ] **T006**: Define `AgentState` schema for LangGraph.
- [ ] **T007**: Create `prompts.py` with system instructions (Planner, Coder, Reviewer).
- [ ] **T008**: Implement `Coding` node (LLM -> Code -> Exec -> Artifacts).
- [ ] **T009**: Implement `Reviewing` node (Validator -> Feedback Loop).
- [ ] **T010**: Assemble the Graph (Entry -> Plan -> Code -> Review -> End).
- [ ] **T011**: Integration test: Generate a simple "Box on Plane" scenario.

**Implementation Sketch**:
Use `langgraph` to define the flow. The "Coding" node is complex: it must execute the generated Python code in a safe-ish way (subprocess) to produce the STLs and XMLs defined in WP01. The "Reviewer" reads the `Validator` report and either passes or adds error context to the state for a retry.

**Dependencies**: WP01.
**Prompt Estimate**: ~450 lines.

## Work Package 3: Manager & CLI
**Goal**: Provide the user interface to generate batches and manage the dataset.
**Priority**: Medium

- [ ] **T012**: Implement `manager.py` CLI entry point.
- [ ] **T013**: Implement `generate` command (Batch loop + Random seeds).
- [ ] **T014**: Implement `promote` command (Staging -> Benchmarks).
- [ ] **T015**: Add specific support for "Tier 1" and "Tier 2" presets.
- [ ] **T016**: E2E Test: Generate 5 scenarios, promote 2.

**Implementation Sketch**:
A simple `argparse` or `typer` CLI. The `generate` command wraps the Agent from WP02 in a loop. It handles the "Seed" injection to ensure we get N variations. `promote` simply moves files and updates a `manifest.json`.

**Dependencies**: WP02.
**Prompt Estimate**: ~300 lines.
