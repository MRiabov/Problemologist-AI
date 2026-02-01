# Tasks: Benchmark Scenario Generator

**Feature**: 005-benchmark-scenario-generator
**Status**: Planned

## Work Packages

### WP01: Infrastructure & Validator

**Goal**: Establish the package structure and implement the automated physics validation logic.
**Priority**: Critical (Blocker)
**Dependencies**: None
**File**: [tasks/WP01-infrastructure.md](tasks/WP01-infrastructure.md)

- [x] T001 Scaffold Package Structure (`src/generators/benchmark/`)
- [x] T002 Define Data Models (`ValidationReport`, `ScenarioManifest`)
- [x] T003 Implement Headless Validator (`validator.py`)

### WP02: Generator Agent Core

**Goal**: Implement the "Scenario Architect" agent using LangGraph to author valid Python scripts.
**Priority**: High
**Dependencies**: WP01
**File**: [tasks/WP02-generator-agent.md](tasks/WP02-generator-agent.md)

- [x] T004 Define System Prompts (`prompts.py`)
- [x] T005 Implement Agent Graph (`agent.py`)
- [x] T006 Integrate LLM Client

### WP03: CLI & Randomization Pipeline

**Goal**: Build the user-facing CLI and the batch processing loop for randomization and export.
**Priority**: High
**Dependencies**: WP02
**File**: [tasks/WP03-cli-pipeline.md](tasks/WP03-cli-pipeline.md)

- [x] T007 Implement CLI Entry Point (`manager.py`)
- [x] T008 Implement Batch Processing & Randomization Loop
- [x] T009 Implement Artifact Export logic