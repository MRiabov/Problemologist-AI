---
work_package_id: WP07
title: CLI Entrypoint
lane: planned
dependencies: []
subtasks: [T028, T029, T030, T031]
---

## Objective

Implement the entrypoint for running the agent from the CLI. This includes configuration loading, Docker/Podman integration scripts, and a final smoke test.

## Context

The agent needs to be runnable via `python -m src.agent.run`. It should load API keys for Spec 001 and DB connection strings from environment variables.

## Subtasks

### T028: Implement `src/agent/run.py`

Create the main entrypoint.

- **File**: `src/agent/run.py`
- **Functionality**:
  - `def main()`:
    - Load config.
    - Initialize Graph (from `src/agent/graph.py`).
    - Parse CLI args (user request).
    - `graph.stream(inputs, config=...)`.
    - Print outputs.

### T029: Implement configuration loading

- **File**: `src/agent/config.py` (or similar).
- **Requirements**:
  - Load `OPENAI_API_KEY` / `ANTHROPIC_API_KEY`.
  - Load `SPEC_001_API_URL` (Worker URL).
  - Load `DB_CONNECTION_STRING`.
  - Validate required vars are present.

### T030: Add Docker/Podman run scripts

- **File**: `scripts/run_agent.sh` (or `Makefile` entry).
- **Requirements**:
  - Script to build and run the agent container.
  - Mount `kitty-specs/` if needed (for skills).
  - Pass env vars.

### T031: Final End-to-End smoke test

- **Action**:
  - Spin up the Agent (WP01-WP06 complete).
  - Spin up a mocked Worker (or real one if available).
  - Run a simple request: "Create a 10x10x10 cube".
  - Verify:
    - Plan created?
    - Script written?
    - Simulation run (mocked)?
    - Final answer returned?

## Definition of Done

- `python -m src.agent.run "Task"` works.
- Environment is correctly configured.
