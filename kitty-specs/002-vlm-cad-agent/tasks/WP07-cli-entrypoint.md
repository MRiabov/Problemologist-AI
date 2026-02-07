---
work_package_id: WP07
title: CLI Entrypoint
lane: "done"
dependencies: []
base_branch: 002-vlm-cad-agent-WP06
base_commit: 3518222e0cff6076161f46093c4870d73375c188
created_at: '2026-02-06T15:02:55.074432+00:00'
subtasks: [T028, T029, T030, T031]
shell_pid: "47012"
agent: "antigravity"
review_status: "has_feedback"
reviewed_by: "MRiabov"
---

## Objective

Implement the entrypoint for running the agent from the CLI. This includes configuration loading, Docker/Podman integration scripts, and a final smoke test.

## Context

The agent needs to be runnable via `python -m controller.agent.run`. It should load API keys for Spec 001 and DB connection strings from environment variables.

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

- `python -m controller.agent.run "Task"` works.
- Environment is correctly configured.

## Activity Log

- 2026-02-06T15:02:55Z – Gemini – shell_pid=678995 – lane=doing – Assigned agent via workflow command
- 2026-02-06T15:04:38Z – Gemini – shell_pid=678995 – lane=for_review – Ready for review: CLI entrypoint, config loading and run scripts implemented.
- 2026-02-06T17:46:26Z – antigravity – shell_pid=47012 – lane=doing – Started review via workflow command
- 2026-02-06T17:57:23Z – antigravity – shell_pid=47012 – lane=planned – Moved to planned
- 2026-02-06T18:00:35Z – antigravity – shell_pid=47012 – lane=done – Review passed: Fixed config to make openai_api_key optional and added openai_api_base for OpenRouter.
