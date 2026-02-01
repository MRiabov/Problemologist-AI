---
work_package_id: WP04
title: Runner & Integration
lane: planned
dependencies: []
subtasks:
- T014
- T015
- T016
---

## Objective

Create the CLI entry point (`runner.py`) and wire the Agent up to the actual `001-agentic-cad-environment` (or a mock for now if environment isn't fully separate package).

## Context

The agent needs to be runnable from the command line, accepting a problem description and executing the full loop. We also need to ensure the "External Tools" (`write_script`, etc.) actually invoke the Environment.

## Subtasks

### T014: Implement Environment Adapter

**Purpose**: Bridge the Agent's tool calls to the Environment.
**Steps**:

1. If Spec 001 code is importable, import the tool functions.
2. If Spec 001 is a separate process/server, implement the client wrapper.
   - *Assumption*: Based on plan, it's "Loose Coupling".
   - Implement `src/agent/environment_adapter.py`.
   - Map `WriteScript` -> calls environment's file writer.
   - Map `PreviewDesign` -> calls environment's renderer.
   - Map `SearchDocs` -> calls environment's retrieval system.
   - *For now, if Env 001 isn't ready, implement Mock versions that return plausible responses.*

### T015: Implement CLI Runner

**Purpose**: The main executable script.
**Steps**:

1. Create `src/agent/runner.py`.
2. Use `argparse` or `click` to accept:
   - `--problem "Make a cube"`
   - `--model "gpt-4o"`
   - `--max-steps 20`
3. Logic:
   - Initialize `LLMClient` with config.
   - Initialize `Memory` and `Context`.
   - Initialize `Engine`.
   - Call `engine.run(problem)`.

### T016: Verify End-to-End Flow & Smoke Test

**Purpose**: Ensure the whole system hangs together.
**Steps**:

1. Create `tests/e2e/test_agent_run.py`.
2. Create a test that runs `runner.py` main function with a mocked LLM (that outputs a predictable sequence of tool calls).
3. Verify the agent completes the loop and exits successfully.

## Files to Create

- `src/agent/environment_adapter.py`
- `src/agent/runner.py`
- `tests/e2e/test_agent_run.py`

## Acceptance Criteria

- [ ] `python src/agent/runner.py --problem "test"` runs without crashing.
- [ ] Agent correctly invokes environment tools (mocked or real).
- [ ] Full "Thought -> Act -> Observe" cycle works in the runner.
