---
work_package_id: "WP03"
title: "Manager & CLI"
lane: "planned"
dependencies: ["WP02"]
subtasks: ["T012", "T013", "T014", "T015", "T016"]
---

# Work Package 3: Manager & CLI

## Objective
Create the user-facing command-line tool to drive the generation process in batch mode and manage the dataset lifecycle (Staging -> Production).

## Context
Developers need a way to say "Make me 10 levers" without writing Python code manually. This CLI wraps the Agent (WP02) and handles the "industrial scale" production of scenarios.

## Subtasks

### T012: Implement Manager Entry Point
**Purpose**: CLI scaffolding.
**File**: `src/generators/benchmark/manager.py`
**Requirements**:
- Use `argparse` or `typer` (check project conventions, `argparse` is safe default).
- Setup `main` function and entry block.
- Commands: `generate`, `list`, `promote`.

### T013: Implement Generate Command
**Purpose**: The main loop.
**Logic**:
- `generate(prompt: str, count: int, tier: str)`
- Loop `count` times:
  - Generate a new random seed.
  - Invoke `GeneratorAgent` (WP02) with `prompt` + `seed`.
  - Save results to `datasets/staging/{run_id}_{seed}/`.
  - Print progress: "Generating 1/10... [Success/Fail]".

### T014: Implement Promote Command
**Purpose**: Human approval workflow.
**Logic**:
- `promote(run_id: str, dest: str)`
- Moves the folder from `datasets/staging/` to `datasets/benchmarks/{dest}/`.
- Updates/creates a `manifest.json` in the destination to index the new scenario.

### T015: Tier Presets
**Purpose**: Easy defaults.
**File**: `src/generators/benchmark/presets.py` (optional) or inside `manager.py`.
**Logic**:
- Define default prompts for "Tier 1" (Spatial/Peg-in-hole) and "Tier 2" (Kinematic/Lever).
- If user passes `--tier 1`, use the preset prompt instead of requiring raw text.

### T016: E2E Test
**Purpose**: Verify the whole pipeline.
**File**: `tests/generators/benchmark/test_manager.py`
**Tests**:
- Run `manager.py generate --count 1 --tier test_dummy` (Mock the agent to avoid LLM costs/latency).
- Assert file structure in `staging`.
- Run `manager.py promote` on that ID.
- Assert file structure in `benchmarks`.

## Definition of Done
- `python -m src.generators.benchmark.manager --help` works.
- Can generate a batch of scenarios.
- Can promote a scenario.
