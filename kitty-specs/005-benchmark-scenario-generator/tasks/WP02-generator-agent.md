---
work_package_id: "WP02"
title: "Generator Agent (Core Logic)"
lane: "planned"
dependencies: ["WP01"]
subtasks: ["T006", "T007", "T008", "T009", "T010", "T011"]
---

# Work Package 2: Generator Agent

## Objective
Implement the "brain" of the system: a LangGraph-based agent that iteratively writes, executes, and fixes the scenario generation scripts.

## Context
We are using `langgraph` (or a compatible state machine approach) to orchestrate an LLM. The agent's goal is to output a *valid* Python script that, when run, produces the STLs and MJCFs defined in the Spec.

## Subtasks

### T006: Define AgentState
**Purpose**: The memory of the agent.
**File**: `src/generators/benchmark/agent.py`
**Requirements**:
- TypedDict `AgentState`:
  - `user_request`: str (e.g., "Tier 1 Peg-in-hole")
  - `plan`: str (The geometric breakdown)
  - `current_code`: str (The Python script)
  - `artifacts_path`: str (Where files were saved)
  - `validation_error`: Optional[str]
  - `attempts`: int

### T007: Implement Prompts
**Purpose**: System instructions for the LLM.
**File**: `src/generators/benchmark/prompts.py`
**Content**:
- `PLANNER_PROMPT`: "You are a mechanical engineer. Break down this request into build123d geometric primitives and MuJoCo joints..."
- `CODER_PROMPT`: "You are a Python expert. Write a script using `build123d` and `asset_manager`..."
  - **Crucial**: Include a few-shot example of a script that defines a `build(seed)` function and exports assets.
- `REVIEWER_PROMPT`: "Analyze this validation error. Fix the code to resolve the collision/syntax error."

### T008: Implement Coding Node
**Purpose**: The "Doer".
**File**: `src/generators/benchmark/agent.py` (add methods)
**Steps**:
1. `generate_code(state)`: Calls LLM with `CODER_PROMPT`.
2. `execute_code(state)`:
   - Writes `state['current_code']` to a temp file.
   - Executes it via `subprocess.run(["python", temp_file])`.
   - **Constraint**: The generated code *must* use the `AssetManager` (from WP01) to save files to a known location.
   - Captures stdout/stderr. If execution fails, return error.

### T009: Implement Reviewing Node
**Purpose**: The "Critic".
**File**: `src/generators/benchmark/agent.py`
**Steps**:
1. `validate_output(state)`:
   - Instantiates `Validator` (WP01).
   - Checks the MJCF at `state['artifacts_path']`.
   - If valid -> returns "END".
   - If invalid -> returns "retry" and updates `state['validation_error']`.

### T010: Assemble the Graph
**Purpose**: Connect the nodes.
**File**: `src/generators/benchmark/agent.py`
**Logic**:
- Start -> Planner -> Coder -> Executor (Internal helper) -> Validator
- Validator --(pass)--> End
- Validator --(fail)--> Coder (with error context)
- Add a `max_retries` check (e.g., 3) to prevent infinite loops.

### T011: Integration Test
**Purpose**: Prove it works.
**File**: `tests/generators/benchmark/test_agent.py`
**Tests**:
- Mock the LLM (using `unittest.mock` or a fake LLM class) to return a pre-canned valid script.
- Verify the graph transitions from Start to End.
- Verify the artifacts are actually created on disk.

## Definition of Done
- `GeneratorAgent` class exists and can be invoked.
- It successfully loops back if the code fails validation.
- It produces a `ScenarioManifest` upon success.
