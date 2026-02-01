---
work_package_id: WP02
title: Generator Agent Core
lane: "doing"
dependencies: [WP01]
base_branch: 005-benchmark-scenario-generator-WP01
base_commit: bd71bfd96bc9caf50b2527d6304c62ca85b98bde
created_at: '2026-02-01T13:20:20.227906+00:00'
subtasks:
- T004
- T005
- T006
shell_pid: "321215"
---

## Objective

Implement the "Scenario Architect" agent. This is a LangGraph state machine that takes a high-level prompt (e.g., "Make a lever") and produces a valid Python script.

## Context

The agent needs to know how to write `build123d` code. We will use a multi-step process: Plan -> Code -> Critique. The Critique step uses the Validator from WP01.

## Subtasks

### T004: Define System Prompts

**Purpose**: Instructions for the LLM.
**Steps**:
1. Create `src/generators/benchmark/prompts.py`.
2. `PLANNER_PROMPT`: "Break down the physics puzzle into parts..."
3. `CODER_PROMPT`: "You are an expert in build123d. Write a script that defines `build(seed)`..."
   - Include few-shot examples of `build123d` syntax if needed.
4. `CRITIC_PROMPT`: "Review the validation error and fix the code."

### T005: Implement Agent Graph

**Purpose**: The LangGraph orchestration.
**Steps**:
1. Create `src/generators/benchmark/agent.py`.
2. Define `GeneratorState` (TypedDict with `messages`, `code`, `errors`, `attempts`).
3. Implement Nodes:
   - `planner_node`: Calls LLM to generate a plan.
   - `coder_node`: Calls LLM to generate/fix Python code.
   - `validator_node`: Executes the generated code (using `exec` or subprocess), runs `validator.validate_mjcf`, updates state.
4. Define Edges:
   - `validator` -> `coder` (if failed & attempts < max).
   - `validator` -> `END` (if passed or max attempts reached).

### T006: Integrate LLM Client

**Purpose**: Connect to the project's LLM utility.
**Steps**:
1. Import `get_model` from `src.agent.utils.llm`.
2. Ensure the nodes in `agent.py` use this client to invoke the prompts.

## Acceptance Criteria

- [ ] Agent accepts a string prompt and returns a `GeneratorState`.
- [ ] If the generated code is invalid, the agent retries at least once (loop works).
- [ ] Successfully generates a script for a simple prompt ("Create a red box").
