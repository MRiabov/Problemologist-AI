---
work_package_id: WP03
title: Cognitive Engine Implementation
lane: planned
dependencies: []
subtasks:
- T009
- T010
- T011
- T012
- T013
---

## Objective

Implement the core `Engine` that drives the Agent's "Think, Plan, Act" loop. This includes the state machine for the three phases (Planning, Execution, Validation), the main execution loop, and comprehensive logging.

## Context

This is the heart of the agent. `src/agent/core/engine.py` will orchestrate the LLM, the Memory, and the Tool execution. It must follow the robust ReAct loop defined in the Plan.

## Subtasks

### T009: Implement Engine Skeleton & State

**Purpose**: Define the Engine class and its state representation.
**Steps**:

1. Create `src/agent/core/engine.py`.
2. Define `AgentState` (Pydantic model or dataclass) tracking:
   - Current Phase (0, 1, 2)
   - Step count
   - Current Plan
   - Last Error (if any)
3. Initialize `Engine` with `LLMClient`, `ContextManager`, `Memory`.

### T010: Implement Phase 0 (Planning)

**Purpose**: Only runs once at start. Reads doc/memory, generates plan.
**Steps**:

1. Implement `run_phase_0(problem_desc)` method in Engine.
2. Logic:
   - Construct Prompt: System + "Here is problem: {desc}. Search docs/memory and output plan."
   - Call LLM (may trigger `read_journal` or `search_docs` tools).
   - Final output should be the Plan.
   - Save Plan to State.

### T011: Implement Phase 1 (Execution Loop)

**Purpose**: The main ReAct loop.
**Steps**:

1. Implement `run_phase_1()` method.
2. Loop while `steps < MAX_STEPS`:
   - **Think**: Construct prompt (System + Plan + Context). Call LLM.
   - **Act**: Parse Tool Calls.
     - If `submit_design`: Transition to Phase 2.
     - If other tool: Execute it (via helper or External Env adapter).
   - **Observe**: Capture tool output (text or image). Add to Context.
   - Update `AgentState`.

### T012: Implement Phase 2 (Validation) & Submission

**Purpose**: Handle `submit_design` and feedback loop.
**Steps**:

1. Implement `run_phase_2(submission_args)`.
2. Logic:
   - Call external `submit_design`.
   - If Success: Write to Journal ("What went right"), End Session.
   - If Fail: Read error report, add to Context as observation.
   - **Self-Correction**: Revert to Phase 1 for Retry (increment retry counter).
   - Verify failure limits (e.g., max 3 retries).

### T013: Implement Logging & Rich UI

**Purpose**: Visibility into the agent's mind.
**Steps**:

1. Integrate `rich` library to print colored logs.
   - Blue for "Thought", Green for "Tool Call", Yellow for "Observation".
2. Implement JSONL logging to `agent_trace.jsonl`.
   - Log entry: `{step, phase, thought, tool_inputs, tool_outputs, timestamp}`.
   - Append to file after every step.

## Files to Create

- `src/agent/core/engine.py`

## Acceptance Criteria

- [ ] Engine can transition from Phase 0 to Phase 1.
- [ ] Execution loop correctly handles tool calls and updates context.
- [ ] `submit_design` triggers Phase 2 logic (success or retry).
- [ ] Trace logs are written to JSONL.
- [ ] Console output is formatted with Rich.
