---
work_package_id: WP03
title: Coder Agent (Generation Logic)
lane: "done"
dependencies: [WP01, WP02]
base_branch: 005-benchmark-scenario-generator-WP02
base_commit: 601ec0e687529bdabaab4c87f855377faf436b19
created_at: '2026-02-07T07:02:29.916799+00:00'
subtasks: [T009, T010, T011, T012]
shell_pid: "210592"
agent: "Gemini"
reviewed_by: "MRiabov"
review_status: "approved"
---

# WP03: Coder Agent (Generation Logic)

**Goal**: Implement the core "Coder" node that generates and refines `build123d` scripts.

## Subtasks

### T009: Coder Prompt Template

**Purpose**: Create the system prompt that instructs the LLM how to write the benchmark scripts.

**Instructions**:

1. Create `src/generators/benchmark/templates/coder_prompt.txt` (or .md).
2. **Key Constraints** to enforce in the prompt:
   - Must import `build123d` and `mujoco`.
   - Must define `def build(seed: int, scale: float = 1.0) -> Tuple[Compound, str]`: returns the build123d Object and the MJCF string.
   - Must use `build123d` for geometry.
   - Must adhere to the Plan provided by the Planner.
   - Must NOT uses hardcoded paths; use relative or generated logic.
   - Must handle exceptions gracefully.

### T010: Implement Coder Node

**Purpose**: The LangGraph node function for the Coder.

**Instructions**:

1. In `src/generators/benchmark/graph.py` (or `nodes.py`), implement `async def coder_node(state: BenchmarkGeneratorState)`.
2. Logic:
   - Retrieve `plan` and `review_feedback` from state.
   - Construct prompt using T009 template.
   - Call LLM (e.g., `ChatOpenAI` or `ChatAnthropic` as configured).
   - Parse output to extract the python code block.
   - Update `state["current_script"]`.
   - **Crucially**: Immediate syntax check (compile) before returning? Or leave it to validation step? -> Better to try `compile()` locally (exec into safe scope) to catch syntax errors immediately.

### T011: Feedback Loop & Validation Integration

**Purpose**: Integrate the validation tools from WP02 into the loop.

**Instructions**:

1. Add logic to `coder_node` (or a separate `validation_node` if preferred, but WP structure suggests Coder handles refinement).
2. Actually, better pattern: `Coder -> ValidatorNode`.
   - Let's keep `coder_node` focused on *writing*.
   - Create a helper or tool wrapper that calls `simulate_stability` (from WP02).
3. Update `coder_node` to handle `state["simulation_result"]`:
   - If `simulation_result.success` is False, append the error log to the next prompt context so the Coder can fix it.

### T012: Verification of Generation

**Purpose**: Ensure the Coder actually writes runnable code.

**Instructions**:

1. Create a test `tests/generators/test_coder.py`.
2. Mock the LLM to return a known valid script.
3. Run `coder_node`.
4. Run the generated script's `build()` function.
5. Verify it produces a Compound and MJCF string.

## Verification

- [ ] Coder node correctly formatting prompts with plan/feedback.
- [ ] Generated code follows the `build(seed, scale)` signature.
- [ ] Loop allows for retries upon validation failure (mocked failure).

## Activity Log

- 2026-02-07T07:02:30Z – Gemini – shell_pid=188309 – lane=doing – Assigned agent via workflow command
- 2026-02-07T07:09:03Z – Gemini – shell_pid=188309 – lane=for_review – Ready for review: Implemented Coder node with prompt template, extract logic, and syntax verification. Also implemented Validator node which runs physics and geometric checks. Added comprehensive tests.
- 2026-02-07T07:16:18Z – Gemini – shell_pid=210592 – lane=doing – Started review via workflow command
- 2026-02-07T07:17:38Z – Gemini – shell_pid=210592 – lane=done – Review passed: Coder and Validator nodes implemented with robust prompt templates, syntax verification, and feedback loop integration. All unit tests passed.
