---
work_package_id: WP04
title: Planner & Reviewer Agents
lane: "for_review"
dependencies: [WP03]
base_branch: 005-benchmark-scenario-generator-WP03
base_commit: 2962558c1cdd376ffb4b30162397bc520bec9635
created_at: '2026-02-07T07:18:13.777909+00:00'
subtasks: [T013, T014, T015, T016]
shell_pid: "212824"
agent: "Gemini"
---

# WP04: Planner & Reviewer Agents

**Goal**: Implement the high-level planning and final review stages, and wire the full graph.

## Subtasks

### T013: Implement Planner Node

**Purpose**: Break down the user's high-level request (e.g., "A lever pushing a box") into a concrete randomization strategy.

**Instructions**:

1. Create prompt template `src/generators/benchmark/templates/planner_prompt.txt`.
   - Output JSON schema: `{ "theme": str, "objects": [...], "constraints": [...], "randomization_ranges": {...} }`.
2. Implement `async def planner_node(state: State)`.
   - Input: `state["state"]` (User prompt).
   - Output: Update `state["plan"]`.

### T014: Implement Reviewer Node

**Purpose**: Visual inspection of the generated benchmark.

**Instructions**:

1. Create prompt template for Reviewer (Vision model).
   - Input: Series of images (from WP02 `render_views`).
   - Criteria: "Does this look like the requested theme?", "Is it too cluttered?", "Are pieces floating?".
2. Implement `async def reviewer_node(state: State)`.
   - Call `render_mjcf` tool (WP02).
   - Pass images to V-LLM (e.g. GPT-4o).
   - Output: `Approve` or `Reject` with feedback.
   - Update `state["review_feedback"]`.

### T015: Graph Construction

**Purpose**: Define the `StateGraph` edges and transitions.

**Instructions**:

1. In `src/generators/benchmark/graph.py`:
   - Initialize `StateGraph(BenchmarkGeneratorState)`.
   - Add nodes: `planner`, `coder`, `validator` (calls `simulate_stability`), `reviewer`.
   - Define Edges:
     - `planner` -> `coder`
     - `coder` -> `validator`
     - `validator` -> `coder` (if fail) OR `reviewer` (if pass)
     - `reviewer` -> `END` (if approve) OR `planner`/`coder` (if reject - decide based on nature of rejection).
       - *Decision*: If visual issue, back to Coder. If conceptual, back to Planner. For simplicity, start with back to Coder with feedback.

### T016: Entry Point

**Purpose**: A function to run the graph.

**Instructions**:

1. Implement `async def run_generation_session(prompt: str) -> GenerationSession`.
   - Initialize state.
   - Run graph.compile().invoke().
   - Return final session result.

## Verification

- [ ] Graph compiles without error.
- [ ] Planner produces structured JSON plan.
- [ ] Reviewer node correctly processes images (mocked inputs).
- [ ] State transitions logic covers all paths (Success, Validation Fail, Review Fail).

## Activity Log

- 2026-02-07T07:18:13Z – Gemini – shell_pid=212824 – lane=doing – Assigned agent via workflow command
- 2026-02-07T08:32:30Z – Gemini – shell_pid=212824 – lane=for_review – Rebased on main and resolved conflicts. Implemented Planner and Reviewer nodes, constructed the StateGraph, and exposed the entry point. Fixed integration between validator (renders) and reviewer.
