---
work_package_id: WP05
title: Sidecar Learner
lane: planned
dependencies: []
subtasks: [T020, T021, T022, T023]
---

## Objective

Implement the Sidecar Learner node. This component runs asynchronously (or at the end of a run) to analyze the agent's performance and suggest new skills.

## Context

The agent may fail repeatedly or discover a new trick. The Sidecar Learner analyzes the `journal.md` (Episodic Memory) to identify these patterns and crystallize them into reusable skills (`skills/*.md`).

## Subtasks

### T020: Implement `src/agent/nodes/sidecar.py`

Create the Sidecar node.

- **File**: `src/agent/nodes/sidecar.py`
- **Type**: Can be a standard node that runs after "End" or a separate process. For simplicity in LangGraph, implementing as a final node in the graph is easiest for V1.
- **Logic**:
  - Read `journal.md`.
  - Read `state["messages"]`.

### T021: Parse Journal for patterns

- **Implementation**:
  - Use LLM to scan the journal.
  - Look for "Struggle" (repeated errors) and "Breakthrough" (success after struggle).
  - Look for "Common Problem" (if historical logs available).

### T022: Implement Skill extraction logic

- **Implementation**:
  - If a pattern is found, draft a new skill file content.
  - Format: Markdown (Title, Problem, Solution, Example).
  - **Action**: Write to a temporary location (e.g., `suggested_skills/`) or log it. The spec says "separate administrative process", so don't overwrite `skills/` directly yet.

### T023: Test Sidecar

- **File**: `tests/agent/test_sidecar.py`
- **Test**:
  - Feed a fake journal with a clear struggle (e.g., "Syntax error in build123d" -> "Fixed by importing X").
  - Expect a suggested skill to be generated.

## Definition of Done

- Sidecar extracts insights from journals.
