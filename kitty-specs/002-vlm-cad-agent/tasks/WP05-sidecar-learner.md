---
work_package_id: WP05
title: Sidecar Learner
lane: "for_review"
dependencies: []
base_branch: 002-vlm-cad-agent-WP04
base_commit: e24a3cc4acf4d23b851337bdd57e3014cb8246b0
created_at: '2026-02-06T14:40:04.508545+00:00'
subtasks: [T020, T021, T022, T023]
shell_pid: "658159"
agent: "Gemini"
---

## Objective

Implement the Sidecar Learner node. This component runs asynchronously (or at the end of a run) to analyze the agent's performance and suggest new skills.

## Context

The agent may fail repeatedly or discover a new trick. The Sidecar Learner analyzes the `journal.md` (Episodic Memory) to identify these patterns and crystallize them into reusable skills (`skills/*.md`).

## Subtasks

### T020: Implement `src/agent/nodes/sidecar.py`

Create the Sidecar node as an asynchronous background worker.

- **File**: `src/agent/nodes/sidecar.py`
- **Type**: A non-blocking process triggered at the end of an episode (Success or Failure). It should not delay the final response to the user.
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

## Activity Log

- 2026-02-06T14:40:04Z – Gemini – shell_pid=658159 – lane=doing – Assigned agent via workflow command
- 2026-02-06T14:41:47Z – Gemini – shell_pid=658159 – lane=for_review – Ready for review: Sidecar learner implemented for skill extraction from journals.
