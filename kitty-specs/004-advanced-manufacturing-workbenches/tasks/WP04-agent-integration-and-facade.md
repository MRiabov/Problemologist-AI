---
work_package_id: WP04
title: Agent Integration & Facade
lane: "doing"
dependencies: [WP02, WP03]
base_branch: 004-advanced-manufacturing-workbenches-WP03
base_commit: 872ab04f9f4c239be6d4356d2898ae23339c05bf
created_at: '2026-02-06T21:17:18.899183+00:00'
subtasks: [T015, T016]
shell_pid: "136359"
agent: "Gemini-CLI"
---

# WP04: Agent Integration & Facade

## Goal

Expose the workbench functionality to the Agent via a unified functional facade and Pydantic report.

## Context

Implement the `validate_and_price` function in `src/worker/utils/dfm.py` which acts as the entry point for agents. It dispatches to specific workbench functions.

## Subtasks

### T015: Implement Functional Facade

**Objective**: Create the entry point function.
**Files**: `src/worker/utils/dfm.py`
**Instructions**:

**Instructions**:

1. Create `tests/workbenches/test_facade.py`.
2. Test calling `validate_and_price` with a CNC part. verify it calls CNC logic.
3. Test calling with IM part.
4. Test calling with missing metadata (Should raise clear error).

## Verification

- Run `pytest tests/workbenches/test_facade.py`.

## Definition of Done

- `validate_and_price` is callable.
- Dispatches correctly.
- Returns clear errors on misuse.

## Activity Log

- 2026-02-06T21:17:19Z – Gemini – shell_pid=128035 – lane=doing – Assigned agent via workflow command
- 2026-02-07T06:18:39Z – Gemini – shell_pid=128035 – lane=for_review – Ready for review: Implemented unified DFM facade in src/worker/utils/dfm.py that dispatches to CNC and IM workbenches. Added comprehensive tests in tests/workbenches/test_facade.py.
- 2026-02-07T06:19:59Z – Gemini-CLI – shell_pid=136359 – lane=doing – Started review via workflow command
