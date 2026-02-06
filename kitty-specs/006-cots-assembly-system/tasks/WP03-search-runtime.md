---
work_package_id: WP03
title: Search Runtime
lane: "doing"
dependencies: [WP02]
base_branch: 006-cots-assembly-system-WP02
base_commit: 0b50c8a77effff8e39c1ec8dd4b9f02d6dd6e89d
created_at: '2026-02-06T17:24:57.268035+00:00'
subtasks: [T010, T011, T012]
shell_pid: "98943"
agent: "Gemini"
---

## WP03: Search Runtime

**Goal**: Expose the catalog to agents via a search tool.

## Subtasks

### T010: Implement `search_parts` function

**Purpose**: Query the database.
**File**: `src/cots/runtime.py`
**Steps**:

- Function `search_parts(query: SearchQuery, db_path: str) -> List[COTSItem]`.
- Logic:
  - If `query.constraints` are provided, apply them via SQL `WHERE` clauses (e.g. `weight_g < 100`).
  - Use simple LIKE matching for name/category if full-text search isn't available in SQLite (or use FTS5 if supported).
  - Limit results.

### T011: Implement Search Subagent

**Purpose**: Provide a reasoning layer for part discovery.
**File**: `src/cots/agent.py`

- Define a `SearchAgent` (using LangGraph/LangChain).
- Tools: `search_parts` (SQL search).
- Prompt: "You are a part lookup assistant. Given a description, find the best-fit part. You can run multiple SQL queries."
- Expose the agent's entrypoint as a tool for the main Engineer agent.

### T012: Integration Test

**Purpose**: Verify end-to-end flow.
**File**: `tests/cots/test_search.py`
**Steps**:

- Setup: Create a temp DB, insert 1 dummy part.
- Execute `search_parts`.
- Verify dummy part is returned.

## Validation

- [ ] Search returns correct parts.
- [ ] Constraints filter results.
- [ ] Tool is importable.

## Implementation Command

`spec-kitty implement WP03 --base WP02`

## Activity Log

- 2026-02-06T17:24:57Z – antigravity – shell_pid=37710 – lane=doing – Assigned agent via workflow command
- 2026-02-06T20:34:38Z – antigravity – shell_pid=37710 – lane=for_review – Ready for review: Implemented search runtime, search agent tool, and added integration tests. Fixed a bug in create_react_agent where state_modifier was used instead of prompt.
- 2026-02-06T20:48:22Z – Gemini – shell_pid=98943 – lane=doing – Started review via workflow command
