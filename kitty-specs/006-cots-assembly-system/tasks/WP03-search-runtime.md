---
work_package_id: "WP03"
title: "Search Runtime"
lane: "planned"
dependencies: ["WP02"]
subtasks: ["T010", "T011", "T012"]
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
