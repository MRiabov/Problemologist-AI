---
work_package_id: "WP04"
subtasks:
  - "T015"
  - "T016"
  - "T017"
  - "T018"
  - "T019"
title: "Environment Tools & RAG"
phase: "Phase 4 - Agent Interface"
lane: "planned"
dependencies: ["WP01"]
history:
  - timestamp: "{{TIMESTAMP}}"
    lane: "planned"
    agent: "system"
    action: "Prompt generated via /spec-kitty.tasks"
---

# Work Package Prompt: WP04 – Environment Tools & RAG

## Objectives & Success Criteria

- **Goal**: Implement the tools the agent actually uses: editing code, viewing results, and searching docs.
- **Success Criteria**:
  - `write_script` updates the file.
  - `edit_script` accurately replaces text.
  - `preview_design` generates an image.
  - `search_docs` returns relevant snippets.

## Context & Constraints

- **Spec**: 4.2 Tool Suite.
- **Tools**: These must be stateless functions (or functional helpers) that the `core.py` will wrap.

## Subtasks & Detailed Guidance

### Subtask T015 – Implement RAG Search (Simple)

- **Purpose**: Help the agent learn syntax.
- **Steps**:
  - Create `src/rag/search.py`.
  - Implement `load_docs(path)`.
  - Implement `search(query: str) -> str`.
  - MVP: A simple glob search over `.md` or `.py` files in `build123d` folder (if available) or just a substring search. Return top 3 matches.
  - Stub with hardcoded common answers if actual docs aren't available yet.
- **Files**: `src/rag/search.py`.
- **Parallel?**: Yes.

### Subtask T016 – Tools Structure & Write Script

- **Purpose**: Base file operations.
- **Steps**:
  - Create `src/environment/tools.py`.
  - Implement `write_script(content: str, filename: str = "design.py")`.
  - Should write to a temporary/workspace path, not repo root.
- **Files**: `src/environment/tools.py`.
- **Parallel?**: No.

### Subtask T017 – Edit Script

- **Purpose**: Precise editing.
- **Steps**:
  - In `src/environment/tools.py`, implement `edit_script(filename, find, replace)`.
  - Read file, `content.replace(find, replace)`, write back.
  - Return error if `find` string is ambiguous (count > 1) or not found.
- **Files**: `src/environment/tools.py`.
- **Parallel?**: No.

### Subtask T018 – Preview Design (Visuals)

- **Purpose**: "Eyes" for the agent.
- **Steps**:
  - In `src/environment/tools.py`, implement `preview_design(filename)`.
  - Logic:
    1. Exec the script (sandboxed ideally, but `exec()` is fine for local MVP). It should produce a `part` variable or `show_object`.
    2. Extract the object.
    3. Use `build123d` export or `vtk`/`matplotlib` to render an isometric view to a PNG.
    4. Return path to PNG.
- **Files**: `src/environment/tools.py`.
- **Parallel?**: Yes.

### Subtask T019 – Search Docs Wrapper

- **Purpose**: Wrap the RAG for the agent.
- **Steps**:
  - In `src/environment/tools.py`, implement `search_docs(query)`.
  - Call `rag.search`.
- **Files**: `src/environment/tools.py`.
- **Parallel?**: No.

## Test Strategy

- **Test File**: `tests/test_tools.py`.
- **Cases**:
  - Write -> Read back.
  - Edit -> Check change.
  - Edit (Partial match) -> Fail.
  - Preview -> Check PNG generated.

## Risks & Mitigations

- **Exec Safety**: `exec()` is dangerous. Warn in comments. For now, trust the agent (local execution).
- **Preview Deps**: Rendering can depend on display drivers. If headless fails, use `build123d` SVG export as a fallback (it's robust and stateless).
