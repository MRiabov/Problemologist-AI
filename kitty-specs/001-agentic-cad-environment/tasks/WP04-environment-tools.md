---
work_package_id: WP04
title: Environment Tools & RAG
lane: "done"
dependencies: []
subtasks:
- T015
- T016
- T017
- T018
- T019
phase: Phase 4 - Agent Interface
assignee: "Antigravity"
agent: "Antigravity"
shell_pid: "90978"
review_status: "has_feedback"
reviewed_by: "MRiabov"
history:
- timestamp: '{{TIMESTAMP}}'
  lane: done
  assignee: "Antigravity"
agent: system
  action: Prompt generated via /spec-kitty.tasks
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

## Activity Log

- 2026-02-01T08:36:06Z – Antigravity – shell_pid=90978 – lane=doing – Started implementation via workflow command
- 2026-02-01T08:39:11Z – Antigravity – shell_pid=90978 – lane=for_review – Implemented environment tools, RAG search with stubs, and added unit tests. All tests passed.
- 2026-02-01T08:40:50Z – Antigravity – shell_pid=113637 – lane=doing – Started review via workflow command
- 2026-02-01T08:43:13Z – Antigravity – shell_pid=113637 – lane=planned – Moved to planned
- 2026-02-01T08:53:24Z – Antigravity – shell_pid=113637 – lane=for_review – Addressed feedback: Implemented real SVG preview using build123d.Drawing, added exec safety warnings, and updated tests.
- 2026-02-01T08:54:47Z – Antigravity – shell_pid=90978 – lane=doing – Started review via workflow command
- 2026-02-01T08:57:42Z – Antigravity – shell_pid=90978 – lane=done – Review passed: Implemented environment tools including write_script, edit_script with ambiguity check, preview_design with SVG export fallback, and RAG search with stubs. Tests passed.
- 2026-02-01T18:41:16Z – Antigravity – lane=done – Marking as done for acceptance check
