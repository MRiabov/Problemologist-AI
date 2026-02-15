---
work_package_id: WP06
title: Chat Enhancements (@-mentions & Code Steering)
lane: "doing"
dependencies: []
base_branch: main
base_commit: daef541770cc1f7e0367eefb41dcb82690465567
created_at: '2026-02-15T12:46:12.399543+00:00'
subtasks: [T010, T011]
shell_pid: "102877"
---

# WP06 - Chat Enhancements (@-mentions & Code Steering)

## Objective
Implement @-mention autocomplete for parts and subassemblies and line-targeted code steering syntax in the chat interface.

## Context
- Quickstart: `kitty-specs/011-interactive-steerability-and-design-feedback/quickstart.md`
- Spec: `kitty-specs/011-interactive-steerability-and-design-feedback/spec.md` (Story 2 & 4)

## Subtasks

### T010: Implement @-mention Autocomplete
**Purpose**: Simplify referencing BOM parts in the chat.
**Steps**:
1. Update `frontend/src/components/Chat/ChatInput.tsx`.
2. Integrate a mention library (e.g., `react-mentions` or custom logic).
3. Populate suggestions from the assembly tree (BOM) state.
4. When a mention is selected (e.g., `@bracket_1`), it should be visually styled and its ID stored in the message's `mentions` metadata.
**Validation**:
- [ ] Typing `@` shows a list of parts from the model.
- [ ] Selecting a part inserts the tag and populates the `mentions` array in the API request.

### T011: Implement Code Steering Syntax
**Purpose**: Allow targeted edits to specific line ranges.
**Steps**:
1. Implement regex-based parsing for `@filename:L1-L2` in the chat input.
2. Validate the reference against the current workspace file list.
3. Store the reference in the message's `code_references` metadata.
4. (Optional) Highlight the referenced lines in the `CodeViewer` when the mention is clicked or typed.
**Validation**:
- [ ] Typing `@model.py:10-20` correctly populates `code_references` in the API payload.
- [ ] UI provides feedback (e.g., red underline) if the file or range is invalid.

## Definition of Done
- @-mention autocomplete is functional with BOM data.
- Code steering syntax is parsed and transmitted to the backend.
