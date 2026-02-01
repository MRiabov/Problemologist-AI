---
work_package_id: WP02
title: Memory & Context Management
lane: planned
dependencies: []
subtasks:
- T006
- T007
- T008
---

## Objective

Implement the agent's state management, including the sliding context window for LLM conversation history and the long-term filesystem-based memory (Journal).

## Context

The agent needs to maintain short-term consistency (conversation history) and long-term learning (Journal). We are implementing `src/agent/core/context.py` and `src/agent/core/memory.py`.

## Subtasks

### T006: Implement Memory System (Journal)

**Purpose**: Enable reading and writing to `journal.md`.
**Steps**:

1. Create `src/agent/core/memory.py`.
2. Implement `Journal` class.
   - `__init__(path: str)`
   - `read(topic: str = None) -> str`: If topic provided, perform simple text match/filter. If not, return recent/all.
   - `write(entry: str, tags: list[str])`: Append formatted entry with timestamp and tags to `journal.md`.
   - Ensure the file is created if it doesn't exist.

### T007: Implement Context Manager

**Purpose**: Manage the LLM message history, handling token limits.
**Steps**:

1. Create `src/agent/core/context.py`.
2. Implement `ContextManager` class.
   - Maintain a list of `messages` (role, content).
   - Implement `add_message(role, content, images=None)`.
   - Implement `get_messages_for_model()`: Return standard list for `litellm`.
   - **Sliding Window**: Implement basic pruning or summarization if message list gets too long (e.g., keep System + last N messages).
   - **Image Handling**: Ensure images passed to `add_message` are formatted correctly for VLM (e.g., `{"type": "image_url", ...}`).

### T008: Implement Meta-Cognitive Tool Logic

**Purpose**: Connect the Pydantic tool models (from WP01) to the actual Memory logic.
**Steps**:

1. Implement helper functions in `src/agent/core/tools_impl.py` (or similar) that execute the meta-tools:
   - `handle_read_journal(tool_call, memory_instance)`
   - `handle_write_journal(tool_call, memory_instance)`
   - `handle_update_plan(tool_call, state_instance)`
2. Ensure these functions return strings that can be fed back to the LLM as tool outputs.

## Files to Create

- `src/agent/core/memory.py`
- `src/agent/core/context.py`
- `src/agent/core/tools_impl.py` (Optional, can be part of engine later but good to separate)

## Acceptance Criteria

- [ ] `Journal` class correctly appends to file and reads from it.
- [ ] `ContextManager` correctly formats messages for LiteLLM.
- [ ] `ContextManager` prunes old messages when limit reached (unit test).
- [ ] Meta-tools can be executed via the handler functions.
