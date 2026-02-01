---
work_package_id: "WP01"
title: "Foundation & Infrastructure"
lane: "for_review"
dependencies: []
subtasks:
  - "T001"
  - "T002"
  - "T003"
  - "T004"
  - "T005"
agent: "Antigravity"
shell_pid: "60800"
---

## Objective

Establish the foundational infrastructure for the VLM CAD Agent. This includes setting up the Python project structure, installing core dependencies (LiteLLM, Pydantic, etc.), implementing the Unified LLM Client, and defining the initial Prompt Templates and Tool Schemas.

## Context

This is the "Step 0" for the agent. We need a robust base where the agent can easily call LLMs (OpenAI, Gemini, Anthropic) via a unified interface and validate structured data. We are building the `src/agent` directory structure as defined in the plan.

## Subtasks

### T001: Scaffold Project Structure

**Purpose**: Create the directory hierarchy for the agent source code.
**Steps**:

1. Create the following directory structure inside `src/agent/`:

   ```text
   src/agent/
   ├── core/
   ├── clients/
   ├── prompts/
   └── __init__.py (and in subdirs)
   ```

2. Ensure `__init__.py` files are present to make them packages.

### T002: Install & Configure Dependencies

**Purpose**: Install required libraries.
**Steps**:

1. Add the following to `pyproject.toml` (or `requirements.txt` if used):
   - `litellm`: For unified LLM access.
   - `pydantic`: For data validation.
   - `jinja2`: For prompting.
   - `rich`: For beautiful terminal output.
   - `tenacity`: For retry logic (standard good practice).
2. Install dependencies.

### T003: Implement LLM Client

**Purpose**: Create a wrapper around `litellm` to handle model calls, retries, and configuration.
**Steps**:

1. Create `src/agent/clients/base.py` defining an abstract `LLMClient` class with `generate(messages, tools)` method.
2. Create `src/agent/clients/llm.py` implementing `LiteLLMClient`.
   - Use `tenacity` for retrying on RateLimitError or APIError.
   - Support loading API keys from environment variables.
   - Implement `generate` method that calls `litellm.completion`.
   - Ensure it supports passing a list of `tools` (JSON schemas).

### T004: Create Base Prompt Templates

**Purpose**: Set up Jinja2 environment and base templates.
**Steps**:

1. Create `src/agent/prompts/system_persona.j2` containing the system prompt (Refer to Spec 3.2 System Persona references or creates a placeholder based on "You are an expert mechanical engineer...").
2. Create `src/agent/prompts/planning.j2` for the Phase 0 planning step.
3. Create `src/agent/core/prompts.py` (or similar) to load these templates using `jinja2.Environment`.

### T005: Define Tool Interface Models

**Purpose**: Define Pydantic models for the environment tools to ensure type safety.
**Steps**:

1. Create `src/agent/clients/tools.py`.
2. Define Pydantic models for:
   - `WriteScript(content: str, path: str)`
   - `EditScript(find: str, replace: str, path: str)`
   - `PreviewDesign()`
   - `SubmitDesign(control_path: str)`
   - `SearchDocs(query: str)`
   - `ReadJournal(topic: str)`
   - `WriteJournal(entry: str, tags: list[str])`
   - `UpdatePlan(status: str, notes: str)`
3. Add a helper function to convert these models to OpenAI-compatible JSON schema format (using `.model_json_schema()`).

## Files to Create

- `src/agent/__init__.py`
- `src/agent/clients/__init__.py`
- `src/agent/clients/base.py`
- `src/agent/clients/llm.py`
- `src/agent/clients/tools.py`
- `src/agent/core/__init__.py`
- `src/agent/prompts/system_persona.j2`
- `src/agent/prompts/planning.j2`

## Acceptance Criteria

- [ ] Dependencies installed and importable.
- [ ] `LiteLLMClient` can successfully make a simple call to a model (mocked or real if key exists).
- [ ] Tool schemas are correctly generated as JSON.
- [ ] Jinja2 templates can be rendered with sample context.

## Activity Log

- 2026-02-01T07:36:05Z – Antigravity – shell_pid=60800 – lane=doing – Started implementation via workflow command
- 2026-02-01T07:43:04Z – Antigravity – shell_pid=60800 – lane=for_review – Implemented core infrastructure, tool models, and prompt management. Verified with unit tests.
