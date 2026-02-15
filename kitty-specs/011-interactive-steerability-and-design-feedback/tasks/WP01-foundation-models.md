---
work_package_id: WP01
title: Foundation & Data Models
lane: "for_review"
dependencies: []
base_branch: main
base_commit: 89a2d71d9c4ffab3699748b642fc061636c03c93
created_at: '2026-02-15T09:14:19.310711+00:00'
subtasks: [T001, T002, T013]
shell_pid: "79713"
agent: "Gemini"
review_status: "has_feedback"
reviewed_by: "MRiabov"
---

# WP01 - Foundation & Data Models

## Objective
Implement the core data structures and persistence layer for the Steerability framework. This includes Pydantic models for geometric selections and prompts, as well as the SQLAlchemy schema for per-user design memory.

## Context
Refer to the following documents:
- Spec: `kitty-specs/011-interactive-steerability-and-design-feedback/spec.md`
- Data Model: `kitty-specs/011-interactive-steerability-and-design-feedback/data-model.md`

## Subtasks

### T001: Implement Steerability Pydantic Models
**Purpose**: Define the schema for geometric selections and steered prompts.
**Steps**:
1. Create `shared/models/steerability.py`.
2. Implement `SelectionLevel` StrEnum (FACE, EDGE, VERTEX, PART, SUBASSEMBLY).
3. Implement `GeometricSelection` model with fields: `level`, `target_id`, `center` (Vector3), `normal`, `direction`.
4. Implement `CodeReference` model with `file_path`, `start_line`, `end_line`.
5. Implement `SteerablePrompt` model wrapping text, selections, code references, and mentions.
**Validation**:
- [ ] Models pass Pydantic validation with sample data.
- [ ] Enums are correctly enforced.

### T002: Implement UserSteeringPreference DB Schema
**Purpose**: Persist user-specific design preferences.
**Steps**:
1. Create `UserSteeringPreference` model in `controller/persistence/models.py` (or equivalent persistence file).
2. Fields: `user_id` (str), `preference_key` (str), `preference_value` (JSON/Dict), `last_updated` (DateTime).
3. Create an Alembic migration for the new table.
**Validation**:
- [ ] Migration runs successfully (`alembic upgrade head`).
- [ ] Table structure matches the design.

### T013: Implement Preference Storage Logic
**Purpose**: CRUD operations for user steering preferences.
**Steps**:
1. Add `get_user_preferences` and `set_user_preference` methods to `controller/persistence/steering_memory.py`.
2. Ensure preferences are scoped to `user_id`.
**Validation**:
- [ ] Preferences are correctly saved and retrieved from Postgres.

## Definition of Done
- Pydantic models are in `shared/models/`.
- DB schema and migration are complete.
- Basic CRUD for preferences is implemented.

## Activity Log

- 2026-02-15T09:14:19Z – Gemini – shell_pid=61843 – lane=doing – Assigned agent via workflow command
- 2026-02-15T09:19:54Z – Gemini – shell_pid=61843 – lane=for_review – Ready for review: Foundation and data models implemented, including Pydantic models, DB schema, migration, and storage logic.
- 2026-02-15T09:21:11Z – Gemini – shell_pid=69990 – lane=doing – Started review via workflow command
- 2026-02-15T09:23:01Z – Gemini – shell_pid=69990 – lane=planned – Changes requested: Missing tests for models and persistence logic.
- 2026-02-15T09:31:28Z – Gemini – shell_pid=79713 – lane=doing – Started implementation via workflow command
- 2026-02-15T09:34:06Z – Gemini – shell_pid=79713 – lane=for_review – Addressed review feedback: Added unit tests for steerability Pydantic models and CRUD tests for user steering preferences using an in-memory DB. Verified all tests pass.
