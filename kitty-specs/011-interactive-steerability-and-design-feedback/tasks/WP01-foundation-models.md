---
work_package_id: "WP01"
title: "Foundation & Data Models"
lane: "planned"
dependencies: []
subtasks: ["T001", "T002", "T013"]
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
