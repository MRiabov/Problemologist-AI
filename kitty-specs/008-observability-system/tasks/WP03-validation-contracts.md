---
work_package_id: "WP03"
title: "Validation & Contracts"
lane: "for_review"
dependencies: []
subtasks: ["T008", "T009", "T010"]
agent: "Gemini"
shell_pid: "36079"
---

# Work Package: Validation & Contracts

## Objective

Enforce strict data contracts across the observability system. We will define robust Pydantic models for all data structures and use `schemathesis` to fuzz-test the API endpoints, ensuring they adhere to the OpenAPI specification.

## Context

To prevent "silent failures" and data corruption, we require strict schema validation. The Controller and Worker communicate via API, and the Observability system ingests data from both. By defining clear Pydantic models and fuzzing inputs, we ensure the system is resilient to malformed data.

## Implementation Guide

### Subtask T008: Define Pydantic models

**Purpose**: Create the canonical data structures for traces and assets.

**Steps**:

1. Create `src/models/observability.py`.
2. Define `TraceEvent(BaseModel)`:
    - Fields: `trace_id`, `agent_id`, `input_tokens`, `output_tokens`, `latency_ms`, `content`, `timestamp`.
3. Define `AssetRecord(BaseModel)`:
    - Fields: `asset_id`, `trace_id` (foreign key equivalent), `s3_key`, `asset_type` (Enum: VIDEO, IMAGE, LOG), `created_at`.
4. Ensure strict types (use `pydantic.StrictStr`, `pydantic.StrictInt` where appropriate).

**Files**:

- `src/models/observability.py` (New)

---

### Subtask T009: Verify and align `data-model.md`

**Purpose**: Ensure documentation matches the code.

**Steps**:

1. Read `kitty-specs/008-observability-system/data-model.md` and `src/models/observability.py`.
2. Update `data-model.md` to reflect any fields added or renamed during T008.
3. (Optional) Add a comment or docstring in the code referencing the spec version.

**Files**:

- `kitty-specs/008-observability-system/data-model.md` (Update)

---

### Subtask T010: Set up `schemathesis` testing

**Purpose**: Fuzz the observability API endpoints (mocked or real) to find edge cases.

**Steps**:

1. Create `tests/contracts/test_observability.py`.
2. Setup a test that runs `schemathesis` against the running app (or specific router).
3. If using FastAPI, use `schemathesis.from_asgi` or `schemathesis.from_pytest_fixture`.
4. Configure checks: `not_a_server_error`, `response_schema_conformance`.
5. Mock the S3/Postgres layers so the fuzzing tests the *API contract* logic, not the backend stability.

**Files**:

- `tests/contracts/test_observability.py` (New)

**Validation**:

- Run `pytest tests/contracts/` and ensure it executes multiple cases (Schemathesis default is usually 100 cases).

## Definition of Done

- [ ] `src/models/observability.py` contains Pydantic models.
- [ ] `data-model.md` is in sync.
- [ ] Schemathesis tests pass and cover the observability endpoints (even if currently empty/mocked, they should be defined).

## Reviewer Guidance

- Ensure Pydantic V2 is used if available.
- Check that `schemathesis` is configured to ignore "expected" 4xx errors if they are part of the contract.

## Activity Log

- 2026-02-06T11:55:38Z – Antigravity – shell_pid=567566 – lane=doing – Started review via workflow command
- 2026-02-06T11:56:16Z – Antigravity – shell_pid=567566 – lane=planned – Moved to planned
- 2026-02-06T16:25:01Z – antigravity – shell_pid=11005 – lane=doing – Started review via workflow command
- 2026-02-06T16:26:44Z – antigravity – shell_pid=11005 – lane=planned – Moved to planned
- 2026-02-06T17:22:25Z – antigravity – shell_pid=36079 – lane=doing – Started implementation via workflow command
- 2026-02-06T20:55:34Z – Gemini – shell_pid=36079 – lane=for_review – Moved to for_review
