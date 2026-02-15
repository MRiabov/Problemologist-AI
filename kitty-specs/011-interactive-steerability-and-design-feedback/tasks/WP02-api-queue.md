---
work_package_id: WP02
title: Steerability API & Queue Management
lane: "doing"
dependencies: []
base_branch: main
base_commit: 63b616d3d42dac8c1fc9f99ba52c94a2e544b896
created_at: '2026-02-15T09:38:20.947190+00:00'
subtasks: [T003, T004]
shell_pid: "86870"
---

# WP02 - Steerability API & Queue Management

## Objective
Create the `SteerabilityService` to manage the in-memory `asyncio.Queue` for interaction steering and expose the REST endpoints for the frontend.

## Context
- API Contract: `kitty-specs/011-interactive-steerability-and-design-feedback/contracts/steerability.yaml`
- Research: `kitty-specs/011-interactive-steerability-and-design-feedback/research.md` (Queue section)

## Subtasks

### T003: Implement SteerabilityService
**Purpose**: Manage transient, in-memory queues for user prompts.
**Steps**:
1. Create `controller/services/steerability/service.py`.
2. Implement `SteerabilityService` using a `dict[session_id, asyncio.Queue[SteerablePrompt]]` to store active queues.
3. Methods: `enqueue_prompt(session_id, prompt)`, `dequeue_prompt(session_id)`, `get_queue(session_id)`.
4. Ensure the queue is thread-safe and scoped to the agent session.
**Validation**:
- [ ] Multiple prompts for the same session are queued in order.
- [ ] Prompts from different sessions are isolated.

### T004: Implement Steerability Endpoints
**Purpose**: Expose the steering functionality to the frontend.
**Steps**:
1. Create `controller/api/routes/steerability.py`.
2. Add `POST /api/v1/sessions/{session_id}/steer`:
    - Validates `SteerablePrompt`.
    - Checks if agent is "busy" (already executing).
    - If busy, enqueues to `SteerabilityService`.
    - If idle, sends directly to agent (integration in WP04).
3. Add `GET /api/v1/sessions/{session_id}/queue`:
    - Returns the list of currently queued prompts.
4. Register the router in `controller/api/main.py`.
**Validation**:
- [ ] Endpoints match the OpenAPI spec.
- [ ] `POST /steer` returns 202 Accepted and the queue position.

## Definition of Done
- `SteerabilityService` is functional in the controller.
- REST endpoints are registered and passing basic integration tests.
