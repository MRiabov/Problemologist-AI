# Work Packages: Interactive Steerability and Design Feedback

This document breaks down the implementation of the Steerability framework into manageable work packages.

## Setup & Foundations

### WP01 - Foundation & Data Models (Priority: P1)
**Goal**: Define the core data structures and persistence for steering metadata and user preferences.
**Independent Test**: Unit tests for Pydantic models and SQLAlchemy schema migrations.

- [x] T001: Implement `GeometricSelection` and `SteerablePrompt` Pydantic models in `shared/models/steerability.py`. [P]
- [x] T002: Implement `UserSteeringPreference` model and SQLAlchemy schema in `controller/persistence/steering_memory.py`.
- [x] T013: Implement persistent user preference storage logic in `controller/persistence/steering_memory.py`.

**Estimated Prompt Size**: ~250 lines

---

### WP02 - Steerability API & Queue Management (Priority: P1)
**Goal**: Create the async queue service and the API endpoints for receiving steered prompts.
**Independent Test**: API integration tests verifying that `POST /steer` correctly populates the in-memory queue.

- [x] **T003**: Create `SteerabilityService` in `controller/services/steerability/service.py` with `asyncio.Queue` management for `TurnQueue`.
- [x] **T004**: Implement `POST /api/v1/sessions/{session_id}/steer` and `GET /api/v1/sessions/{session_id}/queue` in `controller/api/routes/steerability.py`.

**Estimated Prompt Size**: ~300 lines

---

## Backend Intelligence & Rendering

### WP03 - Worker Topology & Rendering (Priority: P1)
**Goal**: Implement the headless rendering logic and topological inspection tools on the worker.
**Independent Test**: Verify `inspect_topology` returns correct geometric data and `rendering.py` produces highlighted isometric snapshots.

- [x] T005: Add `inspect_topology` tool to `worker/tools/topology.py` to return geometric properties of selected features.
- [x] T006: Implement best-angle isometric view selection logic in `shared/simulation/view_utils.py`.
- [x] T007: Update `worker/activities/rendering.py` to support headless snapshots with highlighted features.

**Estimated Prompt Size**: ~450 lines

---

### WP04 - LangGraph Integration (Priority: P2)
**Goal**: Inject the interaction queue into the LangGraph loop to allow mid-trace steering.
**Independent Test**: A mock LangGraph trace is interrupted and redirected by a message in the `TurnQueue`.

- [ ] **T008**: Implement the `SteeringQueue` middleware/node in `controller/graph/steerability_node.py` for LangGraph integration.
- [ ] **T009**: Integrate `SteerabilityService` into the main agent execution loop to deliver queued prompts between tool calls.

**Estimated Prompt Size**: ~350 lines

---

## Frontend UX

### WP05 - Frontend Selection & Toggles (Priority: P2)
**Goal**: Add multi-level selection modes and the mode toggle to the CAD viewer.
**Independent Test**: UI verification that toggling modes correctly changes selection behavior in `three-cad-viewer`.

- [x] **T010**: Update frontend `CADViewer` component to support selection modes (Face, Part, Subassembly) and UI toggle.

**Estimated Prompt Size**: ~350 lines

---

### WP06 - Chat Enhancements (Priority: P3)
**Goal**: Implement @-mentions, autocomplete, and line-targeted code steering in the chat interface.
**Independent Test**: Verification of autocomplete suggestions from BOM and visual highlighting of code references.

- [ ] **T011**: Implement @-mention autocomplete in frontend `Chat` component using the BOM tree.
- [ ] **T012**: Implement line-range parsing and UI highlighting for code steering in frontend `Chat` and `CodeViewer`.

**Estimated Prompt Size**: ~400 lines
