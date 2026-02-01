# Tasks: VLM CAD Agent

**Feature**: 002-vlm-cad-agent
**Status**: Planned

## Work Packages

### WP01: Foundation & Infrastructure

**Goal**: Establish the Python project structure, install dependencies, and implement the core LLM client and Pydantic models.
**Priority**: Critical (Blocker)
**File**: [tasks/WP01-foundation.md](tasks/WP01-foundation.md)

- [ ] **T001**: Scaffold Project Structure
- [ ] **T002**: Install & Configure Dependencies
- [ ] **T003**: Implement LLM Client (LiteLLM)
- [ ] **T004**: Create Base Prompt Templates
- [ ] **T005**: Define Tool Interface Models

### WP02: Memory & Context Management

**Goal**: Implement the state management systems, including filesystem Journal and Sliding Window Context.
**Priority**: High
**Dependencies**: WP01
**File**: [tasks/WP02-memory-context.md](tasks/WP02-memory-context.md)

- [ ] **T006**: Implement Memory System (Journal)
- [ ] **T007**: Implement Context Manager
- [ ] **T008**: Implement Meta-Cognitive Tool Logic

### WP03: Cognitive Engine Implementation

**Goal**: Build the "Think, Plan, Act" ReAct loop, including Phase 0 (Planning), Phase 1 (Execution), and Phase 2 (Validation).
**Priority**: High
**Dependencies**: WP02
**File**: [tasks/WP03-cognitive-engine.md](tasks/WP03-cognitive-engine.md)

- [ ] **T009**: Implement Engine Skeleton & State
- [ ] **T010**: Implement Phase 0 (Planning)
- [ ] **T011**: Implement Phase 1 (Execution Loop)
- [ ] **T012**: Implement Phase 2 (Validation) & Submission
- [ ] **T013**: Implement Logging & Rich UI

### WP04: Runner & Integration

**Goal**: Create the CLI entry point and verify the agent works end-to-end with the Environment.
**Priority**: Medium
**Dependencies**: WP03
**File**: [tasks/WP04-runner-integration.md](tasks/WP04-runner-integration.md)

- [ ] **T014**: Implement Environment Adapter
- [ ] **T015**: Implement CLI Runner
- [ ] **T016**: Verify End-to-End Flow & Smoke Test
