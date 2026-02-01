# Tasks: VLM CAD Agent

**Feature**: 002-vlm-cad-agent
**Status**: Planned

## Work Packages

### WP01: Foundation & Infrastructure

**Goal**: Establish the Python project structure, install dependencies, and implement the core LLM client and Pydantic models.
**Priority**: Critical (Blocker)
**File**: [tasks/WP01-foundation.md](tasks/WP01-foundation.md)

- [x] **T001**: Scaffold Project Structure
- [x] **T002**: Install Dependencies (`langchain`, `langgraph`, `deepagents`)
- [x] **T003**: Configure LangChain ChatModels (OpenAI/Gemini)
- [x] **T004**: Define AgentState TypedDict
- [x] **T005**: Wrap Environmental Tools as LangChain Tools

### WP02: Memory & Context Management

**Goal**: Implement the state management systems, including filesystem Journal and Sliding Window Context.
**Priority**: High
**Dependencies**: WP01
**File**: [tasks/WP02-memory-context.md](tasks/WP02-memory-context.md)

- [x] **T006**: Implement File-System Memory (Workspace)
- [x] **T007**: Configure LangGraph Checkpointer (SQLite)
- [x] **T008**: Implement Journaler Node Logic

### WP03: Graph Architecture Implementation

**Goal**: Build the LangGraph State Machine with Planner, Actor, and Critic nodes.
**Priority**: High
**Dependencies**: WP02
**File**: [tasks/WP03-graph-architecture.md](tasks/WP03-graph-architecture.md)

- [x] T009 Implement Graph Builder & Conditionals
- [x] T010 Implement Planner Node (Reasoning)
- [x] T011 Implement Actor Node (Tool Calling)
- [x] T012 Implement Critic Node (Validation & Reflection)
- [x] T013 Implement Console Streaming (LangGraph Events)

### WP04: Runner & Integration

**Goal**: Create the CLI entry point and verify the agent works end-to-end with the Environment.
**Priority**: Medium
**Dependencies**: WP03
**File**: [tasks/WP04-runner-integration.md](tasks/WP04-runner-integration.md)

- [x] T014: Implement Environment Adapter for LangChain
- [x] T015: Implement Async Runner CLI
- [x] T016: Verify End-to-End Graph Execution
