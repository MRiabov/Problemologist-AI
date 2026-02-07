---
work_package_id: WP03
title: Run Detail View - Live Logs
lane: "doing"
dependencies: [WP02]
base_branch: 007-agentic-cad-dashboard-WP02
base_commit: 422ffc62c8a8174f4eebe4a08be3ba5bc346e0e6
created_at: '2026-02-07T10:09:50.639685+00:00'
subtasks: [T011, T012, T013, T014, T015]
shell_pid: "322089"
agent: "Gemini"
---

## Context

Visual debugging is a core feature. We need to show the agent's "Checklist" (Thoughts), "Tool Calls" (Actions), and raw logs in real-time. This requires WebSocket integration.

## Objective

Build the "Run Detail" page's logging and observability panels utilizing WebSockets for live streaming.

## Subtasks

### T011: Implement WebSocket Client

- **Goal**: Connect to `/api/episodes/{id}/ws` and handle messages.
- **Details**:
  - Create a `useRunWebSocket` hook.
  - Handle connection states (Connecting, Connected, Disconnected).
  - Parse incoming messages (`DashboardUpdate` model):
    - `log`: Append to raw log.
    - `trace`: Append/Update thought trace.
    - `status_change`: Update run status.
- **Files**:
  - `src/frontend/src/hooks/useRunWebSocket.ts`

### T012: Build "Log Stream" Component

- **Goal**: specific "Console" view for raw logs.
- **Details**:
  - A scrollable area with monospace font.
  - Auto-scroll to bottom behavior.
  - Syntax highlighting for JSON/Python logs (optional, simple coloring is enough).
- **Files**:
  - `src/frontend/src/components/logs/LogStream.tsx`

### T013: Build "Structured Thought" Component

- **Goal**: Visualizing the "Reasoning Trace".
- **Details**:
  - A list of steps (Accordion or Timeline style).
  - Each step shows: Thought content + Tool Call (if any) + Result.
  - Status icons (Thinking, Success, Failed).
  - Use Shadcn `Accordion` or custom styled div.
- **Files**:
  - `src/frontend/src/components/logs/ThoughtTrace.tsx`

### T014: Integrate Live Updates into Run Detail Page

- **Goal**: Assemble the Run Detail view.
- **Details**:
  - Split screen layout (or Tabs):
    - Left: Thought Trace (Structured).
    - Bottom/Right: Raw Logs.
    - (3D Viewer will go in the other pane in WP04).
  - Connect the `useRunWebSocket` hook to feed these components.
- **Files**:
  - `src/frontend/src/pages/RunDetailPage.tsx`

### T015: Add "Status Badge" and Indicators

- **Goal**: Visual feedback on run state.
- **Details**:
  - Components for: "Running", "Completed", "Failed", "Paused".
  - Pulsing dot for "Running".
  - Header of Run Detail page shows these statuses.
- **Files**:
  - `src/frontend/src/components/runs/RunStatusBadge.tsx`

## Implementation Guidelines

- **Performance**: Ensure the log list is virtualized if logs get very long (use `react-window` if necessary, or just CSS `overflow-anchor`).
- **Resiliency**: WebSocket should attempt auto-reconnect.

## Validation (Definition of Done)

- [ ] WebSocket connects to endpoint (or mock).
- [ ] Incoming messages appear in "Log Stream" immediately.
- [ ] "Thought Trace" updates structure dynamically.
- [ ] Status badge reflects current state.

## Activity Log

- 2026-02-07T10:09:50Z – Gemini – shell_pid=322089 – lane=doing – Assigned agent via workflow command
