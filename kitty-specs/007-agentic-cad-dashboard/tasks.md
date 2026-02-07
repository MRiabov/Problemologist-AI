# Tasks: Agentic CAD Dashboard

**Spec**: [Spec 007](../spec.md)
**Status**: Planning

## Work Packages

### WP01: Dashboard Skeleton & Infrastructure

- [x] T001: Initialize Vite React Project with TypeScript in `src/frontend`
- [x] T002: Setup TailwindCSS and Shadcn/UI (Button, Card, Input, Sheet, Sidebar)
- [x] T003: Configure React Router (Routes: Home, Run Detail, Wizard)
- [x] T004: Setup Layout Components (Sidebar, Header, AppShell)
- [x] T005: Configure ESLint, Prettier, and path aliases

**Summary**: Set up the foundational React technology stack and project structure.
**Dependencies**: None
**Est. Prompt Size**: ~300 lines

### WP02: API Integration & Data Layer

- [ ] T006: Setup OpenAPI Code Generation (from backend schema)
- [ ] T007: Configure TanStack Query (React Query) and Axios client
- [x] T008: Implement `RunService` and `BenchmarkService` wrappers (Basic `client.ts` implemented)
- [x] T009: specific "Runs List" Component (Home View) with pagination (Episode History sidebar implemented)
- [ ] T010: Create "Run Detail" Context/Hook for shared state

**Summary**: Establish the type-safe communication layer with the Controller.
**Dependencies**: WP01
**Est. Prompt Size**: ~300 lines

### WP03: Run Detail View - Live Logs

- [ ] T011: Implement WebSocket Client for live updates (`/api/episodes/{id}/ws`)
- [ ] T012: Build "Log Stream" Component (Console-like view for generic logs) (Visual skeleton exists)
- [ ] T013: Build "Structured Thought" Component (Collapsible steps for Thoughts/Tools) (Visual skeleton exists)
- [ ] T014: Integrate Live Updates into Run Detail Page
- [ ] T015: Add "Status Badge" and real-time state indicators (Visual skeleton exists)

**Summary**: Create the real-time debugging interface for agent observation.
**Dependencies**: WP02
**Est. Prompt Size**: ~400 lines

### WP04: 3D Asset Viewer

- [ ] T016: Provide `react-three-fiber` and `drei` setup
- [ ] T017: Implement STL/GLTF Reader Component
- [ ] T018: Build "3D Scene" Component with OrbitControls, Grid, and Lights (Visual skeleton exists)
- [ ] T019: Integrate 3D Viewer into "Run Detail" Tabs
- [ ] T020: Add "View Controls" (Wireframe, Auto-rotate, Reset View) (Visual buttons exist)

**Summary**: Implement the visualization capabilities for CAD artifacts.
**Dependencies**: WP02
**Est. Prompt Size**: ~350 lines

### WP05: Benchmark Wizard & Control

- [x] T021: Build "Benchmark Creation Wizard" (Multi-step Form) (Visual implementation in `BenchmarkGeneration.tsx`)
- [ ] T022: Implement "Plan -> Review -> Approve" Workflow UI (Visual steps exist)
- [ ] T023: Add "Interrupt/Stop" and "Retry" Action Buttons to Run Detail
- [ ] T024: Implement "New Engineer Run" Form
- [ ] T025: Final Polish (Toast notifications, Error boundaries, Empty states)

**Summary**: Enable user control over agents and the benchmark creation process.
**Dependencies**: WP03, WP04
**Est. Prompt Size**: ~400 lines
