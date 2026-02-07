---
work_package_id: WP02
title: API Integration & Data Layer
lane: "planned"
dependencies: [WP01]
base_branch: 007-agentic-cad-dashboard-WP01
base_commit: b0b7bf77614a0fb87a464a2c2a457b04904e4222
created_at: '2026-02-07T07:35:09.440759+00:00'
subtasks: [T006, T007, T008, T009, T010]
shell_pid: "251987"
agent: "Gemini"
review_status: "has_feedback"
reviewed_by: "MRiabov"
---

## Context

The dashboard needs to fetch data from the Controller backend. We require a typed API client to ensure that frontend types match the backend OpenAPI schema.

This work package sets up the `openapi-typescript-codegen` (or compatible) client, React Query (TanStack Query) for state management, and implements the data fetching for the main lists.

## Objective

Establish a robust, type-safe data fetching layer and implement the "Runs List" view.

## Subtasks

### T006: Setup OpenAPI Code Generation

- **Goal**: Generate TypeScript client from backend schema.
- **Details**:
  - Install `openapi-typescript-codegen` as a dev dependency.
  - Create a script in `package.json`: `"gen:api": "openapi --input http://localhost:8000/openapi.json --output ./src/api --client axios"`.
  - **Note**: Since backend might not be running, you may need to rely on a local `openapi.json` file. If one exists in the repo, point to it. If not, assume a structure or ask user to provide one. *For this task, assume we can mock or use a provided schema file.*
  - Run the generation to create `src/api`.
- **Files**:
  - `src/frontend/package.json`
  - `src/frontend/src/api/` (generated)

### T007: Configure TanStack Query

- **Goal**: Setup global data fetching state.
- **Details**:
  - Install `@tanstack/react-query` and `@tanstack/react-query-devtools`.
  - Wrap the App in `QueryClientProvider`.
  - Configure default options (staleTime, retry).
  - Setup a global `Axios` instance with base URL (configured via env var `VITE_API_URL`).
- **Files**:
  - `src/frontend/src/lib/queryClient.ts`
  - `src/frontend/src/App.tsx`
  - `src/frontend/src/lib/axios.ts`

### T008: Implement Service Wrappers

- **Goal**: Create clean interface for API calls.
- **Details**:
  - While the generated client is good, creating custom hooks (`useRuns`, `useRunDetail`) using React Query is best practice.
  - Create `src/hooks/useRuns.ts`.
  - Create `src/hooks/useBenchmark.ts`.
- **Files**:
  - `src/frontend/src/hooks/api/useRuns.ts`

### T009: Implement "Runs List" Component

- **Goal**: Display the list of past/active agent runs.
- **Details**:
  - Update `HomePage.tsx`.
  - Fetch runs using `useRuns`.
  - Display in a Shadcn `Table` or `Card` list.
  - Columns: ID, Status (Badge), Goal (Truncated), Created At, Actions (View).
  - Add Pagination support (mocked if API doesn't support yet, but plan for it).
- **Files**:
  - `src/frontend/src/pages/HomePage.tsx`
  - `src/frontend/src/components/runs/RunsList.tsx`

### T010: Create "Run Detail" Context

- **Goal**: Manage state for the specific run view.
- **Details**:
  - Create `RunContext` to store the current `runId` and `run` data.
  - This will be used by the sub-components (Logs, 3D viewer) to access shared data without prop drilling.
- **Files**:
  - `src/frontend/src/context/RunContext.tsx`

## Implementation Guidelines

- **Type Safety**: strict `noImplicitAny`. Use the generated types for API responses.
- **Mocking**: If the backend is not ready, use `msw` (Mock Service Worker) or simple manual mocks in the hooks to simulate data. *Prefer simple mocking for speed.*

## Validation (Definition of Done)

- [ ] API client generates successfully (or mocks are in place).
- [ ] Home Page loads and displays a list of "Runs" (dummy or real).
- [ ] Navigation to Run Detail sets the context correctness.
- [ ] No type errors in the console.

## Activity Log

- 2026-02-07T07:35:09Z – gemini-cli – shell_pid=229530 – lane=doing – Assigned agent via workflow command
- 2026-02-07T07:53:40Z – gemini-cli – shell_pid=229530 – lane=for_review – Ready for review: Implemented API integration with generated client, React Query setup, RunsList component, and RunContext.
- 2026-02-07T07:54:50Z – Gemini – shell_pid=251987 – lane=doing – Started review via workflow command
- 2026-02-07T07:58:30Z – Gemini – shell_pid=251987 – lane=planned – Moved to planned
