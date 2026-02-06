---
work_package_id: "WP05"
title: "Benchmark Wizard & Control"
lane: "planned"
dependencies: ["WP03", "WP04"]
subtasks: ["T021", "T022", "T023", "T024", "T025"]
---

## Context

The final piece is interaction: managing the creation of benchmarks (Spec 005) and controlling running agents (Interrupt/Retry).

## Objective

Implement the "Wizard" for benchmark creation and the control actions for the Run Detail page.

## Subtasks

### T021: Build Benchmark Creation Wizard UI

- **Goal**: Multi-step form container.
- **Details**:
  - Route: `/wizard`.
  - Steps:
        1. **Prompt**: Text area for user description.
        2. **Plan**: View generated plan (Markdown).
        3. **Review**: View generated assets (using `CadViewerScene`).
        4. **Finish**: Summary.
  - Use state machine or simple step state.
- **Files**:
  - `src/frontend/src/pages/BenchmarkWizardPage.tsx`

### T022: Implement "Plan -> Review -> Approve" Workflow

- **Goal**: Logic for the wizard.
- **Details**:
  - **Step 1 Next**: Call API `POST /benchmarks/generate-plan`.
  - **Step 2 Next**: Call API `POST /benchmarks/generate-assets`.
  - **Step 3 (Review)**: User approves or rejects.
    - Approve: `POST /benchmarks/approve`.
    - Reject: Loop back to Prompt/Plan with feedback.
- **Files**:
  - `src/frontend/src/pages/BenchmarkWizardPage.tsx`
  - `src/frontend/src/hooks/useBenchmarkWizard.ts`

### T023: Add Interrupt/Retry Actions

- **Goal**: Control running runs.
- **Details**:
  - In `RunDetailPage`, add "Actions" dropdown or buttons.
  - **Stop**: `POST /episodes/{id}/interrupt` (payload: stop).
  - **Retry**: `POST /episodes/{id}/retry`.
  - Add confirmation dialogs (AlertDialog from Shadcn).
- **Files**:
  - `src/frontend/src/components/runs/RunActions.tsx`

### T024: New Engineer Run Form

- **Goal**: Kick off a solver agent.
- **Details**:
  - In `HomePage` (or dedicated "New Run" modal).
  - Select Benchmark (Dropdown).
  - Select Settings (optional).
  - Submit -> Redirect to new Run ID.
- **Files**:
  - `src/frontend/src/components/runs/NewRunDialog.tsx`

### T025: Final Polish

- **Goal**: UX consistency.
- **Details**:
  - Add `Sonner` (Toast) for success/error messages.
  - Add Error Boundaries for components (especially 3D viewer).
  - Ensure Empty States (No runs found) look good.
- **Files**:
  - `src/frontend/src/App.tsx` (Toaster)

## Implementation Guidelines

- **UX**: The wizard should feel guided. Disable "Next" buttons while loading.
- **Feedback**: Immediate feedback on actions via Toasts.

## Validation (Definition of Done)

- [ ] Wizard flows through all 4 steps securely.
- [ ] API calls are made at each step.
- [ ] "Stop" button sends the correct request.
- [ ] Toast notifications appear on success/error.
