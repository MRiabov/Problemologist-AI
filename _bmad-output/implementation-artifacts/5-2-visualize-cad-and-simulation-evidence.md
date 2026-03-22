# Story 5.2: Visualize CAD and Simulation Evidence

Status: ready-for-dev

## Story

As a human operator, I want to visualize CAD models, render evidence, and simulation playback so that I can verify the run visually rather than from text alone.

## Acceptance Criteria

1. Given a completed episode with CAD assets, when I open the viewer, then I can inspect the model and hide or isolate parts.
1. Given simulation renders or video exist, when I open the run, then I can play, pause, and scrub the evidence over time.
1. Given latest-revision media exists, when I inspect the run, then the UI binds the view to the latest revision rather than stale prior media.
1. Given a run has no preview media yet, when I open the viewport, then I see an explicit empty or rebuild state rather than a blank or misleading viewer.

## Tasks / Subtasks

- [ ] Bind episode assets in `UnifiedGeneratorView` to the latest media bundle and pass only the newest relevant render, video, and model assets into the viewport. (AC: 2, 3, 4)
  - [ ] Prefer controller-sorted episode assets over filename heuristics.
  - [ ] If latest-revision selection cannot be represented cleanly with the current payload, add the minimal typed field to `EpisodeResponse` / `AssetResponse` and regenerate the frontend client instead of parsing ad hoc JSON.
- [ ] Tighten `DesignViewer` and `ModelViewer` so the viewport can inspect the model, toggle topology visibility, and preserve loaded-model state while video playback and scrubbing are active. (AC: 1, 2)
  - [ ] Keep the existing topology browser, face / part / subassembly selection modes, and camera reset affordance.
  - [ ] Keep the simulation/video mode selection explicit and user-controlled.
  - [ ] Keep empty-state and rebuild affordances visible when model assets are absent.
- [ ] Keep the simulation evidence surfaces in sync with the selected media bundle so the viewport, heatmaps, and related render evidence reflect the same latest revision. (AC: 2, 3)
  - [ ] Reuse the existing `SimulationResults`, `CircuitSchematic`, and `WireView` renderers where they already fit.
  - [ ] Avoid a second visualization subsystem or any text-only fallback that pretends to show media.
- [ ] Add or extend live-browser integration coverage for topology inspection, hide/show behavior, simulation timeline playback, controller-proxied asset fetches, and latest-revision media binding. (AC: 1-4)
  - [ ] Anchor the regression coverage on `INT-165`, `INT-166`, `INT-167`, and `INT-174`.
  - [ ] Add a backend preview or media regression only if the frontend cannot verify latest-revision binding from persisted assets alone.

## Dev Notes

- `UnifiedGeneratorView` is the composition layer that resolves `selectedEpisode.assets` and feeds the viewport. Keep the asset resolution there instead of building a parallel media lookup in the viewer components.
- `DesignViewer` already exposes the required MVP modes: 3D, video, heatmaps, and electronics. The story should refine the binding to real episode assets, not add a new viewing paradigm.
- `ModelViewer` already owns topology search, hide/show, selection modes, and the play / pause / timeline controls. Preserve those controls and make them operate on the same latest asset set.
- The frontend dashboard is controller-first. Episode state and asset URLs must continue to come from controller APIs and the `/api/episodes/{id}/assets/{path}` proxy, not from worker filesystem reads.
- The UX spec says the default viewport should show the latest preview artifact, with simulation video taking precedence when present. Keep that contract explicit in code and tests.
- The viewer is inspection-only. Do not move CAD editing, simulation execution, or asset synthesis into the browser.
- If the asset payload does not cleanly indicate latest-revision media, add a minimal typed field to the controller schema and regenerate the frontend client instead of inferring from filenames or ad hoc JSON parsing.
- For empty runs or incomplete runs, the viewport should keep showing the explicit rebuild or empty state that the current UI already uses rather than a blank canvas.
- Media evidence rules still apply to reviewer roles in the architecture docs; this story should not weaken the controller-first or media-inspection contracts.

### Source Tree Components To Touch

- `frontend/src/components/workspace/UnifiedGeneratorView.tsx`
- `frontend/src/components/visualization/DesignViewer.tsx`
- `frontend/src/components/visualization/ModelViewer.tsx`
- `frontend/src/components/visualization/SimulationResults.tsx`
- `frontend/src/components/workspace/ArtifactView.tsx`
- `controller/api/schemas.py` only if a typed latest-media field is missing
- `frontend/src/api/generated/` if the schema changes
- `tests/integration/frontend/test_int_165.py`
- `tests/integration/frontend/test_int_166.py`
- `tests/integration/frontend/p0/test_frontend_p0.py`
- `tests/integration/frontend/p0/test_int_174.py`

### Testing Standards Summary

- Use integration tests only; avoid adding unit-test-only coverage for this story.
- Prefer live browser assertions plus live API assertions against `/episodes/{id}` and `/episodes/`.
- Keep assertions tied to persisted backend assets, render media, and loaded-model state.

### Project Structure Notes

- `UnifiedGeneratorView` is the shell-to-viewport bridge; `DesignViewer` is the mode switcher; `ModelViewer` is the 3D and topology control surface.
- Avoid duplicating preview summary state in the backend. If the UI needs a derived field, derive it from the persisted episode payload first.
- Preserve the route-stable shared shell so the visualization changes stay inside the workspace composition layer rather than introducing a parallel page.

### References

- [Source: _bmad-output/planning-artifacts/epics.md, Epic 5: UI, Visualization, and Demo, Story 5.2]
- [Source: _bmad-output/planning-artifacts/prd.md, preview and simulation evidence requirements, plus Phase 1 UI scope]
- [Source: _bmad-output/planning-artifacts/ux-design-specification.md, section 11 Preview And Visualization]
- [Source: specs/frontend-specs.md, CAD and simulation viewer requirements]
- [Source: docs/component-inventory.md, visualization component responsibilities]
- [Source: docs/backend-reference.md, validation preview split and latest revision evidence contract]
- [Source: controller/api/routes/episodes.py, episode asset ordering and controller-proxied asset fetch]
- [Source: controller/api/schemas.py, `EpisodeResponse` and `AssetResponse` typing]
- [Source: frontend/src/components/workspace/UnifiedGeneratorView.tsx, episode asset resolution and viewport composition]
- [Source: frontend/src/components/visualization/DesignViewer.tsx, view-mode switching]
- [Source: frontend/src/components/visualization/ModelViewer.tsx, topology browser and simulation controls]
- [Source: frontend/src/components/workspace/ArtifactView.tsx, persisted simulation evidence rendering]
- [Source: specs/architecture/CAD-and-other-infra.md, render artifacts and latest preview policy]
- [Source: specs/architecture/distributed-execution.md, controller-first asset proxy contract]
- [Source: specs/architecture/observability.md, render and media inspection, plus latest-revision media attribution]
- [Source: specs/integration-tests.md, INT-165, INT-166, INT-167, INT-174, and INT-188]

## Dev Agent Record

### Agent Model Used

TBD

### Debug Log References

### Completion Notes List

### File List
