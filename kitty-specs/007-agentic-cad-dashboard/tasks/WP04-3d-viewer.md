---
work_package_id: "WP04"
title: "3D Asset Viewer"
lane: "planned"
dependencies: ["WP02"]
subtasks: ["T016", "T017", "T018", "T019", "T020"]
---

## Context

Users need to inspect generated CAD models (STL/GLTF/GLB) directly in the browser to verify the agent's output. We will use `react-three-fiber` (R3F) ecosystem.

## Objective

Implement a robust 3D viewer component capable of loading assets from URLs and providing basic inspection controls.

## Subtasks

### T016: R3F Setup

- **Goal**: Install and configure 3D libs.
- **Details**:
  - Install `three`, `@types/three`, `@react-three/fiber`, `@react-three/drei`.
  - Verify Canvas rendering.
- **Files**:
  - `src/frontend/package.json`

### T017: STL/GLTF Reader Component

- **Goal**: Load geometry from URL.
- **Details**:
  - Create generic `AssetLoader` component.
  - Detect file type by extension or mime type.
  - Use `useLoader(STLLoader, url)` or `useGLTF(url)`.
  - Handle loading states (Suspense/Spinner) and errors.
- **Files**:
  - `src/frontend/src/components/viewer/AssetLoader.tsx`

### T018: Build "3D Scene" Component

- **Goal**: The environment for the model.
- **Details**:
  - Component `CadViewerScene`.
  - Include:
    - `Canvas` (R3F).
    - `Stage` (from drei) for auto-centering and lighting.
    - `Grid` (optional reference).
    - `OrbitControls`.
- **Files**:
  - `src/frontend/src/components/viewer/CadViewerScene.tsx`

### T019: Integrate Viewer into Run Detail

- **Goal**: Place the viewer in the UI.
- **Details**:
  - In `RunDetailPage`, add a "View Artifacts" tab or panel.
  - Fetch the list of assets for the run via `useRuns` hook.
  - Allow user to select an asset to view.
- **Files**:
  - `src/frontend/src/pages/RunDetailPage.tsx`

### T020: Add View Controls

- **Goal**: User tools for inspection.
- **Details**:
  - Floating toolbar in the viewer.
  - Toggles:
    - Wireframe mode.
    - Auto-rotate.
    - Reset View (Camera).
    - Axes Helper toggle.
- **Files**:
  - `src/frontend/src/components/viewer/ViewerControls.tsx`

## Implementation Guidelines

- **Performance**: Dispose of geometries/materials when unmounting.
- **UX**: Show a clear loading spinner for large STLs.

## Validation (Definition of Done)

- [ ] Can load a sample .stl and .gltf/glb file from a public URL (test asset).
- [ ] Model is centered and lit correctly.
- [ ] Orbit controls work (rotate, zoom, pan).
- [ ] Wireframe toggle works.
