# Frontend Implementation Tasks

This document tracks the missing frontend features as specified in [frontend-specs.md](file:///home/maksym/Work/proj/Problemologist/Problemologist-AI/kitty-specs/frontend-specs.md).

## 1. Layout & Panels

- [x] Implement 3rd column: Session History Sidebar (3/12 ratio).
- [x] Refactor `UnifiedGeneratorView` to 3:3:6 layout (History : Chat : Viewport/Filesystem).
- [x] Move `ArtifactView` (Filesystem) to be vertically split with `ModelViewer` in the rightmost column.
- [x] Ensure all panels are resizable and persist their state.

## 2. Chat UI & Agent Interaction

- [x] **Reasoning Traces**: Hide by default. Add expandable accordion/collapsible for each reasoning step.
- [ ] **Tool Call Details**:
  - [ ] Render actual `git diff` for `write_file` and `replace_file_content` instead of just icons.
  - [ ] Improved icons for all agent tool calls (using VSCode-style icons).
- [x] **Interrupt Button**: Move the Stop button to supersede the "Send" button during generation.
- [x] **Steerability / Context Selection**:
  - [x] Create Context Cards above the input area.
  - [x] Implement "Add to Context" for clicked parts (CAD) and lines (Code).
  - [x] Add "Clear/Remove" button (cross icon) to context cards.

## 3. CAD & Simulation Viewer (YACV)

- [x] **Topology View**: Implement a panel to show/hide parts and subassemblies.
- [x] **Simulation Controls**:
  - [x] Add a playback controller (Play/Pause, Seek slider, Rewind, Fast Forward).
  - [x] Display time-progressive simulation metrics (force, stress, etc. if available).
- [ ] **Selection**: Implement selection modes (Part, Primitive, Subassembly).
- [ ] **Deformable Meshes**: Ensure Three.js viewer supports time-progressive mesh deformation.

## 4. Code Viewer (ArtifactView)

- [x] Add line numbers to the syntax highlighter.
- [x] Implement line-level "Add to context" on click.

## 5. Electronics & Circuits

- [ ] Integrate `tscircuit` for schematic and PCB rendering.
- [ ] Enable context selection for circuit components.

## 6. Feedback System

- [ ] Add Thumbs Up/Down interaction after the agent finishes its task.
- [ ] Implement feedback modal (Recall rating, Text explanation, Categorized issues).

## 7. Plan Approval

- [ ] Add explicit Confirmation button at the bottom of the chat interface and top of the file explorer for plans.
