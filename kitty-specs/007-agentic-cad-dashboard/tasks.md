# Tasks: Agentic CAD Dashboard

**Feature**: 007-agentic-cad-dashboard
**Status**: Planned

## Work Packages

### WP01: Foundation & Data Layer

- [x] T001: Install dependencies (streamlit, stpyvista, pyvista, sqlalchemy)
- [x] T002: Create directory structure (src/dashboard/...)
- [x] T003: Implement `src/dashboard/utils.py` (Path resolution)
- [ ] T004: Implement `src/dashboard/data.py` (SQLAlchemy models & queries)

*Dependency Setup*: None
*Parallelism*: Prerequisite for WP02/WP03.

### WP02: Dashboard Shell & Text Components

- [ ] T005: Create `src/dashboard/main.py` with basic layout (Sidebar/Main)
- [ ] T006: Implement `src/dashboard/components/chat.py` (CoT & Logs)
- [ ] T007: Implement `src/dashboard/components/code.py` (Syntax highlighting)
- [ ] T008: Implement `src/dashboard/components/sidebar.py` (Episode selection)

*Dependency Setup*: Depends on WP01
*Parallelism*: Can be done partly in parallel with WP03, but WP03 integrates here.

### WP03: 3D Visualization & Integration

- [ ] T009: Implement `src/dashboard/components/viewer_3d.py` (STL loading with PyVista)
- [ ] T010: Integrate 3D viewer into main layout
- [ ] T011: Implement "Live Mode" polling logic
- [ ] T012: Final Polish & Error Handling

*Dependency Setup*: Depends on WP01, WP02 (for integration)
*Parallelism*: Integration step requires WP02.
