# Implementation Plan: Advanced Manufacturing Workbenches

*Path: kitty-specs/004-advanced-manufacturing-workbenches/plan.md*

**Branch**: `004-advanced-manufacturing-workbenches` | **Date**: 2026-02-05 | **Spec**: [spec.md](spec.md)
**Input**: Feature specification from `/kitty-specs/004-advanced-manufacturing-workbenches/spec.md`

## Summary

Implement the **CNC** and **Injection Molding** workbenches as a Python library (`src/worker/utils/workbenches`). This library will be exposed to agents via the `validate_and_price` utility function. It utilizes `trimesh` for advanced geometric queries (draft analysis, undercuts) and `build123d` for topology traversal.

## Technical Context

**Language/Version**: Python 3.10+
**Dependencies**:

- `build123d`: Geometry.
- `trimesh`: Raycasting/Draft analysis.
- `numpy`: Math.

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

[No conflicts. Aligned with Utility-based architecture.]

## Project Structure

### Documentation

```
kitty-specs/004-advanced-manufacturing-workbenches/
├── plan.md              # This file
├── research.md          # Research
├── data-model.md        # Cost Models
└── contracts/           # API
```

### Source Code

```text
src/worker/utils/
├── workbenches/
│   ├── __init__.py      # Facade (validate_and_price)
│   ├── cnc.py           # CNC Logic
│   ├── injection.py     # IM Logic
│   └── common.py        # Shared Geometry Checks
```

## Complexity Tracking

| Violation | Why Needed | Simpler Alternative Rejected Because |
|-----------|------------|-------------------------------------|
| `trimesh` | Raycasting for undercuts. | `build123d`/OCP is too slow/complex for robust undercut detection. |
| Utils Pattern | Agent integration. | Agents write code using imports; a Tool-only interface restricts their ability to iterate quickly within a script. |
