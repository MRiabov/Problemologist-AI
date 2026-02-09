# Architecture Delta: Feb 9 Round 1

**Date**: 2026-02-09  
**Status**: Review of current implementation against `desired_architecture.md` and previous review rounds.

---

## 1. Motor Controller Functions *(MISSING - NEW)*

The recently added motor controller specifications are not yet implemented.

| Component | Description | Missing |
|-----------|-------------|---------|
| **Time-based** | `constant`, `sinusoidal`, `square`, `trapezoidal` functions | Entire `controllers` package |
| **Position-based** | Inverse kinematics (rotate to 45deg, etc.) | Implementation & logic |

> [!IMPORTANT]
> A new utility package `worker/utils/controllers` (or similar) is required to house these functions.

---

## 2. Agent Flow & Contracts

Handover validation is largely in place, but some refinements are needed.

- **[MISSING]** Engineering `plan.md` and `todo.md` templates are missing from the `engineer/` template repo.
- **[PARTIAL]** `objectives.yaml` is persisted during handover, but not all fields are utilized by subsequent tools.

---

## 3. Frontend Specifications

The design and 3-column layout are completed and look premium.

- **[MISSING]** **CAD Viewer Format**: Still uses **STL (client-side)**. Needs to switch to **GLB (server-side rendering fallback)** as per spec.
- **[MISSING]** **Artifact Viewer**: Full file tree for browsing artifacts in the 3rd column.

---

## 4. Infrastructure

- **[MISSING]** **Repo Structure**: Separate `pyproject.toml` and `uv.lock` for `worker/` and `controller/`.
- **[DONE]** **Git Library**: Usage of `GitPython`. (Note: Transition to git2 is cancelled, GitPython is fine).
- **[PARTIAL]** **Type Safety**: Beartype usage is sparse despite the mandate.

---

## 5. Agent Tools & Validation

- **[MISSING]** **`build_zone` Integration**: The main `validate` tool in `worker/utils/validation.py` uses a generic `MAX_SIZE` instead of the specific `build_zone` bounds from `objectives.yaml`.
- **[DONE]** `preview_design` tool and `/lint` endpoint are implemented.

---

## Suggested Next Steps

1. **Implement `controllers` package** with time and position-based functions.
2. **Refactor CAD Viewer** to support GLB and server-side preview integration.
3. **Split repository structure** into distinct service packages with separate dependency management.
4. **Update `engineer/` templates** to include structured `plan.md` and `todo.md`.
