# Research: Agentic CAD Environment

## Decisions

### 1. Geometry Pipeline (CAD -> Physics)

**Decision**: Use `build123d` -> STL -> `trimesh` -> `mujoco` (MJCF).
**Rationale**:

- `build123d` exports high-quality STLs.
- `trimesh` is the standard library for mesh processing in Python. It can handle inertia calculation and convex hull generation which `mujoco` needs for stable collisions.
- For MVP, we will use **Convex Hulls** of individual solids. If a part has multiple solids, we represent them as a multi-body MJCF.
**Alternatives**:
- Direct Step-to-MJCF: Too complex to implement a custom mesher.
- `gmsh`: powerful but heavier dependency.

### 2. Sandbox Security

**Decision**: Local execution with restricted imports (if possible) or simply reliance on Docker container isolation.
**Rationale**: Implementing a robust python sandbox in pure python is non-trivial. Docker provides OS-level isolation which is sufficient for "System Damage" prevention.
**Alternatives**: `restrictedpython` (too restrictive for CAD), `pywasm` (too slow/complex).

### 3. Documentation Search

**Decision**: Grep-based search on a local directory of markdown files.
**Rationale**: Extremely fast, simple to implement, zero dependencies. User specified this.
