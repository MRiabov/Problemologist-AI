## 2024-05-23 - [Mujoco Contact Loop Optimization]
**Learning:** In Mujoco simulations, `mujoco.mj_id2name` involves string lookups which are expensive inside the simulation loop (e.g. `check_termination` or observation generation).
**Action:** Pre-calculate IDs of interest (like forbidden geoms) and cache site names during initialization to use fast integer comparisons or dictionary lookups during the loop.

## 2026-02-07 - [Python List vs C++ Export]
**Learning:** Iterating over `build123d.tessellate` results in Python is 5x slower than exporting to STL (C++) and reading back. Avoid pure Python loops for large geometric data.
**Action:** Use `build123d.export_stl` to bytes buffer and `trimesh.load(BytesIO)` to keep processing in optimized C++ paths while avoiding disk I/O redundancy.
