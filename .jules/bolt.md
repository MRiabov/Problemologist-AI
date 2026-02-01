## 2024-05-23 - [Mujoco Contact Loop Optimization]
**Learning:** In Mujoco simulations, `mujoco.mj_id2name` involves string lookups which are expensive inside the simulation loop (e.g. `check_termination` or observation generation).
**Action:** Pre-calculate IDs of interest (like forbidden geoms) and cache site names during initialization to use fast integer comparisons or dictionary lookups during the loop.
