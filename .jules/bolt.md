## 2024-05-23 - [Mujoco Contact Loop Optimization]
**Learning:** In Mujoco simulations, `mujoco.mj_id2name` involves string lookups which are expensive inside the simulation loop (e.g. `check_termination` or observation generation).
**Action:** Pre-calculate IDs of interest (like forbidden geoms) and cache site names during initialization to use fast integer comparisons or dictionary lookups during the loop.

## 2024-05-24 - [MuJoCo ID Lookup Caching]
**Learning:** `mujoco.mj_name2id` calls from Python bindings incur significant overhead (~30x vs dict lookup) even for small models.
**Action:** Always cache ID lookups for bodies, sites, actuators, etc. when accessing them in the simulation loop.
