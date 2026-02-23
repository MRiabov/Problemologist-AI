## 2024-05-23 - [Mujoco Contact Loop Optimization]
**Learning:** In Mujoco simulations, `mujoco.mj_id2name` involves string lookups which are expensive inside the simulation loop (e.g. `check_termination` or observation generation).
**Action:** Pre-calculate IDs of interest (like forbidden geoms) and cache site names during initialization to use fast integer comparisons or dictionary lookups during the loop.

## 2024-05-24 - [MuJoCo ID Lookup Caching]
**Learning:** `mujoco.mj_name2id` calls from Python bindings incur significant overhead (~30x vs dict lookup) even for small models.
**Action:** Always cache ID lookups for bodies, sites, actuators, etc. when accessing them in the simulation loop.

## 2026-02-23 - Simulation Loop Check Frequency
**Learning:** Checking for simulation failure (collision, motor overload) and collecting metrics at every single simulation step (2ms) is extremely expensive due to Python <-> C++ (MuJoCo/Genesis) overhead and O(N) loops. Most failure conditions (like overload or falling off world) don't need 500Hz resolution.
**Action:** Reduce check frequency to ~50Hz (every 10-20 steps) for massive speedups (3.8x observed). Ensure metrics that integrate over time (like energy) are scaled correctly to account for skipped samples.
