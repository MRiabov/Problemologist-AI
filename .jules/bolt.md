## 2024-05-23 - [Mujoco Contact Loop Optimization]
**Learning:** In Mujoco simulations, `mujoco.mj_id2name` involves string lookups which are expensive inside the simulation loop (e.g. `check_termination` or observation generation).
**Action:** Pre-calculate IDs of interest (like forbidden geoms) and cache site names during initialization to use fast integer comparisons or dictionary lookups during the loop.

## 2024-05-24 - [MuJoCo ID Lookup Caching]
**Learning:** `mujoco.mj_name2id` calls from Python bindings incur significant overhead (~30x vs dict lookup) even for small models.
**Action:** Always cache ID lookups for bodies, sites, actuators, etc. when accessing them in the simulation loop.

## 2026-02-24 - [Genesis Collision Check Optimization]
**Learning:** `GenesisBackend.get_contact_forces` fetches contacts from the simulation engine (potentially transferring data from GPU/C++). In `check_collision` loops (called for every body against every forbidden zone), this method was called repeatedly within the same simulation step, leading to significant overhead (O(N*M) calls per step).
**Action:** Implemented caching for `get_contact_forces` scoped to the current simulation step (`_step_counter`). This reduces the number of engine calls to 1 per step, regardless of how many collision checks are performed. Verified with a synthetic benchmark showing ~20x speedup for 100 bodies/zones.
