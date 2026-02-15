# Research: Steerability & Interaction Logic

## Decision: Worker-Side Headless Isometric Rendering
- **Choice**: Use a headless `pyvista` or `VTK` renderer on the worker (integrated with `build123d` export) to generate snapshots.
- **Rationale**: Robustness. Ensures that the highlight and angle exactly match the agent's internal coordinate system. The frontend's camera might be in an arbitrary state; forcing one of 8 standard isometric views (e.g., [1,1,1], [-1,1,1], etc.) provides stable spatial grounding for VLMs.
- **Implementation**: The `SimulationBuilder` will accept a list of `highlight_ids`. It will render the scene with these features colored/brightened and return a URL to the S3-stored PNG.

## Decision: Transient Async Queue for Interaction
- **Choice**: `collections.deque` or `asyncio.Queue` held in the Controller's session manager.
- **Rationale**: The user suggested avoiding Redis/Postgres for a queue that only lasts 10-20 seconds. An in-memory queue is fastest and sufficient for bridging "Human -> Agent" turns.
- **Persistence**: If the controller restarts, active queues are lost, which is acceptable for transient steerability feedback (user can just resend).

## Decision: Topological Naming Stability
- **Decision**: Accept indices as "hints" but force the agent to use `find_closest` or semantic filtering for actual code generation.
- **Rationale**: `build123d` indices can change if the model is modified. By providing the center and normal of the selected face, the agent can generate `faces().sort_by(lambda f: (f.center() - Vector(1,2,3)).length)[0]` which is much more stable than `faces()[12]`.

## Technical Strategy: Best-Angle Snapshot Logic
To determine the optimal isometric view:
1. Extract the `normal` vector of the selected face.
2. Find the isometric unit vector (from the set of 8 standard corners) that has the largest dot product with the normal.
3. Position the camera along that vector, offset by the part's bounding box radius, looking at the feature's center.
4. Render using a `OffscreenRenderer` in the Worker activity.

## Technical Strategy: LangGraph Queue Integration
To support "graceful intervention":
1. Add a `SteeringQueue` middleware to the LangGraph execution loop.
2. At the exit of every tool-calling node, the graph will check the `asyncio.Queue` for the current `session_id`.
3. If a prompt is found, it is merged into the current state as a `HumanMessage`, and the graph branches to the "Planner" node instead of the "Next Step" node, effectively overriding the agent's current trajectory.

## Technical Strategy: Topological Inspector Tool
A new worker-side tool `inspect_topology` will:
1. Load the assembly in a "headless" state.
2. Accept a `target_id` (e.g. `face_12`).
3. Return a JSON object containing the feature's geometric properties (center, normal, area, vertex count).
4. This allows the agent to "see" the feature's characteristics to write a robust semantic selector.

## Decision: @-Mention Autocomplete
- **Choice**: Frontend-cached BOM tree.
- **Rationale**: Since the BOM (assembly tree) is already loaded in the CAD viewer, the frontend can perform local filtering for autocomplete, making the UX very snappy. The backend will perform the final validation/resolution during prompt enrichment.
