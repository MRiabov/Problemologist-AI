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

## Decision: @-Mention Autocomplete
- **Choice**: Frontend-cached BOM tree.
- **Rationale**: Since the BOM (assembly tree) is already loaded in the CAD viewer, the frontend can perform local filtering for autocomplete, making the UX very snappy. The backend will perform the final validation/resolution during prompt enrichment.
