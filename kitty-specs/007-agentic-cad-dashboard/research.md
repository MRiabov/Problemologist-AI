# Research: Agentic CAD Dashboard

## Dashboard Role

The dashboard is the primary human-in-the-loop (HITL) interface for monitoring and debugging long-running agents.

### 1. Unified Monitoring

- **Decision**: Centralized view of Controller and Worker logs.
- **Rationale**: Agents are distributed; the dashboard must aggregate `structlog` events and LangChain traces into a single timeline.
- **Human-in-the-loop**: Support for interrupting agents or approving plans (HITL) as natively supported by `deepagents`.

### 2. Frontend Architecture: Vite + React

- **Decision**: Modern SPA architecture with `@react-three/fiber`.
- **Rationale**: High-performance 3D visualization is required to inspect meshes and simulation results. React allows for modular UI components (Chat, Telemetry, Viewport).

### 3. Backend: FastAPI (Controller)

- **Decision**: FastAPI as the business logic and API layer.
- **Rationale**:
  - Shared Pydantic models with the agents/workers.
  - Streaming updates via WebSockets or SSE for live reasoning traces.
  - Integration with Postgres and LangFuse.

### 4. 3D Visualization Strategy

- **Decision**: Browser-side rendering of S3-hosted mesh assets.
- **Rationale**:
  - Workers upload STL/OBJ meshes to S3.
  - The dashboard fetches these directly and renders them using Three.js.
  - Highlights manufacturability violations in the 3D viewport using face indices.
