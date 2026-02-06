# Data Model: MuJoCo Simulation Engine

## Core Models (Pydantic)

We use `pydantic.BaseModel` for API communication and internal orchestration via Temporal.

### 1. SimulationRequest

```python
class SimulationRequest(BaseModel):
    request_id: UUID = Field(default_factory=uuid4)
    env_geometry_url: str  # S3 URL to environment geometry
    agent_geometry_url: str # S3 URL to agent's code/model
    config: SimulationConfig
```

### 2. SimulationConfig

```python
class SimulationConfig(BaseModel):
    duration: float = 10.0
    timestep: float = 0.002
    seed: int = 42
    render_video: bool = False
    perturb_position: bool = True # For robustness testing
```

### 3. SimulationResult

```python
class SimulationResult(BaseModel):
    request_id: UUID
    status: Literal["success", "collision", "out_of_bounds", "timeout", "instability", "error"]
    metrics: Dict[str, float]
    video_url: Optional[str] = None # S3 URL if rendered
    summary: str # Text summary for agent feedback
```

## Internal Orchestration

The engine is stateless. Worker nodes pull assets from S3 based on the URLs in the `SimulationRequest`.

1. **Geometry Parsing**: The worker parses the environment from S3 to identify goals and forbidden zones based on naming conventions (`zone_goal_*`, `zone_forbid_*`).
2. **Execution**: Code is executed inside a **Podman/Docker** sandbox.
3. **Traceability**: All simulation results are reported back to the Controller and persisted in the observability DB.
