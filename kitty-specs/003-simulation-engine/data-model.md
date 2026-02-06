# Data Model: MuJoCo Simulation Engine

## Core Models (Pydantic)

### 1. PhysicsObjective (AABB)

Definition of zones within the simulation environment.

```python
class PhysicsObjective(BaseModel):
    name: str
    objective_type: Literal["build", "goal", "forbid", "bounds"]
    min_point: Tuple[float, float, float]
    max_point: Tuple[float, float, float]
```

### 2. SimulationTask

The payload for a single simulation execution (orchestrated via Temporal).

```python
class SimulationTask(BaseModel):
    task_id: UUID
    mjcf_content: str
    duration: float = 10.0
    jitter_config: Optional[Dict[str, float]] = None
    render_video: bool = False
    callback_url: HttpUrl
```

### 3. SimulationTelemetry

Temporal data returned from the simulation.

```python
class SimulationTelemetry(BaseModel):
    timestamps: List[float]
    object_positions: Dict[str, List[Tuple[float, float, float]]]
    motor_power: List[float]
    contact_forces: List[float]
```

## Artifact Storage (S3 / Railway)

Artifacts are stored with the following structure to ensure consistency across worker nodes.

- **`rendering/video/<task_id>.mp4`**: The simulation recording.
- **`rendering/frames/<task_id>/<angle>.png`**: 24 multi-view renders.
- **`physics/mjcf/<task_id>.xml`**: The generated MJCF file.
- **`physics/meshes/<hash>.stl`**: Cached mesh files for reuse.

## Success Definition

1. **Safety**: No `forbid` zone contact && No `out_of_bounds` excursion.
2. **Achievement**: Goal object's center of mass stays within `goal` zone for >1.0s.
3. **Stability**: No simulation "explosions" (NaN velocities).
