# Data Model: MuJoCo Simulation Engine

Since the service is stateless/ephemeral, this models the **API DTOs** (Data Transfer Objects) and internal objects, rather than a persistent schema.

## 1. API Objects

### 1.1. Simulation Request (`POST /simulate`)

| Field | Type | Description |
| :--- | :--- | :--- |
| `id` | `UUID` | Unique request ID (for correlation). |
| `env_geometry_step` | `Base64<STEP>` | The static environment (walls, floors, zones) in STEP format. Must preserve Solid names (e.g. `zone_goal`). |
| `agent_geometry_step` | `Base64<STEP>` | The agent's mechanism in STEP format. |
| `control_script` | `String` | Python code defining `def control_policy(obs): ...`. |
| `config` | `TaskConfig` | Simulation parameters. |

#### TaskConfig

| Field | Type | Default | Description |
| :--- | :--- | :--- | :--- |
| `duration` | `Float` | `10.0` | Max simulation time (seconds). |
| `timestep` | `Float` | `0.002` | Physics step size (seconds). |
| `seed` | `Int` | `42` | Random seed for physics noise. |
| `return_trace` | `Bool` | `False` | If true, return MJCF/Trace capability. |

### 1.2. Simulation Response

| Field | Type | Description |
| :--- | :--- | :--- |
| `request_id` | `UUID` | Echo ID. |
| `status` | `Enum` | `SUCCESS`, `FAIL_COLLISION`, `FAIL_TIMEOUT`, `SYSTEM_ERROR`. |
| `metrics` | `SimulationMetrics` | Performance data. |
| `error` | `String?` | Error message if system failure or code crash. |

#### SimulationMetrics

| Field | Type | Description |
| :--- | :--- | :--- |
| `completion_time` | `Float` | Time simulation ended (sim-time). |
| `energy_joules` | `Float` | Total work done by actuators. |
| `goal_distance_min`| `Float` | Closest distance to goal achieved. |

## 2. Internal Domains

### 2.1. Simulation Bundle (Worker Input)

Sent to the worker process.

```python
@dataclass
class SimulationBundle:
    scene_mjcf_path: Path  # Compiled XML file
    control_script_path: Path # Isolated script file
    config: TaskConfig
```

### 2.2. Zone Definition

Parsed from `env_geometry_step`.

| Zone Type | Naming Convention | Behavior |
| :--- | :--- | :--- |
| **GOAL** | `zone_goal_*` | Defines success volume. |
| **FORBID** | `zone_forbid_*` | Defines rapid failure volume. |
| **SPAWN** | `zone_start_agent` | Where to place Agent root. |
| **OBJECT** | `zone_start_object`| Where to place Target object. |
