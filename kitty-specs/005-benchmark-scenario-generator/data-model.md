# Data Model: Benchmark Scenario Generator

## 1. Entities (Database)

### Scenario Record

Benchmarks are persisted in the central database using the following schema:

| Field | Description |
|-------|-------------|
| `id` | Unique scenario identifier |
| `tier` | kinematic, spatial, etc. |
| `description` | Natural language goal |
| `script_content` | The generated Python code |
| `target_quantity` | Production volume for cost calc |
| `max_unit_cost` | Budget constraint |
| `cost_record` | Lowest unit cost achieved so far |
| `mjcf_path` | Path to generated MuJoCo XML |
| `seed_range` | Randomization bounds |
| `validation_status`| Pass/Fail from Verification Pipeline |

### GenerationRequest

Input configuration for the generator agent.

```python
class GenerationRequest(TypedDict):
    prompt: str          # e.g., "Create a gravity trap"
    tier: int            # 1, 2, or 3
    count: int           # Number of variations to generate
    output_dir: str      # Target directory
```

### ValidationReport

Output from the `validator.py` stability check.

```python
class ValidationReport(TypedDict):
    is_valid: bool
    error_message: Optional[str]
    sim_duration: float  # Time simulated (target 1.0s)
    max_energy: float    # Kinetic energy check
```
