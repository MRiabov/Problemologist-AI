# Data Model: Benchmark Scenario Generator

## 1. Entities

### ScenarioManifest

A JSON metadata file accompanying each generated benchmark.

```json
{
  "id": "tier2_lever_001",
  "tier": "kinematic",
  "description": "Class 1 lever with variable fulcrum",
  "script_path": "scripts/tier2_lever_001.py",
  "assets": {
    "mjcf": "assets/tier2_lever_001.xml",
    "meshes": ["assets/base.stl", "assets/arm.stl"]
  },
  "randomization": {
    "seed_range": [0, 9999],
    "parameters": ["fulcrum_pos", "load_mass"]
  },
  "validation": {
    "passed": true,
    "max_velocity": 0.05
  }
}
```

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
