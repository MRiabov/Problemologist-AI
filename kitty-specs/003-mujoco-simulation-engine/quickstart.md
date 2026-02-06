# Quickstart: MuJoCo Simulation Engine

## Running a Standalone Simulation

The simulation engine can be invoked directly for debugging or batch processing.

```python
from src.simulation.engine import MujocoEngine
from src.simulation.models import SimulationTask

# 1. Initialize Engine
engine = MujocoEngine()

# 2. Define Task
task = SimulationTask(
    mjcf_content=open("model.xml").read(),
    render_video=True
)

# 3. Execute
result = engine.run_task(task)
print(f"Status: {result.status}")
print(f"Video saved to: {result.video_url}")
```

## Visualizing Results

The engine outputs telemetry that can be plotted or viewed in the dashboard.

```python
import matplotlib.pyplot as plt

# Plot Z-height of the moved object
plt.plot(result.telemetry.timestamps, [p[2] for p in result.telemetry.object_positions['obj_1']])
plt.show()
```

## Integration with Worker

In the distributed setup, the worker node hosts the simulation engine and exposes it via an internal API used by the `simulate()` Python utility.

```bash
# Internal Worker API call
curl -X POST http://localhost:8080/simulate \
     -d '{"mjcf_url": "s3://bucket/model.xml"}'
```
