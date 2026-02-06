# Quickstart: Agentic CAD Environment

## Initialization

The environment is managed via `deepagents` on the Controller, which delegates execution to Worker nodes.

```python
from deepagents.core import AgentNode
from deepagents.middlewares import FilesystemMiddleware, TodoListMiddleware
from src.environment.worker import WorkerClient

# 1. Setup Worker Client (connecting to distributed worker)
worker = WorkerClient(url="http://worker-service:8080")

# 2. Initialize Agent with Middlewares
agent = AgentNode(
    name="cad_engineer",
    middlewares=[
        FilesystemMiddleware(backend=worker.filesystem),
        TodoListMiddleware()
    ]
)

# 3. Running an Episode (via Temporal)
# Episodes are usually triggered via the Controller's API
```

## Worker-Side Execution (Internal)

The worker manages the `build123d` and `mujoco` context.

```python
# The agent writes to 'script.py' in the sandbox
# The environment then executes it on the worker:
result = worker.execute("python script.py")

print(f"Simulation Outcome: {result.stdout}")
```

## Running Verification

```bash
# Run the test suite against the distributed setup
pytest tests/integration/test_worker_controller.py
```
