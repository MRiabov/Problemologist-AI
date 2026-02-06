# Quickstart: Agentic CAD Environment

## Initializing an Agent Session

The environment is managed via the `deepagents` SDK. The Controller initializes the agent and injects the distributed worker's filesystem.

```python
from deepagents import AgentNode
from deepagents.middlewares import FilesystemMiddleware, TodoListMiddleware
from src.environment.worker_client import SandboxClient

# 1. Connect to the distributed Worker
worker = SandboxClient(api_key="...", endpoint="https://worker-node:8080")

# 2. Setup Agent with specific Middlewares
# This exposes ls, aread, awrite, aexecute to the agent
agent = AgentNode(
    name="engineer_1",
    middlewares=[
        FilesystemMiddleware(
            backend=worker.get_backend(),
            read_only_paths=["/skills", "/utils"]
        ),
        TodoListMiddleware(path="/todo.md")
    ]
)

# 3. Start Task (Orchestrated via Temporal)
# The agent will now 'live' in the worker's filesystem
```

## Agent Tool Usage (Example)

The agent naturally uses tools to interact with the environment.

```python
# Agent: "I need to check the template and write my solution."
await agent.tools.aread("/utils/template.py")
await agent.tools.awrite("/script.py", content="from utils import simulate...")
await agent.tools.aexecute("python /script.py")
```

## Domain Tools (Utils)

Within the `script.py`, the agent uses high-level Python utilities provided in the environment.

```python
from utils import simulate, validate_and_price
from build123d import Box

# Define solution
part = Box(10, 10, 10)

# Verify against constraints
price_info = validate_and_price(part)
print(f"Current unit cost: {price_info['unit_cost']}")

# Run simulation
result = simulate(part)
if result.success:
    print("Objective reached!")
```
