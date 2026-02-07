# Quickstart: VLM CAD Agent (Engineer)

## Running an Engineering Episode

Engineering episodes are typically triggered through the Dashboard or the Controller API.

```bash
# Trigger an episode via the Controller API
curl -X POST http://controller:8000/api/episodes \
     -H "Content-Type: application/json" \
     -d '{
       "agent_type": "engineer",
       "goal": "Design a support bracket for a 5kg load that fits in a 10cm cube.",
       "constraints": {"max_cost": 50.0, "max_weight": 0.5}
     }'
```

## Inspecting Agent Progress

During execution, the agent's internal state is reflected in the worker's filesystem.

1. **Journal**: View the high-level reasoning.

   ```bash
   # From the controller/dashboard
   deepagents cat journal.md --episode-id <ID>
   ```

2. **TODOs**: Check the current status of tasks.

   ```bash
   deepagents cat todo.md --episode-id <ID>
   ```

## Local Development (Simulated Worker)

To run the agent graph locally without a full distributed setup:

```python
from controller.agent.engineer_graph import create_engineer_graph
from deepagents.backends import LocalFilesystemBackend

# Initialize with local sandbox for rapid testing
graph = create_engineer_graph(
    filesystem=LocalFilesystemBackend(root_dir="./sandbox"),
    llm_provider="openai/gpt-4o"
)

# Run a thread
graph.invoke({"goal": "Create a 10mm bolt."})
```

## Observability

- **LangFuse**: Monitor the "think" steps and tool call latencies.
- **Journal Snapshots**: View the evolving `journal.md` narrative in the Dashboard.
