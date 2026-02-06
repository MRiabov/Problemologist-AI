# Quickstart: VLM CAD Agent

## Running the Agent (Development Only)

For development, you can run the agent locally against a remote worker.

```bash
# 1. Export variables
export WORKER_URL="http://your-worker:8080"
export LITELLM_MODEL="openai/gpt-4o"

# 2. Run the runner script
python -m src.agent.runner --goal "Design a generic ball joint for MuJoCo"
```

## Production Interaction

In production, the agent is managed by the **Controller** and triggered via the dashboard or API.

```bash
# Example API call to start an agent episode
curl -X POST http://controller/api/episodes \
     -H "Content-Type: application/json" \
     -d '{"goal": "Create a compliant 1-DOF flexure", "spec_id": "002"}'
```

## Monitoring

- Follow traces in **LangFuse**.
- Check **Temporal** dashboard for simulation workflow status.
- Files generated (vidoes, meshes) are uploaded to **S3 (Railway Bucket)**.
