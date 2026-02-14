# Quickstart: Observability System

## 1. Accessing LangFuse

Langfuse is used in its cloud-hosted version.

- **URL**: `https://cloud.langfuse.com`
- **Setup**: Export environment variables in the Controller (or set them in your `.env` file).

  ```bash
  export LANGFUSE_PUBLIC_KEY="..."
  export LANGFUSE_SECRET_KEY="..."
  export LANGFUSE_HOST="https://cloud.langfuse.com"
  ```

## 2. Viewing Structured Logs

Logs from all nodes are aggregated in the Dashboard or can be viewed via the CLI.

```bash
# View live logs from a specific episode
deepagents logs --episode-id <ID> --follow
```

## 3. Running Schema Checks

To verify API consistency between components:

```bash
# Run schemathesis against the local Controller API
uv run schemathesis run http://localhost:18000/openapi.json
```

## 4. Episode Reconstruction

To rebuild the environment as the agent saw it at step 5:

```bash
uv run python -m src.observability.reconstruct --episode <ID> --step 5
```
