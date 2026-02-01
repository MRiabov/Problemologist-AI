# Quickstart: VLM CAD Agent

## Prerequisites

1. **Environment Setup**: Ensure Spec 001 (Agentic CAD Environment) is built and running.

   ```bash
   # From project root
   pip install -e .
   ```

2. **LLM Keys**: Set your API keys.

   ```bash
   export OPENAI_API_KEY=sk-...
   export GEMINI_API_KEY=...
   ```

## Running the Agent

### CLI Mode

Refine a specific problem description:

```bash
python -m src.agent.runner --goal "Create a 100mm cube with a 10mm hole in the center" --model "gpt-4o"
```

### Resume a Session

```bash
python -m src.agent.runner --resume "session-id-123"
```

## Logs

* **Real-time**: Check the rich terminal output.
* **Trace**: View `agent_trace.jsonl` for full step-by-step tool calls.
* **Memory**: `journal.md` will be updated after success/failure.
