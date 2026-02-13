# Research: Utilizing Langfuse v3 to Minimize Model Confusion in Problemologist-AI

## Executive Summary

Model "confusion" in complex agentic systems like Problemologist-AI typically manifests as:

1. **Context Loss**: Forgetting previous instructions or constraints (e.g., ignoring `objectives.yaml`).
2. **Hallucination**: Inventing file contents or tool capabilities (e.g., using nonexistent `build123d` methods).
3. **Looping**: Repeating failed actions without learning.
4. **Drift**: Deviating from the high-level plan over long horizons.

Langfuse v3 provides specific features—**Tracing**, **Prompt Management**, **Evaluation**, and **Datasets**—that can be leveraged to diagnose and mitigate these issues, directly aligning with the `desired_architecture.md`.

## 1. Trace Everything: The Foundation of Debugging Confusion

To understand *why* a model is confused, you must see exactly what it saw and did.

### Strategy for Problemologist Architecture

- **Deep Tracing of Agent Flows**:
  - Use **Nested Spans** to mirror the agent hierarchy: `BenchmarkGenerator` -> `Planner` -> `CAD Engineer`.
  - Instrument the `deepagents` middleware. Every tool call (`read_file`, `write_file`, `run_command`) should be a child span of the agent's execution step.
  - **Capture Inputs/Outputs**:
    - When `read_file` is called, trace *exactly* what content was returned to the model. If the file was empty or truncated, the model's subsequent "confusion" is actually a correct response to bad input.
    - When `write_file` is called, trace the content *before* it hits the disk.
- **Link to Journals**:
  - The architecture specifies a "Journal" for episodic memory.
  - **Recommendation**: Push Journal entries as **Label** or **Metadata** on the trace. This allows you to correlate the model's internal monologue (Journal) with its external actions (Tool Calls).

### Implementation (Python SDK)

Use the `@observe()` decorator to trace functions. It supports Pydantic models for inputs/outputs, aligning with your coding standards.

```python
from langfuse.decorators import observe

@observe(name="Planner.create_plan")
def create_plan(objectives: Objectives) -> Plan:
    # ... logic ...
```

## 2. Prompt Management: Versioning Context to Reduce Ambiguity

"Confusion" often stems from ambiguous or suboptimal system prompts, especially with complex context like `SKILL.md` or `capabilities.md`.

### Strategy

- **Centralize Prompts**: Move hardcoded prompts (e.g., "You are an expert mechanical engineer...") and template strings into Langfuse Prompt Management.
- **Version Capabilities**:
  - Your `capabilities.md` defines what the agent *can* do. If the agent hallucinates tools, the prompt is likely unclear.
  - **Action**: Store `capabilities.md` content as a prompt variable or a managed prompt in Langfuse. When you update the capabilities, you create a new version.
  - **Rollback**: If a new prompt version causes regression (more confusion), you can instantly rollback without redeploying code.
- **A/B Testing**:
  - Run experiments: Does "Chain of Thought" (CoT) reduce confusion compared to "Direct Instruction" for the *Planner*?
  - Langfuse allows you to serve different prompt versions to different trace sessions and compare metrics.

## 3. Evaluation: Automated "Confusion" Detection

You have "Reviewer" and "Critic" agents. Langfuse **Evaluations** can automate parts of this critique to flag confusion *as it happens* or closer to real-time.

### Strategy

- **Model-based Evaluation (LLM-as-a-Judge)**:
  - Create an evaluator that checks for **Constraint Adherence**.
  - **Input**: The generated `plan.md`.
  - **Context**: The `objectives.yaml` (Gold Standard).
  - **Prompt**: "Does the plan in `plan.md` violate any constraints in `objectives.yaml`? Answer Yes/No."
  - **Trigger**: Run this evaluator asynchronously on every tracing completion of `Planner.create_plan`.
- **Visualizing Quality**:
  - If the "Confusion Score" (derived from evaluations) spikes, you know a recent change (code or prompt) degraded reliability.

## 4. Datasets: Turning Failure into Test Cases

When a model gets confused, it’s a learning opportunity.

### Strategy

- **Capture Failures**:
  - When you identify a trace where the model failed (e.g., tried to use a non-existent screw type), add that trace's input to a **Langfuse Dataset** named `complex-assembly-confusion`.
- **Regression Testing**:
  - Before deploying a new prompt version or agent logic, run it against this dataset.
  - Ensure the model *no longer* gets confused on these known hard cases.

## 5. Session Tracking for Long-Running Context

The architecture involves long-running agents that might "drift".

### Strategy

- **Session IDs**:
  - Pass a consistent `session_id` to all traces for a single overarching task (e.g., "Design a gearbox").
  - Langfuse aggregates traces by session. You can visualize the *entire* history.
  - **Drift Detection**: If the cost/token usage for a session explodes, it’s a strong signal the agent is looping (confused). Set alerts on **Session Cost** or **Trace Duration**.

## Integration Checklist for Problemologist

1. **Install SDK**: pip install `langfuse`
2. **Environment**: Set `LANGFUSE_PUBLIC_KEY`, `LANGFUSE_SECRET_KEY`, `LANGFUSE_HOST`.
3. **Middleware**: Add Langfuse tracing to `deepagents` or your custom agent loop.
    - Trace: `agent_step`, `tool_call`.
4. **Prompts**: Port `capabilities.md` and initial system prompts to Langfuse dashboard.
5. **Evaluators**: Define a simple "Refusal check" evaluator – if the model refuses a task, is it valid?
