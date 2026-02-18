# WP07: DSPy Optimization & Reward Pipeline Integration

## Status

- **Current State**: DSPy "Scaffolding" is implemented. Signatures, `CodeAct` programs, and a `WorkerInterpreter` exist.
- **Missing State**: No automated optimization (compilation), no unified reward metrics for LLM training, and no automated feedback loop from production traces to prompt improvement.

## Objective

To transform the current agent nodes from static prompt wrappers into self-improving programs that optimize their own instructions and few-shot examples based on simulation performance.

---

## Phase 1: Metric Unification (`dspy.metric`)

Currently, simulation metrics live in `worker/simulation/metrics.py` and are returned as `SimulationMetrics`. We must create a unified `CADMetric` function for DSPy.

### 1.1 Implementation Requirements

- **Location**: `controller/agent/dspy_utils.py`
- **Logic**:
  - **Hard Constraints (Binary)**: Geometric validity (no self-intersections), script compilation (no syntax errors), and physics stability. If these fail, score = `0.0`.
  - **Soft Constraints (Float)**:
    - **Performance**: Does the design meet the `target_quantity` or `max_unit_cost` defined in `objectives.yaml`?
    - **Efficiency**: Stress-to-weight ratio (Structural efficiency).
    - **Electrical**: Continuity and power efficiency for electronics tasks.
- **Output**: A float between `0.0` and `1.0`.

```python
def cad_simulation_metric(gold, prediction, trace=None):
    # 1. Extract simulation result from prediction (must run simulation if not cached)
    # 2. Compare against gold.objectives
    # 3. Calculate penalty for cost/weight overages
    # 4. Return aggregate score
    ...
```

---

## Phase 2: Dataset Engineering

We have raw data in `evals/datasets/*.json`. These must be accessible as `dspy.Example` objects.

### 2.1 Loader Implementation

- Create `controller/agent/benchmark/data_loader.py`.
- Function `load_benchmark_dataset(agent_type: str) -> list[dspy.Example]`.
- Ensure examples include `inputs` (prompt, context) and `labels` (optimal `objectives.yaml` or a "Gold" `script.py` known to pass).

---

## Phase 3: The Optimization Pipeline (The "Optimizer")

We need a dedicated workflow (or CLI tool) to run the DSPy `Teleprompter`.

### 3.1 Optimizer Selection

- **Initial**: `BootstrapFewShotWithRandomSearch`. It is robust for `CodeAct` programs and generates few-shot examples where the agent successfully solved a hard task.
- **Advanced**: `MIPRO` (Multi-objective Instruction Proposal). This will optimize the actual "Instruction" text in our `Signatures` using a separate LLM.

### 3.2 Workflow Logic

1. **Initialize**: Load the uncompiled Program (e.g., `CoderNode`).
2. **Train/Val Split**: Split `evals/datasets` into training (10-20 items) and validation sets.
3. **Compile**: Run the Teleprompter against the `cad_simulation_metric`.
4. **Evaluate**: Compare the compiled program vs. the baseline using `dspy.Evaluate`.
5. **Export**: Save the "Compiled Program" (the optimized prompt + successful few-shot traces) to a JSON file.

---

## Phase 4: Production Wiring & Persistence

The `PromptManager` must be updated to handle compiled DSPy programs.

### 4.1 Prompt Versioning

- **Directory**: `.agent/prompts/compiled/`
- When a node initializes, it should check if a compiled version of its signature exists for the current model.
- `node.program.load("path/to/compiled_state.json")`.

### 4.2 Fallback Strategy

If no compiled state is found, the system defaults to the static YAML prompts in `config/prompts.yaml`.

---

## Phase 5: Closing the Loop (Production Feedback)

1. **Langfuse Integration**: When a user provides feedback (Thumbs Up/Down) or a simulation succeeds in the field, the `Episode` is marked as a "Candidate for Training."
2. **Automated Collection**: A daily cron job extracts "Successful" traces from the Observability DB.
3. **Continuous Compilation**: The Optimizer runs on the new dataset, proposing a "Prompt PR" if performance increases by >5% on the benchmark.

---

## Success Criteria

- [ ] `cad_simulation_metric` is defined and returns accurate 0.0-1.0 scores.
- [ ] A script `scripts/optimize_agents.py` can be run to trigger a DSPy compilation run.
- [ ] The `BenchmarkCoder` achieves >80% pass rate on `evals/datasets/benchmark_coder.json` after compilation.
- [ ] Compiled prompts are automatically used by the `controller` if available.
