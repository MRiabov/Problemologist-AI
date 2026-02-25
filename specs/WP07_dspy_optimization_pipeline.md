# WP07: DSPy Optimization & Reward Pipeline Integration

## Status

- **Current State**: DSPy "Scaffolding" is implemented. Signatures, `ReAct` programs, and a `WorkerInterpreter` exist.
- **Missing State**: No automated optimization (compilation), no unified reward metrics for LLM training, and no automated feedback loop from production traces to prompt improvement.

## Objective

To transform the current agent nodes from static prompt wrappers into self-improving programs that optimize their own instructions and few-shot examples based on simulation performance.

---

## Phase 1: Metric Unification (`dspy.metric`)

Currently, simulation metrics live in `worker/simulation/metrics.py` and are returned as `SimulationMetrics`. We must create a unified `cad_simulation_metric` function for DSPy that loads per-agent reward weights from `config/reward_config.yaml`.

### 1.1 Reward Design Principles

The metric must **never return a flat `0.0`** for partial failures. A flat zero creates a landscape the optimizer cannot climb — it cannot distinguish between "syntax error" and "5% over budget". Instead we use a **staged, additive reward** with continuous penalties:

1. **Simulation success is the dominant term** (weight 0.55–0.65 depending on agent). Everything else is a signal to help the optimizer find the path to simulation success.
2. **Validation gates give partial credit** — each gate (compiles, geometry valid, manufacturability, cost, weight) contributes a small additive score so the optimizer can rank partial progress.
3. **Cost and weight overages use a continuous penalty**, not a binary pass/fail:

   ```python
   score = max(0.0, 1.0 - max(0.0, actual / cap - 1.0))
   # 10% over cap → 0.90,  50% over → 0.50,  2× over → 0.0
   ```

4. **Partial simulation credit** uses distance-to-goal, discounted to incentivise full success:

   ```python
   # on failure:
   score = sim_weight * (1.0 - min_distance / initial_distance) * 0.4
   # on success:
   score = sim_weight  # full credit
   ```

5. **Minimum scores** (`minimum_score: 0.02`) distinguish "file missing" from "script has a syntax error".

All per-agent milestone weights and formulas live in **`config/reward_config.yaml`**. The metric reads this file at import time so weights can be tuned without touching code.

### 1.2 Bootstrap Threshold

Only traces scoring **≥ 0.75** are used as few-shot examples by `BootstrapFewShotWithRandomSearch`. This keeps examples high-quality (passed validation + near-successful simulation) while still allowing the optimizer to search over the full score range.
<!-- NOTE: we'll migrate to GEPA optimizer soon. Literally no reason to use BootstrapFewShotWithRandomSearch AFAIK. -->

```python
teleprompter = BootstrapFewShotWithRandomSearch(
    metric=cad_simulation_metric,
    metric_threshold=0.75,   # from reward_config.yaml bootstrap_threshold
    max_bootstrapped_demos=4,
    num_candidate_programs=10,
)
```

### 1.3 Implementation Requirements

- **Location**: `controller/agent/dspy_utils.py`
- **Config**: `config/reward_config.yaml` — per-agent milestone weights and penalty formulas.
- **Logic** (for `cad_engineer`, the primary agent; other agents follow the same pattern with their own weights):

```python
def cad_simulation_metric(gold: dspy.Example, prediction: dspy.Prediction, trace=None) -> float:
    cfg = load_reward_config().engineer.cad_engineer  # agent-specific weights
    score = 0.0

    # Gate 1 — script compiles
    if not prediction.script_compiled:
        return cfg.script_compiles.minimum_score  # 0.02, not 0.0
    score += cfg.script_compiles.weight           # 0.05

    # Gate 2 — CAD geometry valid
    if not prediction.cad_geometry_valid:
        return score
    score += cfg.cad_geometry_valid.weight        # 0.08

    # Gate 3 — manufacturability
    if prediction.manufacturability_valid:
        score += cfg.manufacturability_valid.weight  # 0.07

    # Gate 4 — build zone
    if prediction.parts_within_build_zone:
        score += cfg.parts_within_build_zone.weight  # 0.05

    # Gate 5 — cost (continuous penalty)
    cost_ratio = prediction.actual_cost / gold.objectives.max_unit_cost
    score += cfg.cost_within_cap.weight * max(0.0, 1.0 - max(0.0, cost_ratio - 1.0))

    # Gate 6 — weight (continuous penalty)
    weight_ratio = prediction.actual_weight / gold.objectives.max_weight
    score += cfg.weight_within_cap.weight * max(0.0, 1.0 - max(0.0, weight_ratio - 1.0))

    # Gate 7 — simulation (dominant term)
    sim_w = cfg.simulation_result.weight          # 0.60
    if prediction.simulation_success:
        score += sim_w
    elif prediction.simulation_ran:
        dist_score = 1.0 - prediction.min_distance_to_goal / prediction.initial_distance
        score += sim_w * max(0.0, dist_score) * 0.4  # discounted partial

    return min(score, 1.0)
```

- **Output**: A float in `[0.02, 1.0]` (never exactly `0.0`).

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

- **Initial**: `BootstrapFewShotWithRandomSearch`. It is robust for `ReAct` programs and generates few-shot examples where the agent successfully solved a hard task.
- **Advanced**: `MIPRO` (Multi-objective Instruction Proposal). This will optimize the actual "Instruction" text in our `Signatures` using a separate LLM.
<!-- why not GEPA? isn't it the best? it's basically for RL. Use GEPA. -->

### 3.2 Workflow Logic

1. **Initialize**: Load the uncompiled Program (e.g., `CoderNode`).
2. **Train/Val Split**: Split `evals/datasets` into training (10-20 items) and validation sets.
3. **Compile**: Run the Teleprompter against the `cad_simulation_metric` with `metric_threshold=0.75`.
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

- [ ] `config/reward_config.yaml` is defined with per-agent milestone weights summing to 1.0.
- [ ] `cad_simulation_metric` in `controller/agent/dspy_utils.py` loads weights from `reward_config.yaml` and returns scores in `[0.02, 1.0]` (never exactly `0.0`).
- [ ] A script `scripts/optimize_agents.py` can be run to trigger a DSPy compilation run.
- [ ] The `BenchmarkCoder` achieves >80% pass rate on `evals/datasets/benchmark_coder.json` after compilation.
- [ ] Compiled prompts are automatically used by the `controller` if available.
