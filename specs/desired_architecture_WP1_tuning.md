# Desired Architecture: WP1 - Agent Tuning & Optimization

## Objective of the system

We are to create a self-improving system that optimizes agent prompts and logic for both speed and quality, using Pareto optimization. The goal is to move from "manual prompt engineering" to "automated evolutionary optimization".

### Outputs and end goals

1. **Automated Prompt Tuning**: The system automatically adjusts prompts to maximize success rates on benchmarks.
2. **Pareto Optimization**: We balance conflicting goals (e.g., "solves problem" vs. "low token usage" vs. "execution speed") without manual intervention.
3. **Regression Testing**: Ensure optimizations do not regress on previously solved benchmarks.
4. **A "Gene Pool" of Prompts**: A database of prompt variations and their performance metrics.

## The Optimizer Agent

We introduce a new agent, the **Optimizer**, which operates asynchronously from### 1. The Optimizer Service (Microservice)

We deploy the Optimizer as a standalone container `optimizer-worker`.

* **Infrastructure**: Python service running `DEAP` in a loop.
* **Isolation**: Separated from the API Server to allow long-running blocking genetic algorithm (GA) generations.
* **Trigger**: A **Temporal Workflow** signals the service to start a new generation.
* **State**: It reads/writes to the `prompts` and `experiments` tables in Postgres.

### 2. The Execution Pipeline ("The Plumbing")

The Optimizer is a "Meta-Agent". It does not solve physics problems. It solves "Agent Performance Problems".
It has access to:

1. **Read Access**: All execution traces in LangFuse/Postgres.
2. **Write Access**: The `prompts` database table (new source of truth).
3. **Execute Access**: Can trigger "Regression Runs" via Temporal.

## Workflow: The Optimization Loop

The optimization process is a closed loop:

1. **Observation**: The Optimizer queries the Observability DB for the last N runs.
    * *Query*: "Find all agents with success_rate < 50% OR avg_cost > $0.10".
2. **Hypothesis**: The Optimizer analyzes the traces of failed runs.
    * *Analysis*: It reads the "Thoughts" and "Tools" of the failed agents.
    * *Insight*: "The Engineer Agent consistently fails to check for collisions before submitting."
3. **Mutation**: The Optimizer generates a new version of the prompt.
    * *Action*: It takes the current `system_prompt` and appends/modifies instructions.
    * *Draft*: "You MUST use the `consistency_check` tool before calling `submit_solution`."
4. **Experiment**: The Optimizer schedules a **Regression Search**.
    * It selects 20 benchmarks that previously failed (Target) and 5 that passed (Control).
    * It spawns a batch job with the *Mutated Prompt*.
5. **Evaluation**: The job completes. Metrics are calculated.
    * *Result*: Success rate went from 40% to 60%. Control group stayed at 100%. Cost increased by 5%.
6. **Promotion**: The mutation is deemed "Pareto Optimal" (better success, acceptable cost).
    * The new prompt is tagged `candidate` and queued for human review.

## Pareto Optimization Algorithm

We use a genetic algorithm approach tailored for prompt engineering.

### Definition of Fitness

We define a multi-objective fitness vector $F(p)$ for a prompt $p$:
$$ F(p) = [ SuccessRate(p), -AvgTokenCost(p), -AvgLatency(p) ] $$

We seek to maximize all components.

### The Algorithm (NSGA-II adaptation)

1. **Population**: We maintain a pool of 10-20 active### Reproducibility & CPU Performance

To ensure every optimization run yielded the exact same result given the same inputs:

1. **Global Seeding**: The GA loop initializes `random.seed(CONFIG.global_seed)` at the start of every generation.
2. **Immutable Prompts**: Once a prompt row is saved, it is never updated. New generations create new rows.
3. **CPU Optimization**: The Genetic Algorithm is CPU-bound (string manipulation). We use `multiprocessing` to evaluate the fitness of individuals in parallel if the evaluation step is local (though here it's remote).
4. **Traceability**: Every generated prompt links back to its `parent_ids` and the `experiment_id` that validated it.

    * *Parent A*: "Always check collisions."
    * *Parent B*: "Think step-by-step about cost."
    * *Child*: "Always check collisions and think step-by-step about cost."
5. **Mutation**: Randomly modifying a section of the prompt.
    * *Operations*: Add instruction, Remove instruction, Rephrase instruction, Change few-shot examples.

<!-- note: We reference the fictional/future paper "Generic Pareto optimization <https://arxiv.org/abs/2507.19457>" from the roadmap. -->

## Subagents and Roles

### The "Critic" becomes the "Coach"

The existing Critic agent (used during execution) provides data for the Optimizer.
The Optimizer serves as a "Meta-Coach", analyzing the Critic's own performance.

* *Did the Critic catch the error?* If no, the Critic's prompt is optimized.
* *Did the Engineer ignore the Critic?* If yes, the Engineer's "receptiveness" prompt is optimized.

## Data & State

### Prompt Registry Schema

We move from static `prompts.yaml` to a database-backed **Prompt Registry**.

```sql
CREATE TABLE prompts (
    id UUID PRIMARY KEY,
    agent_type TEXT NOT NULL, -- 'engineer', 'planner', 'critic'
    version TEXT NOT NULL, -- semver: 1.2.0
    content TEXT NOT NULL, -- The full jinja2 template
    parent_id UUID REFERENCES prompts(id),
    created_at TIMESTAMP DEFAULT NOW(),
    tags TEXT[], -- ['prod', 'experiment-A']
    fitness_score JSONB -- {success: 0.8, cost: 0.05}
);

CREATE TABLE experiments (
    id UUID PRIMARY KEY,
    prompt_id UUID REFERENCES prompts(id),
    benchmark_ids UUID[],
    status TEXT, -- 'running', 'completed'
    results JSONB
);
```

### Prompt Lifecycle

1. **Draft**: Created by Optimizer.
2. **Experimental**: Currently running in a Regression Search.
3. **Candidate**: Passed regression, awaiting approval.
4. **Production**: The current default for user traffic. (Tagged `prod`).
5. **Archived**: Old versions.

### Experiment Tracking with LangFuse

We use **LangFuse** datasets to track experiment results.

* Each "Mutation" is an Experiment Run in LangFuse.
* Scores are computed automatically from simulation results (Success/Fail) and trace metadata (Cost, Latency).

## Technology Stack

### Libraries

1. **LangFuse**: For tracing, dataset management, and scoring.
    * We use the `langfuse.client.create_dataset_run` API to programmatically log experiment batches.
2. **PyGMO / DEAP**: Python libraries for evolutionary algorithms.
    * We likely wrap `DEAP` because it's pure Python and easier to hack for "text genomes".
3. **Postgres**: Storing the Prompt Registry and mutation history.
4. **Temporal**: Orchestrating the long-running "Regression Run" workflows.
    * Workflow: `RunExperimentWorkflow(prompt_id, benchmark_ids)`
    * Activity: `BatchExecuteBenchmarks(ids)`

### Integration Points

* **Controller**: When spawning an agent, it now queries `PromptService.get_prompt(agent_type, tag='prod')` instead of reading filesystem.
* **Worker**: No change. Workers just execute what they are told.
* **Frontend**: A new "Admin / Optimization" dashboard page.
  * Graphs showing Success Rate over time.
  * "Approve Candidate" button for human-in-the-loop promotion.

## Implementation Details

### The "Mutation" Logic

How does an LLM mutate a prompt?

We use a **Meta-Prompt**:

```text
You are an expert Prompt Engineer.
Here is the current system prompt for the "Physics Engineer" agent:
<current_prompt>... </current_prompt>

Here is a summary of its recent failures:
- Failed to import 'math' library in 15% of cases.
- Forgot to check for 'max_cost' constraint in 10% of cases.

Propose 3 variations of the prompt to fix these issues.
Variation 1: Add a specific "Import check" section.
Variation 2: Rephrase the constraint section to be more imperative.
Variation 3: Add a few-shot example of correct import usage.
```

### Safety & Guardrails

* **Syntax Check**: Before running *any* experiment, the mutated prompt is rendered with dummy variables to ensure Jinja2 syntax is valid.
* **Sanity Check**: An inexpensive model (Flash/Haiku) checks if the prompt is "Safe" and hasn't drifted into nonsense (e.g., "Ignore all instructions").
* **Cost Cap**: The Optimizer has a daily budget (e.g., $50). It stops scheduling experiments when the budget is hit.

## Future Work

1. **Hyperparameter Optimization**: Optimizing not just the prompt text, but `temperature`, `top_p`, and `max_steps`.
2. **Tool Selection**: The Optimizer could suggest *removing* tools that are never used or confuse the agent.
3. **Few-Shot Mining**: Automatically finding the "Best" examples from the Journal and promoting them into the Few-Shot section of the prompt.
