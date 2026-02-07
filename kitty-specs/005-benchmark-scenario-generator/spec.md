---
feature: 005-benchmark-scenario-generator
status: draft
mission: software-dev
---

# Specification: Benchmark Scenario Generator

The **Benchmark Scenario Generator** is an autonomous agentic pipeline that creates the training and evaluation dataset for the Engineer Agent. It operates as a specialized design-time agent that generates randomized, physics-verified CAD scenarios (Benchmarks).

## 1. Introduction

The goal is to create a benchmark and a training dataset for evaluating LLM models on creating and solving dynamics problems. The generator is responsible for creating these problems, verifying them for validity, and persisting them as assets for the solver to tackle.

## 2. Goals & Success Criteria

### 2.1. Primary Goals

1.  **Generate randomized benchmarks**: Create parameterizable Python scripts that produce `build123d` geometry and MuJoCo XML (MJCF).
2.  **Verify Validity**: Ensure every generated benchmark is geometrically valid (no intersections, in bounds) and physically stable (doesn't explode in MuJoCo).
3.  **Reasoning Traces**: Capture the logic and struggle of the generator agent to help in future training and skill creation.
4.  **Skills Acquisition**: Proactively identify and persist new skills discovered during the generation process.

### 2.2. Success Criteria

-   **Validity**: 100% of Promoted benchmarks pass the `simulate()` stability check.
-   **Distribution**: Randomization covers variations in workspace size, object positions, and scales.
-   **Review**: Includes a Reviewer agent loop that inspects renders and simulation results before acceptance.

## 3. Functional Requirements

### 3.1. Agent Graph

We use LangChain and LangGraph for the agentic infrastructure, specifically leveraging the `deepagents` abstraction.

#### 3.1.1. Roles

1.  **Planner**: Responsible for architecting the generation task, creating and persisting a TODO list.
2.  **Coder (Engineer)**: Implements the `build123d` script to create the environment and objects.
3.  **Reviewer (Critic)**: Assesses the simulation stability, validity, and visual correctness.

#### 3.1.2. Agent Environment & Filesystem

Agents "live" directly in the filesystem of their container. We use `FilesystemMiddleware` for `ls`, `read`, `write`, `edit`, and `execute`.

**Structure**:
```text
.
├── skills/                     # [Read-Only] Learned skills and documentation
├── utils/                      # [Read-Only] Fixed utilities and shared code
├── renders/                    # [Read-Only/tool-generated] Media outputs (S3)
│   ├── images/                 # Images from 24 angles.
│   └── videos/                 # Video of the simulation.
├── reviews/                    # [Read-Write] Reviews from the critic.
├── journal.md                  # [Read-Write] Episodic Memory (Intent, Result, Reflection)
├── todo.md                     # [Read-Write] Execution plan from Planner
├── plan.md                     # [Read-Only] High-level plan
└── script.py                   # [Read-Write] Main execution script
```

### 3.2. Agent Artifacts

#### 3.2.1. Journal (Episodic Memory)
A structured Markdown log where the agent records a high-level narrative. Used for debugging, training, and skill creation.
Delimited with markdown headings for easier disclosure.

#### 3.2.2. TODOs
Managed via `TodoListMiddleware`. The Planner builds this list; the Critic verifies completion against it.

#### 3.2.3. Reviews
Stored in the `/reviews/` folder. Includes YAML frontmatter for strict typing (e.g., `decision: approved`, `comments: []`).

### 3.3. Skills

-   **Repo-level skills**: Stored in `.agent/skills/`.
-   **Learned skills**: Created/updated by an async **Learner Agent** based on patterns observed in the Journal.
-   **Skill format**: `SKILL.md` as per Anthropic standards.

### 3.4. Tools (Python Utils)

Imported and used directly within the agent's code:

-   **`validate(Compound) -> bool`**: Checks for intersections, out-of-bounds, and validity across all randomizations.
-   **`simulate(Compound) -> SimulationResult`**: Runs physics simulation, produces videos/renders, and reports stability.
-   **`submit_for_review(Compound)`**: Hands over the result to the Reviewer.
-   **`get_docs_for(type)`**: Invokes a documentation subagent.

### 3.5. Simulation & Success

Benchmarks consist of:
1.  **Build zone**: Where the solver can create parts.
2.  **Goal zone**: Target area for the moved object.
3.  **Moved object**: The object to be manipulated.
4.  **Forbid zone**: Forbidden area.

Failure occurs on timeout, out-of-bounds, or simulation instability.

## 4. Technical Design

### 4.1. Distributed Execution
-   **Controller Node**: Runs the LLM and orchestrates via LangGraph.
-   **Worker Node**: Runs simulations and python scripts in a Podman container.
-   **Temporal**: Orchestrates long-running tasks and handles retries/persistence.

### 4.2. Observability
-   **Langfuse**: Tracks all LLM traces and tool calls.
-   **Database**: Postgres for Temporal and Langfuse; metadata and asset tracking.

### 4.3. Assets
Final scripts, MJCFs, and renders are stored in S3 (Railway buckets) and tracked in the main DB.

## 5. Implementation Roadmap

1.  **Foundation**: DB models, `deepagents` graph scaffolding, and worker container setup.
2.  **Utilities**: `simulate` and `validate` logic on the worker.
3.  **Agent Logic**: Planner, Coder, and Reviewer prompts and state transitions.
4.  **Observability & Skills**: Langfuse integration and Learner agent node.
5.  **CLI & Batch**: Tools for running generation at scale.