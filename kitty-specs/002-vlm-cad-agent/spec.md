# Feature Specification: Engineer Agent

**Feature**: 002-engineer-agent
**Status**: Draft
**Mission**: software-dev

## 1. Overview

The **Engineer Agent** is the autonomous cognitive engine designed to solve benchmarks in the **Agentic CAD Environment** (Spec 001). It is a specialized graph-based system built using **LangGraph** on top of the **`deepagents`** framework.

The agent follows an **Architect → Engineer → Critic** workflow, leveraging a persistent **TODO list** and **Skill-based Memory**. It is designed for high-latency, high-quality reasoning, creating valid `build123d` scripts that are executed in the distributed Worker environment.

## 2. Goals & Success Criteria

### 2.1. Primary Goals

1. **Solve Mechanical Problems**: Autonomously generate CAD models that satisfy geometric and physics constraints.
2. **Long-Running Durability**: Support execution times of 10+ minutes, utilizing `deepagents` state persistence to handle interruptions.
3. **Skill Acquisition**: Learn from failures via a Sidecar Learner agent that updates the Read-Only skill library.
4. **Distributed Execution**: Generate code on the Controller, but execute it strictly on the Worker node via the 001 API.

### 2.2. Success Criteria

- **Success Rate**: Solves >50% of "Easy" benchmarks (moving object to goal).
- **Skill Usage**: Successfully retrieves and applies relevant skills (e.g., "how to make a gear") from the filesystem.
- **Cost/Manufacturability**: Designs pass the Critic's checks (Spec 004 Workbenches) before final submission.

## 3. User Stories

- **As an Agent**, I want to break down complex tasks into a TODO list so I don't lose track.
- **As an Agent**, I want to consult my "Journal" (Episodic Memory) to see what I tried before.
- **As an Agent**, I want to verify my design cheaply (validation/pricing) before running expensive simulations.

## 4. Functional Requirements

### 4.1. Cognitive Architecture (The Graph)

The agent is a StateGraph with the following nodes:

1. **Architect (Planner)**:
    - **Input**: User Request + Benchmarks.
    - **Action**: Decompiles requirements, checks Skills, writes `plan.md` and initial `todo.md`.
    - **Output**: State update.

2. **Engineer (Actor)**:
    - **Input**: Current TODO item.
    - **Action**: Writes Python code (`script.py`) using `view_file` (Skills) and `write_file`. Calls `run_command` to execute.
    - **Loop**: Iterates until the code runs and locally validates.

3. **Critic**:
    - **Input**: Simulation Result (from `simulate` util) + Workbench Report.
    - **Action**: Decides if the solution is robust and cost-effective.
    - **Output**:
        - **Approve**: Submits for Final Review.
        - **Reject**: Updates `journal.md` with failure reason, loops back to Architect or Engineer.

4. **Sidecar Learner (Async)**:
    - **Trigger**: End of Episode (Success or Failure).
    - **Action**: Analyzes `journal.md` and execution traces.
    - **Output**: Updates `skills/` (via a separate administrative process/PR, as skills are Read-Only for the runner).

### 4.2. Tool Interface

The agent sees a simplified "OS-like" interface provided by the Controller (Spec 001).

**Core Tools**:

- `ls`, `view_file`, `write_file`, `edit_file`, `wait`.

**Utils (Python Library)**:
The agent interacts with the domain *through code it writes*, leveraging the pre-installed `utils` library on the Worker:

- `from utils import validate_and_price`
- `from utils import simulate`
- `from utils import submit_for_review`

### 4.3. Memory Systems

1. **Short-term (Working Memory)**: LangGraph State (Messages, Scratchpad).
2. **Episodic (Journal)**: `journal.md` (Read-Write). The agent logs intent, result, reflection, and next steps. Stored in the **local, ephemeral sandbox**.
3. **Procedural (Skills)**: `skills/*.md` (Read-Only). How-to guides for `build123d`, `mujoco`, etc. Pulled from Git before execution.
4. **Planned (TODOs)**: `todo.md` (Read-Write). Stored in the **local, ephemeral sandbox**.

## 5. Technical Design

### 5.1. Tech Stack

- **Orchestrator**: LangGraph (Python).
- **Framework**: `deepagents`.
- **LLM**: Gemini 2.0 Pro / Claude 3.5 Sonnet.
- **State Store**: Postgres (via LangGraph checkpointing).

### 5.2. Component Structure

```text
src/agent/
├── graph.py            # LangGraph definition
├── nodes/              # Architect, Engineer, Critic
├── state.py            # AgentState (TypedDict)
└── prompt_manager.py   # System prompts & template rendering
```

## 6. Assumptions & Constraints

- **Code Execution**: The agent NEVER executes code on the Controller. It MUST use `write_file` + `run_command` which routes to the Worker.
- **Skill Updates**: The agent cannot self-modify skills during a run. Improvements are captured in Journal and processed asynchronously.
