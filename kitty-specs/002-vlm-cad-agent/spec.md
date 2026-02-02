# Feature Specification: VLM CAD Agent

**Feature**: 002-vlm-cad-agent
**Status**: Draft
**Mission**: software-dev

## 1. Overview

The **VLM CAD Agent** is the autonomous cognitive engine designed to operate within the **Agentic CAD Environment** (Spec 001). It is a specialized Vision-Language Model (VLM) system built using the **DeepAgents** framework (based on LangGraph). It acts as a mechanical engineer, taking a natural language problem description and iteratively producing valid, functional `build123d` CAD scripts.

Unlike generic coding assistants, this agent leverages the "Deep Agent" architecture to perform long-horizon planning, sub-task delegation, and persistent file-system based memory management. It validates its own work using visual feedback (renders) and simulation results.

## 2. Goals & Success Criteria

### 2.1. Primary Goals

1. **Autonomous Resolution**: The agent must be able to solve geometric problems from start to finish without human intervention, handling its own errors and edge cases.
2. **Multimodal Reasoning**: Effectively use the `preview_design` tool to visually inspect geometry and correct spatial errors that are not visible in the code text.
3. **Structured Cognition**: Enforce a strict separation between **Planning** (understanding the problem, researching docs) and **Execution** (writing code, iterating).
4. **Economic Optimization**: Respect `max_unit_cost` and `target_quantity` constraints by iteratively optimizing material volume, part reuse, and manufacturing process selection.
5. **Budget-Aware Self-Correction**: When a submission is rejected due to cost overruns, the agent must distinguish between "Inefficient Design" (fixable) and "Physical Impossibility" (requires justification).
6. **Skill-Based Learning**: Implement a "Skill Population" system where the agent can record specific techniques (e.g., "Correct syntax for build123d Loft") into the `build123d_cad_drafting_skill` to avoid repeating mistakes across sessions.

### 2.2. Success Criteria

* **Solver Rate**: The agent successfully solves >50% of the "Easy" benchmark problems defined in Spec 001 on the first try.
* **Budget Adherence**: The agent meets or beats the `max_unit_cost` in >70% of successful completions.
* **Self-Healing**: The agent can recover from at least one syntax error and one geometric violation per session without crashing.
* **Economic Adaptability**: The agent demonstrates switching from CNC to Injection Molding when production volume increases (e.g., from 1 to 10,000 units).
* **Visual Utility**: In >30% of iterations, the agent explicitly modifies code after requesting a `preview_design`, indicating active use of visual feedback.
* **Skill Growth**: The `build123d_cad_drafting_skill` reference directory contains >5 new instruction files after 10 sessions of problem-solving.

## 3. User Stories

### 3.1. As an Operator/Researcher

* **As a Researcher**, I want to configure the agent to use different backend models (Gemini Pro, GPT-4o) so I can benchmark their spatial reasoning capabilities.
* **As an Operator**, I want to see a real-time structured log of the agent's "Thought," "Tool Call," and "Observation" so I can debug its reasoning process.
* **As an Operator**, I want the agent to automatically save its successful strategies to a `knowledge.md` file so it gets smarter over time.

### 3.2. As the Agent (Internal Monologue)

* **As the Agent**, I want to look up `build123d` documentation before writing complex features to ensure I use the correct API signature.
* **As the Agent**, I want to render a low-res preview of my part to check if the holes are aligned before submitting the final expensive physics simulation.
* **As the Agent**, I want to read my past notes on "Press Fit Tolerances" before designing a connector.

## 4. Functional Requirements

### 4.1. Cognitive Architecture (DeepAgents Graph)

The agent shall be implemented as a **LangGraph** state machine with the following nodes:

1. **Planner Node**:
    * **Role**: Analyzes the request and `journal.md`.
    * **Action**: Generates a structured plan in `plan.md`.
    * **Transition**: -> `Actor`.

2. **Actor Node** (The Builder):
    * **Role**: Executes the current step of the plan.
    * **Action**: Calls tools (`write_script`, `preview_design`).
    * **Transition**: -> `Critic` (if submission or preview) OR -> `Actor` (if continuing).

3. **Critic Node** (The Validator):
    * **Role**: Visual, Logic, and Economic validation.
    * **Trigger**: After `submit_design` or `preview_design`.
    * **Action**:
        - Checks simulator feedback or vision output.
        - **Cost Guard**: Compares current unit cost against budget.
        - **Consensus**: Evaluates `force_submit` justifications. If valid, signals `HARD_LIMIT_REACHED` to Planner.
    * **Transition**:
        - **Success**: -> `Skill Populator` -> `End`.
        - **Failure/Cost Overrun**: -> `Planner` (Re-planning) -> `Actor`.

4. **Skill Populator Node**:
    * **Role**: Procedural memory management.
    * **Action**: Updates the `build123d_cad_drafting_skill` with successful patterns, lessons learned, or budget optimization strategies.

### 4.2. Tool Interface

The agent interacts with the environment via **LangChain Tools** (wrapping Spec 001 JSON schema).

#### 4.2.1. Environment Tools (Wrapped from Spec 001)

* `search_docs(query: str)`
* `write_script(content: str, path: str)`
* `edit_script(find: str, replace: str, path: str)`
* `preview_design()` -> Returns schema with `image_url` (or base64).
* `submit_design(control_path: str)` -> Returns evaluation report.

#### 4.2.2. Meta-Cognitive Tools

* `read_journal(topic: str)`: Fuzzy search or full read of the simplified `journal.md` memory file.
* `write_journal(entry: str, tags: list[str])`: Append a learned lesson to the long-term memory.
* `update_plan(status: str, notes: str)`: Update the current session's scratchpad.

### 4.3. LLM Integration

* **Framework**: `langchain-core` / `langchain-anthropic` / `langchain-google-genai`.
* **Context Management**: Handles by `langgraph` checkpoints and `deepagents` file-system memory.
* **Image Handling**: Native multimodal support via LangChain's message content types.

### 4.4. Observability & Logging

* **LangSmith**: Primary observability platform (if key provided).
* **Console**: Rich output via `rich`, streaming graph events.

## 5. Technical Design

### 5.1. Tech Stack

* **Language**: Python 3.10+
* **Framework**: `deepagents` (presumed library name) / `langgraph` / `langchain`.
* **Models**: `langchain-openai`, `langchain-google-genai`.
* **Schema Validation**: `Pydantic` (V2).

### 5.2. Component Structure

```
src/agent/
├── graph/
│   ├── graph.py            # Definition of Nodes and Edges
│   ├── state.py            # TypedDict State definition
│   └── nodes/
│       ├── planner.py
│       ├── actor.py
│       └── critic.py
├── tools/
│   ├── env_tools.py        # LangChain tool wrappers
│   └── memory_tools.py     # Journaling tools
├── runner.py               # CLI entry point to run the Graph.
└── utils/                  # DeepAgents utilities
```

## 6. Assumptions & Constraints

* **State Persistence**: Uses LangGraph `MemorySaver` (in-memory or SQLite) for session state, and File System for cross-session "Deep Memory".
* **Single Orchestrator**: One top-level graph coordinating the process.
