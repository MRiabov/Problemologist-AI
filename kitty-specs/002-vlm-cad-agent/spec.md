# Feature Specification: VLM CAD Agent

**Feature**: 002-vlm-cad-agent
**Status**: Draft
**Mission**: software-dev

## 1. Overview

The **VLM CAD Agent** is the autonomous cognitive engine designed to operate within the **Agentic CAD Environment** (Spec 001). It is a specialized Vision-Language Model (VLM) system built using the **DeepAgents** framework (based on LangGraph). It acts as a mechanical engineer, taking a natural language problem description and iteratively producing valid, functional `build123d` CAD scripts.

Unlike generic coding assistants, this agent leverages the "Deep Agent" architecture to perform long-horizon planning, sub-task delegation, and persistent **Skill-based Memory Management**. It validates its own work using visual feedback (renders) and simulation results.

## 2. Goals & Success Criteria

### 2.1. Primary Goals

1. **Autonomous Resolution**: The agent must be able to solve geometric problems from start to finish without human intervention, handling its own errors and edge cases.
2. **Multimodal Reasoning**: Effectively use the `preview_design` tool to visually inspect geometry and correct spatial errors that are not visible in the code text.
3. **Structured Cognition**: Enforce a separation between **Planning** (understanding the problem) and **Execution** (writing code).
4. **Standard Agent Interface**: The agent operates like a human developer, using standard tools (`view_file`, `edit_file`, `run_command`) effectively in a persistent environment, rather than relying on narrow, custom API calls.
5. **Economic Optimization**: Respect `max_unit_cost` and `target_quantity` constraints.
6. **Skill-Based Learning**: Implement a "Skill Population" system where the agent records specialized knowledge into `docs/skills/` to reuse proven strategies strategies.

### 2.2. Success Criteria

* **Solver Rate**: The agent successfully solves >50% of the "Easy" benchmark problems defined in Spec 001 on the first try.
* **Budget Adherence**: The agent meets or beats the `max_unit_cost` in >70% of successful completions.
* **Self-Healing**: The agent can recover from at least one syntax error and one geometric violation per session without crashing.
* **Economic Adaptability**: The agent demonstrates switching from CNC to Injection Molding when production volume increases (e.g., from 1 to 10,000 units).
* **Visual Utility**: In >30% of iterations, the agent explicitly modifies code after requesting a `preview_design`, indicating active use of visual feedback.
* **Skill Growth**: The `.agent/skills/` directory contains >5 new reference files or scripts after 10 sessions of problem-solving.

## 3. User Stories

### 3.1. As an Operator/Researcher

* **As a Researcher**, I want to configure the agent to use different backend models (Gemini Pro, GPT-4o) so I can benchmark their spatial reasoning capabilities.
* **As an Operator**, I want to see a real-time structured log of the agent's "Thought," "Tool Call," and "Observation" so I can debug its reasoning process.
* **As an Operator**, I want the agent to automatically save its successful strategies to specialized **Skill folders** so it gets smarter over time.

### 3.2. As the Agent (Internal Monologue)

* **As the Agent**, I want to look up `build123d` documentation before writing complex features to ensure I use the correct API signature.
* **As the Agent**, I want to render a low-res preview of my part to check if the holes are aligned before submitting the final expensive physics simulation.
* **As the Agent**, I want to read the `build123d_cad_drafting_skill` to recall proven patterns for "Press Fit Tolerances" before designing a connector.

## 4. Functional Requirements

### 4.1. Cognitive Architecture (DeepAgents Graph)

The agent shall be implemented as a **LangGraph** state machine with the following nodes:

1. **Planner Node**:
    * **Role**: Analyzes the request and existing Skills.
    * **Action**: Discovers relevant skills using `list_skills` and reads them via `read_skill`. Generates a structured plan.
    * **Transition**: -> `Actor`.

2. **Actor Node** (The Builder):
    * **Role**: Executes the current step of the plan.
    * **Action**: Calls environment tools (`write_script`, `preview_design`) and skill tools (`read_skill`) to maintain high coding standards.
    * **Transition**: -> `Critic` (if submission or preview) OR -> `Actor` (if continuing).

3. **Critic Node** (The Validator):
    * **Role**: Visual, Logic, and Economic validation.
    * **Trigger**: After `submit_design` or `preview_design`.
    * **Action**:
        * Checks simulator feedback or vision output.
        * **Cost Guard**: Compares current unit cost against budget.
        * **Consensus**: Evaluates `force_submit` justifications. If valid, signals `HARD_LIMIT_REACHED` to Planner.
    * **Transition**:
        * **Success/Terminal**: -> `Skill Populator` -> `End`.
        * **Failure/Iteration**: -> `Planner` (Re-planning) -> `Actor`.

4. **Skill Populator Node**:
    * **Role**: Persistent procedural memory management.
    * **Action**: Uses `update_skill` to capture new insights, recurring patterns, or bug fixes discovered during the session into the appropriate skill folder.

### 4.2. Tool Interface

The agent interacts with the environment via **LangChain Tools**.

#### 4.2.1. Environment Tools (CAD & DFM)

* `search_docs(query: str)`: RAG retrieval from documentation.
* `write_script(content: str, path: str)`: Creates or overwrites files in the workspace.
* `edit_script(find: str, replace: str, path: str)`: Targeted text replacement.
* `preview_design()` -> Visual rendering and spatial check.
* `submit_design(control_path: str)` -> Full physics simulation and grading.
* `check_manufacturability(process, quantity)` -> DFM analysis and cost estimation.

#### 4.2.2. Skill Management Tools (Persistent Memory)

* `list_skills()`: Lists available specialized knowledge categories.
* `read_skill(skill_name, filename)`: Reads instructions (`SKILL.md`) or references.
* `update_skill(skill_name, content, filename)`: Records new insights or patterns.
* `init_skill(skill_name)`: Initializes a new skill category.
* `package_skill(skill_name)`: Validates and distributes a skill.

### 4.3. LLM Integration

* **Framework**: `langchain-core` / `langchain-anthropic` / `langchain-google-genai`.
* **Context Management**: Progressive disclosure of skills (Metadata -> SKILL.md -> References) to optimize token usage.
* **Image Handling**: Native multimodal support via LangChain.

## 5. Technical Design

### 5.1. Tech Stack

* **Language**: Python 3.10+
* **Framework**: `langgraph` / `langchain`.
* **Skills System**: File-system based storage under `.agent/skills/`.
* **Models**: Gemini 2.0 Pro (preferred), Claude 3.5 Sonnet.

### 5.2. Component Structure

```
src/agent/
├── graph/
│   ├── graph.py            # Definition of Nodes and Edges
│   ├── state.py            # TypedDict State definition
│   └── nodes/
│       ├── planner.py
│       ├── actor.py
│       ├── critic.py
│       └── skill_populator.py
├── tools/
│   ├── env.py              # Environment & Skill tools
│   └── env_adapter.py      # Async wrappers and env mapping
├── runner.py               # CLI entry point to run the Graph.
└── utils/                  # Prompt management and LLM helpers
```

## 6. Assumptions & Constraints

* **State Persistence**: Uses LangGraph `MemorySaver` (in-memory or SQLite) for session state, and File System for cross-session "Deep Memory".
* **Single Orchestrator**: One top-level graph coordinating the process.
