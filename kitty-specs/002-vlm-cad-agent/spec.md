# Feature Specification: Engineer Agent

**Feature**: 002-engineer-agent
**Status**: Draft
**Mission**: software-dev

## 1. Overview

The **Engineer Agent** is the autonomous cognitive engine designed to solve benchmarks in the **Agentic CAD Environment**. It is a specialized graph-based system built using LangGraph. It acts as a mechanical engineer, taking a natural language problem description and iteratively producing valid, functional `build123d` CAD scripts.

The agent follows an **Architect → Engineer → Critic** workflow, leveraging a persistent **TODO list** and **Skill-based Memory**.

## 4. Functional Requirements

### 4.1. Cognitive Architecture (Engineer Agent Graph)

The agent shall be implemented as a **LangGraph** state machine with the following nodes:

1. **Architect (Planner) Node**:
    * **Role**: Analyzes the request, existing Skills, and decomposes the problem.
    * **Action**: Creates and persists a **TODO List**.
    * **Transition**: -> `Engineer`.

2. **Engineer (Actor) Node**:
    * **Role**: Executes the current step of the plan/TODO list.
    * **Action**: Writes CAD code, calls environment tools.
    * **Capability**: Can **refuse** the plan if the Architect's requirements are proven impossible.
    * **Transition**: -> `Critic` (on preview/submission) OR -> `Engineer` (on progress).

3. **Critic Node**:
    * **Role**: Validates the implementation against constraints (cost, weight, manufacturability).
    * **Action**: Reviews visual previews and simulation feedback.
    * **Transition**:
        * **Success**: -> `Skill Populator` -> `End`.
        * **Failure**: -> `Architect` (Re-planning).

4. **Skill Populator Node**:
    * **Role**: Captures successful solutions and patterns into `.agent/skills/`. NOTE: the agent skills and skills in this repository are different.

### 4.2. Tool Interface

The agent interacts with the environment via **LangChain Tools**.

#### 4.2.1. Environment Tools (CAD & DFM)

* `search_docs(query: str)`: RAG retrieval from documentation.
* `view_file(path: str)`: Reads the content of any file (scripts, documentation, skills).
* `write_file(content: str, path: str, mode: str)`: Creates, overwrites, or appends to files. Handles `journal.md` specially.
* `edit_file(path: str, find: str, replace: str)`: Targeted text replacement in files.
* `run_command(command: str)`: Executes shell commands (e.g., `python design.py`) in the sandbox.
* `preview_design(path: str)` -> Visual rendering (SVG) and spatial check.
* `submit_design(control_path: str)` -> Full physics simulation and grading.
* `check_manufacturability(design_file, process, quantity)` -> DFM analysis and cost estimation.
* `search_parts(query: str)`: Search for COTS components.
* `preview_part(part_id: str)`: Detailed metadata and recipe for a COTS part.

#### 4.2.2. Skill Management Tools (Persistent Memory)

* `list_skills()`: Lists available specialized knowledge categories.
* `read_skill(skill_name, filename)`: Reads instructions (`SKILL.md`) or references.
* `update_skill(skill_name, content, filename)`: Records new insights or patterns.
* `init_skill(skill_name)`: Initializes a new skill category.
* `package_skill(skill_name)`: Validates and distributes a skill.
* `list_skill_files(skill_name)`: Lists files within a skill folder.
* `run_skill_script(skill_name, script_name, arguments)`: Runs a deterministic helper from a skill.

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

```text
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
