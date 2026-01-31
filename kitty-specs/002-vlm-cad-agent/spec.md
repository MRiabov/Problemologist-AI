# Feature Specification: VLM CAD Agent

**Feature**: 002-vlm-cad-agent
**Status**: Draft
**Mission**: software-dev

## 1. Overview

The **VLM CAD Agent** is the autonomous cognitive engine designed to operate within the **Agentic CAD Environment** (Spec 001). It is a specialized Vision-Language Model (VLM) system that acts as a mechanical engineer, taking a natural language problem description and iteratively producing valid, functional `build123d` CAD scripts.

Unlike generic coding assistants, this agent implements a rigorous **"Think, Plan, Act"** cognitive architecture. It validates its own work using visual feedback (renders) and simulation results, maintaining a persistent "Journal" of lessons learned to improve over time.

## 2. Goals & Success Criteria

### 2.1. Primary Goals

1. **Autonomous Resolution**: The agent must be able to solve geometric problems from start to finish without human intervention, handling its own errors and edge cases.
2. **Multimodal Reasoning**: Effectively use the `preview_design` tool to visually inspect geometry and correct spatial errors that are not visible in the code text.
3. **Structured Cognition**: Enforce a strict separation between **Planning** (understanding the problem, researching docs) and **Execution** (writing code, iterating).
4. **Self-Correction**: When a submission fails (physics or geometry violation), the agent must analyze the failure report, hypothesize a fix, and retry.
5. **Long-Term Memory**: Implement a "Journaling" system where the agent can record specific techniques (e.g., "Correct syntax for build123d Loft") to avoid repeating mistakes across sessions.

### 2.2. Success Criteria

* **Solver Rate**: The agent successfully solves >50% of the "Easy" benchmark problems defined in Spec 001 on the first try.
* **Self-Healing**: The agent can recover from at least one syntax error and one geometric violation per session without crashing.
* **Visual Utility**: In >30% of iterations, the agent explicitly modifies code after requesting a `preview_design`, indicating active use of visual feedback.
* **Cost Efficiency**: The reasoning loop optimizes for token usage by summarizing observations rather than keeping full verbose logs in the active context window.

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

### 4.1. Cognitive Architecture

The agent shall implement a customized ReAct (Reason + Act) loop with distinct phases:

1. **Phase 0: Orientation & Planning**
    * **Input**: User Prompt + Environment Constraints.
    * **Action**: Read `journal.md` (Memory), Browse Docs (RAG).
    * **Output**: A concise **Implementation Plan** (written to `plan.md` or internal scratchpad) outlining the geometric approach.

2. **Phase 1: Execution Loop**
    * **Input**: Current file state + Plan.
    * **Cycle**:
        * **Think**: Analyze current state vs. plan.
        * **Act**: Write/Edit Code or Request Preview.
        * **Observe**: Parse execution output or Image.
    * **Constraint**: Maximum `N` steps (e.g., 20) to prevent infinite loops.

3. **Phase 2: Validation & Reflection**
    * **Trigger**: Agent calls `submit_design`.
    * **Outcome**:
        * **Success**: Update `journal.md` with "What went right".
        * **Failure**: Analyze Feedback, revert to Phase 1.

### 4.2. Tool Interface

The agent interacts with the environment via a strictly typed interface (JSON Schema).

#### 4.2.1. Environment Tools (Wrapped from Spec 001)

* `search_docs(query: str)`
* `write_script(content: str)`
* `edit_script(find: str, replace: str)`
* `preview_design()` -> Returns schema with `image_url` (or base64).
* `submit_design()` -> Returns evaluation report.

#### 4.2.2. Meta-Cognitive Tools

* `read_journal(topic: str)`: Fuzzy search or full read of the simplified `journal.md` memory file.
* `write_journal(entry: str, tags: list[str])`: Append a learned lesson to the long-term memory.
* `update_plan(status: str, notes: str)`: Update the current session's scratchpad.

### 4.3. LLM Integration

* **Provider Agnostic**: The system shall use a standardized adapter pattern to support:
  * **OpenAI** (GPT-4o)
  * **Google** (Gemini 1.5 Pro)
  * **Anthropic** (Claude 3.5 Sonnet)
* **Context Management**:
  * **Sliding Window**: Summarize older conversation turns if context limit is approached.
  * **Image Handling**: Images from `preview_design` are passed as native multimodal inputs (if supported) or transiently described (if text-only model). **Priority is Native Multimodal**.

### 4.4. Observability & Logging

* **Traceability**: Every agent step (Thought -> Tool -> Result) is logged to a structured JSONL file `agent_trace.jsonl`.
* **Console Output**: Rich terminal output (using `rich` library) showing the agent's "Thought" in one color and "Action" in another.

## 5. Technical Design

### 5.1. Tech Stack

* **Language**: Python 3.10+
* **Model Client**: `litellm` (or direct SDKs if preferred for specific VLM features).
* **Schema Validation**: `Pydantic`.
* **Prompt Management**: `Jinja2` templates for system prompts.

### 5.2. Component Structure

```
src/agent/
├── core/
│   ├── engine.py           # The main ReAct loop
│   ├── context.py          # Context window manager
│   └── memory.py           # Journaling system (File-based)
├── clients/
│   ├── base.py             # Abstract LLM Client
│   ├── openai_client.py
│   └── gemini_client.py
├── prompts/
│   ├── system_persona.j2   # "You are an expert CAD engineer..."
│   └── planning.j2         # "First, outline your approach..."
└── runner.py               # CLI entry point to run agent on a problem.
```

## 6. Assumptions & Constraints

* **Sync/Async**: The loop will be synchronous for simplicity (Step 1 wait for Step 2).
* **Single Agent**: No multi-agent collaboration (e.g., separate Critic/Coder) in Version 1. One model does it all.
* **Local File Memory**: Long-term memory is a flat Markdown file (`journal.md`), not a vector database, to keep dependencies low.
