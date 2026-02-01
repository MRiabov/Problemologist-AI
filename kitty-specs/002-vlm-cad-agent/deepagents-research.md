# Research: DeepAgents Framework (LangChain)

**Date**: 2026-02-01
**Source**: Web Research
**Subject**: Migration from Native Tools to `deepagents` Framework

## 1. Overview

`deepagents` is a framework by LangChain developers designed to build "deep" agents—agents capable of long-horizon planning, iteration, and complex task execution, moving beyond simple "shallow" tool-calling loops. It is built on top of **LangGraph**.

Key philosophy:

* **Planning First**: Explicit planning steps before execution.
* **Deep Memory**: Uses file-system based memory (workspace) to persist context and state across long runs, preventing context window overflow.
* **Sub-Agent Delegation**: Capable of spawning sub-agents for specialized tasks, sharing a common workspace/file system.
* **Graph-Based**: Leverages LangGraph for state management, conditional routing, and persistence.

## 2. Key Capabilities for VLM CAD Agent

### 2.1. Planning & Reflection

`deepagents` provides primitives for the "Think, Plan, Act" loop we aim to implement. Instead of writing a custom while-loop in Python, we define a Graph where:

* **Planner Node**: Generates the high-level plan.
* **executor Node**: Executes steps.
* **Reflector Node**: Validates the output (Visual Feedback) and updates the plan.

### 2.2. File System Memory

The framework emphasizes using the file system as "Long Term Memory". This aligns perfectly with our `journal.md` and `plan.md` requirements. The agent can read/write files to store intermediate state, ensuring that even if the context window is reset, the "knowledge" remains on disk.

### 2.3. Sub-Agents (Future Proofing)

While our VLM CAD Agent is primarily a single entity, `deepagents` allows us to easily isolate the "Visual Validator" as a separate sub-agent if the logic becomes too complex, without rewriting the orchestration layer.

## 3. Implementation Changes

### 3.1. Tech Stack

* **Add**: `deepagents`, `langgraph`, `langchain-core`, `langchain-openai` (or `langchain-google-genai`).
* **Remove/Refactor**: Custom `engine.py` (ReAct loop), custom `context.py` (LangGraph handles state).

### 3.2. Architecture

* **State**: Defined as a TypedDict (LangGraph mechanics).
* **Nodes**: Python functions representing steps (Plan, Act, Observe).
* **Edges**: Logic to route between nodes (e.g., if `validation_failed` -> goto `Replan`).

## 4. Resources

* **Official Docs**: Search for `deepagents` on LangChain documentation or GitHub (likely under `langchain-ai/deepagents` or similar if released as a standalone, otherwise part of `langgraph` examples).
* **LangGraph**: [LangGraph Documentation](https://python.langchain.com/docs/langgraph)
