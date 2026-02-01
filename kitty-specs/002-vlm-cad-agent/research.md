# Phase 0 Research: VLM CAD Agent

## 1. Resolved Unknowns

### 1.1. Architecture Coupling

* **Question**: Should the Agent be tightly coupled (module import) or loosely coupled (process/container) to the Environment?
* **Decision**: **Loose Coupling** via Gymnasium interface.
* **Rationale**: Security isolation. The environment executes code in a sandboxed container. The agent is a client.

### 1.2. VLM Integration

* **Decision**: Use `litellm` for standardized API access.
* **Rationale**: Supports switching between GPT-4o and Gemini Pro without code changes, essential for benchmarking spatial reasoning.

## 2. Technical Decisions

* **Loop**: Custom ReAct loop instead of LangChain/CrewAI.
  * *Reason*: Specific requirement for "Think, Plan, Act" phases with visual feedback cycles and strictly structured logging (`agent_trace.jsonl`).
* **Memory**: Flat file `journal.md`.
  * *Reason*: Simplicity and portability. Vector DB adds unnecessary complexity for <1000 journal entries.
