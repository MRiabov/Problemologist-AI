# Problemologist-AI Architecture

> **Purpose**: This document defines the structural decisions, integration patterns, and non-functional requirements that **all code in this repository must follow**. It unifies the high-level agentic vision with concrete technical constraints.

---

## 1. System Overview: The Agentic Framework

The system creates a benchmark and a training dataset for evaluation of LLM models on dynamics problems. It uses **DeepAgents** (an abstraction over LangChain/LangGraph) to orchestrate complex, long-running agents.

The core consists of two primary autonomous agent graphs:

1. **Benchmark Generator Agent**: Creates, validates, and compiles dataset problems (benchmarks) consisting of CAD models and descriptions.
2. **Engineer Agent**: Solves these problems by designing CAD models under constraints (cost, manufacturability, weight, materials).

### 1.1 Core Philosophy

* **Fail Fast**: Early termination is preferred over complex fallback logic. The system should prioritize the "happy path" and fail quickly if requirements are not met.
* **Lean Code**: Avoid "innovative" or overly complex code. Use battle-tested best practices.
* **No Redundant Fallbacks**: Use assertions over fallbacks. If something fails, update the logic rather than nesting retries.
* **Human-Readable Intermediates**: Feedback and reasoning are stored in Markdown to be friendly to both humans and LLMs.

---

## 2. DeepAgents Framework & Middleware

We leverage the **DeepAgents** framework for managing long-running, complex agents.

### 2.1 Middleware Components

* **FilesystemMiddleware**: Agents operate directly within the filesystem of their assigned container.
  * Supports `ls`, `read`, `write`, `edit` (and async variants `aread`, `awrite`, etc.).
  * **SandboxFilesystemBackend**: Worker nodes use a disposable sandbox environment for safe execution.
* **TodoListMiddleware**: Provides native management for high-level plans and TODO lists.

### 2.2 Distributed Execution

* **Controller Node**: Runs the LLM logic, tool parsing, and state management.
* **Worker Node**: Executes the "body" of the work:
  * Simulation (MuJoCo).
  * Python script execution (CAD generation).
  * Linting and validation.
  * **Isolation**: Uses **Podman** containers with attached volumes.
* **Orchestration**: **Temporal** is used to orchestrate workers and handle long-running tasks/retries (e.g., simulations > 30s).

---

## 3. Agent Architecture Details

### 3.1 Benchmark Generator Agent

**Goal**: Create randomized, valid CAD benchmarks converted into MJCF (MuJoCo XML).

* **Workflow**:
    1. **Drafting**: Generates CAD models with randomized parameters (size, position, etc.).
    2. **Internal Review**: specialized critic node reviews logic.
    3. **Verification**:
        * **Geometry**: No self-intersections.
        * **Compilation**: Code must run cleanly.
        * **MJCF Validity**: Validates against XML schema.
        * **Stability**: Runs a few simulation frames to ensure no immediate explosion.
* **Artifacts**:
  * Generates **24-view renders** (clockwise, 3 levels) for the Engineer to "see".

### 3.2 Engineer Agent

**Goal**: Solve benchmarks by generating manufacturable CAD designs.

* **Roles**:
  * **Architect (Planner)**: Creates high-level `plan.md` and `todo.md`.
  * **Engineer (Actor)**: Implements the solution in `script.py`. Can preview tools.
  * **Critic**: Verifies against constraints (Cost, Weight, Manufacturability).
* **Constraints**:
  * **Manufacturability**: Verified via "Workbenches" (CNC, Injection Molding, 3D Printing).
  * **Cost**: must be within target unit cost.
* **Feedback**:
  * Received in **Markdown** format.
  * Simulation feedback includes coordinates and simple text summaries (e.g., "failed to hit objective").

---

## 4. Skills, Learning & Memory

The system employs a continuous learning loop.

### 4.1 Artifacts & Memory

* **Journal (`journal.md`)**: Episodic memory. A structured log where agents record intent, result, reflection, and next steps. used for context across retries and debugging.
* **TODOs (`todo.md`)**: Operational plan managed by the Planner.
* **Plan (`plan.md`)**: High-level strategic plan.

### 4.2 Making Agents Smarter: The Learner Sidecar

* **Sidecar Agent**: An async "Learner" agent (using smaller models like DeepSeek) runs parallel to execution.
* **Process**:
    1. Scans Journals for repeated struggles (>4 failed tool calls) or patterns.
    2. Updates **Skills** in `skills/` folder (Git-versioned).
    3. Pushes updates to a skills repository.
* **Skill Access**: Agents pull skills from a Git repo at the start of every run. Skills are **read-only** for the executing agent.

---

## 5. Simulation & Verification Environment

We use **MuJoCo** for physics simulation due to speed and stability.

### 5.1 Simulation Objectives

defined by Axis-Aligned Bounding Boxes (AABB):

1. **Build Zone**: Where the agent can create parts.
2. **Goal Zone**: Where the target object must end up.
3. **Forbid Zone**: Areas to avoid.

### 5.2 Randomization

Benchmarks are randomized to ensure robustness:

* **Volume scaling**: 2x variance.
* **Position jitter**: Up to 40% of size.
* The Engineer must solve for the randomized environment, not a static one.

---

## 6. Technical Architecture & Constraints

### 6.1 Tooling & API

Tools are Python functions imported into the agent's script environment:

* `validate_and_price(component) -> float | dict`: Checks manufacturability and calculates cost.
* `simulate(Compound) -> SimulationResult`: Submits model for simulation (runs locally in worker, commits files).
* `submit_for_review(Compound)`: Submits final assembly for review.
* `get_docs_for(type)`: Invokes documentation subagent (RAG over local docs).

### 6.2 Infrastructure (Deployment)

* **Platform**: **Railway**.
* **Databases**:
  * **Postgres**: For Temporal, LangFuse, and App Data (partitioned).
  * **S3**: For Assets (Videos, MJCF, final CAD models).
* **Observability**:
  * **LangFuse**: For LLM traces.
  * **Structlog**: For structured application logging.
  * **Backups**: Daily cron backup of DB to S3.

### 6.3 Frontend

* **Stack**: Vite + React.
* **Types**: Autogenerated from Backend OpenAPI schemas.
* **Capabilities**:
  * Support for **Batch Generation**.
  * Debug views (inspect agent reasoning, interrupt execution).

### 6.4 Non-Functional Requirements (NFRs)

#### Security

* **Sandbox**: All agent code runs in isolated Podman containers.
* **Strict API**: OpenAPI schemas enforced by `schemathesis`.
* **Read-Only Utils**: Core utilities are mounted read-only to prevent agents from breaking the harness.

#### Latency & Networking

* **Internal Network**: Usage of Railway internal networking for speed.
* **File Sync**: Files are written independently to worker storage; explicitly uploaded to S3 only when needed (Assets).

---

## 7. Decision Log

| Date | Decision | Rationale |
|------|----------|-----------|
| 2026-02-06 | **DeepAgents Framework** | Adopted `deepagents` for unified agent/middleware management. |
| 2026-02-06 | **Sidecar Learner** | Use a separate async agent for skill extraction to avoid context overflow and ensure cleaner updates. |
| 2026-02-06 | **Journaling** | Introduce `journal.md` as structured episodic memory for debugging and learning. |
| 2026-02-04 | **Fail Fast Principle** | Prioritize "happy path" and early termination over complex fallback logic. |
| 2026-02-04 | **Strict Observability** | Record all thoughts, errors, and renders for training. |
| 2026-02-04 | **Podman Sandbox** | Requirement for containerized execution of untrusted agent code. |
