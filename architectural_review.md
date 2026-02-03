# Architectural & Implementation Logic Consistency Review

**Date**: 2026-02-03
**Reviewer**: Jules (AI Agent)
**Methodology**: "Spec-Kitty Checklist" - Cross-referencing Feature Plans, Architecture Document, and Current Codebase State.

## Executive Summary

A comprehensive review of the current `kitty-specs` and source code reveals several inconsistencies between the planned state (in `plan.md` files), the architectural standards (in `architecture.md`), and the actual implementation. The primary issues stem from legacy references (Gymnasium, DeepAgents) persisting in documentation despite being refactored out of the codebase.

Additionally, a "Gap Analysis" (Section 3) identifies critical Underspecifications—areas where the requirements are missing, vague, or where the "Contract" document describes a fiction (e.g., HTTP APIs that don't exist).

---

## 1. Feature Consistency Checklist (Plan vs. Code)

### Feature 001: Agentic CAD Environment
**Spec**: `kitty-specs/001-agentic-cad-environment/`

- [x] **Plan vs. Architecture**: ❌ **MISMATCH**. Plan mentions `gymnasium` and `CADEnv`. Architecture explicitly deprecates them.
- [x] **Plan vs. Codebase**: ❌ **MISMATCH**. Plan implies inheritance from `gym.Env`. Codebase (`src/environment/core.py`) does not appear to use `gymnasium` (grep check).
- [x] **Directory Structure**: ⚠️ **PARTIAL**. `src/environment/core.py` exists, but its role has changed from a Gym Env to a Runtime component.

**Action**: Rewrite `plan.md` to remove `gymnasium`/`CADEnv`. Rename/Refactor `core.py`.

### Feature 002: VLM CAD Agent
**Spec**: `kitty-specs/002-vlm-cad-agent/`

- [x] **Plan vs. Architecture**: ⚠️ **WARNING**. Plan mentions `deepagents`. Architecture allows `langgraph`.
- [x] **Plan vs. Codebase**: ❌ **MISMATCH**. Plan lists `deepagents` as a dependency. Codebase does not use `deepagents`.
- [x] **Memory Alignment**: ✅ **MATCH**. The Plan mentions `journal.md` and file-based memory, which aligns with current implementation.

**Action**: Remove `deepagents` references. Clarify `langgraph` usage.

### Feature 003: MuJoCo Simulation Engine
**Spec**: `kitty-specs/003-mujoco-simulation-engine/`

- [x] **Plan vs. Architecture**: ✅ **MATCH**. Both emphasize `PodmanSandbox` and containerization.
- [x] **Plan vs. Codebase**: ⚠️ **MINOR**. Plan lists `src/simulation/` as the directory. Codebase uses `src/simulation_engine/`.

**Action**: Update plan directory structure.

### Feature 004: Advanced Manufacturing Workbenches
**Spec**: `kitty-specs/004-advanced-manufacturing-workbenches/`

- [x] **Plan vs. Architecture**: ✅ **MATCH**. Separate domain layer, strict DFM validation.
- [x] **Plan vs. Codebase**: ✅ **MATCH**. Files in `src/workbenches/` align with the plan.

### Feature 005: Benchmark Scenario Generator
**Spec**: `kitty-specs/005-benchmark-scenario-generator/`

- [x] **Plan vs. Architecture**: ⚠️ **WARNING**. Plan mentions `deepagents`.
- [x] **Plan vs. Codebase**: ❌ **MISMATCH**. Plan relies on `deepagents` for the "Generator Agent".

**Action**: Update Plan to use `langgraph`.

### Feature 006: COTS Assembly System
**Spec**: `kitty-specs/006-cots-assembly-system/`

- [x] **Plan vs. Architecture**: ✅ **MATCH**.
- [x] **Plan vs. Codebase**: ✅ **MATCH**. `src/cots/` structure matches.

### Feature 007: Agentic CAD Dashboard
**Spec**: `kitty-specs/007-agentic-cad-dashboard/`

- [x] **Plan vs. Architecture**: ✅ **MATCH**.
- [x] **Plan vs. Codebase**: ✅ **MATCH**. Plan says Streamlit, Code says Streamlit.
- [x] **Memory vs. Reality**: ❌ **MISMATCH**. Internal System Memory incorrectly states the dashboard is FastAPI + React.

**Action**: Update System Memory to reflect Streamlit reality.

---

## 2. Global Architectural Consistency

- [x] **No Global State**: ✅ Verified `ToolRuntime` injection pattern.
- [x] **Sandbox Enforcement**: ✅ Verified `PodmanSandbox` usage in `src/environment/sandbox.py`.
- [x] **Configuration**: ✅ Verified usage of `Config` object.

---

## 3. Underspecifications & Air Gaps (The "Missing" Logic)

This section highlights logic and interfaces that are *undefined* in the specs, or where the defined contract contradicts the architectural reality.

### 🛑 001-Environment: The "Missing" Agent Interface
*   **The Gap**: The contract `contracts/env_interface.py` defines a standard `gym.Env` (Reset/Step/Render) interface. However, the architecture has moved to a generic `ToolRuntime`.
*   **Implication**: There is no formal specification for how the Agent "drives" the Runtime. Is it just "Here is a list of tools"?
*   **Underspecification**: The protocol for "Resetting" a task is lost. Does the Agent call a `reset_task` tool? Does the Runtime auto-reset? The Gym interface handled this clearly; `ToolRuntime` does not specified it.

### 🛑 003-Simulation: The "Fictional" REST API
*   **The Gap**: The contract `contracts/api.yaml` describes a RESTful HTTP API (`POST /simulate`) accepting Base64 inputs.
*   **Reality**: The Architecture and Codebase (`src/environment/sandbox.py`) use a `PodmanSandbox` with file-based I/O (mounting volumes, reading JSON results). There is no HTTP server.
*   **Implication**: The Contract is a fiction. Any external integrator (or the Agent itself) looking at `api.yaml` will try to make HTTP calls that will fail.
*   **Underspecification**: The *actual* "Sandbox Interface" (e.g., input JSON schema, expected output file location, exact error codes) is undocumented.

### ⚠️ 007-Dashboard: Data Model & Concurrency
*   **The Gap**: `contracts/schema.md` defines the Database Schema but lacks the *concurrency model* specified in `plan.md` (WAL mode).
*   **Underspecification**:
    *   **Result Metrics Schema**: The `result_metrics` column is defined as "JSON". There is no schema for this JSON. Different tasks will likely produce different metrics (e.g., "max_stress" vs "assembly_time"). Without a schema, the Dashboard cannot reliably render charts.
    *   **Feedback Loop**: The Plan mentions "Human-in-the-loop co-creation" (Feature 005) but the Dashboard Spec (007) is purely read-only ("monitoring"). There is no specified mechanism for the User to inject feedback *back* into the Agent's context.

### ⚠️ General: Error Handling & Propagation
*   **The Gap**: The Architecture mandates "Structured Logging" and "No bare excepts", but does not define the **Error Propagation Protocol** between the Sandbox and the Agent.
*   **Underspecification**: If a user's script segfaults the Physics Engine inside Podman:
    1.  Does `ToolRuntime` catch this?
    2.  What `Observation` does the Agent receive? (A generic "Error"? A stack trace? The partial output?)
    3.  How does the Agent distinguish between "Bad Design" (Physics failure) and "Bad Code" (Syntax error)?
    This logic is critical for agent self-correction but is absent from the specs.

### ⚠️ General: Skill Evolution Criteria
*   **The Gap**: Feature 002 mentions a "Skill Populator" and "Persistent knowledge".
*   **Underspecification**: The *criteria* for promoting a solution to a "Skill" are undefined. Is it "passed once"? "passed 10 times"? "user approved"? Without this, the Agent may pollute its skill library with lucky, brittle solutions.

## 4. Final Recommendations

1.  **Purge Legacy Contracts**: Delete or rewrite `001/.../env_interface.py` and `003/.../api.yaml` to match the `ToolRuntime` + `Podman/File-IO` reality.
2.  **Define the "Runtime Protocol"**: Create a new contract defining exactly how `ToolRuntime` accepts commands and returns observations, replacing the Gym `step()` model.
3.  **Document the Data Schemas**: Define the JSON schema for `result_metrics` in the Dashboard contract so visualization components can be built.
4.  **Spec the Feedback Loop**: If "Co-creation" is a goal, spec the API for the Dashboard to write to a `feedback_queue` that the Agent monitors.
