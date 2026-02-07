# Frontend Component Review Report

## Overview
This report evaluates the current frontend components of the Agentic CAD Dashboard against the `desired_architecture.md`. The primary goal of the system is to create a benchmark and training dataset for evaluating LLM models on solving dynamics problems. The dashboard must serve as a visualization and debugging tool for two main agents: the **Benchmark Generator** and the **Engineer (Problem Solver)**.

## 1. Essential Components (Keep & Enhance)
These components are critical to the core mission of visualizing reasoning traces, agent state, and outcomes.

- **Benchmark Generation View (`BenchmarkGeneration.tsx`)**: Essential for monitoring the creation and validation of dynamics problems.
- **Engineer Workspace (`EngineerWorkspace.tsx`)**: Essential for observing the solver agent's progress, TODOs, and solution verification.
- **Reasoning Traces / Cognition Stream**: Currently integrated into both views. Critical for understanding *how* the agent arrived at a solution.
- **Asset Viewport (Media/3D)**: Critical for visual verification. Current implementation uses images/videos; needs transition to interactive 3D (WP04).
- **TODO List / Architect's Guidance**: Essential for the task-driven nature of the Engineer agent.
- **Metadata Cards (Cost, Weight, DFM)**: Essential for visualizing the constraints mentioned in the architecture.
- **Live Logs / WebSocket Integration**: Essential for real-time debugging.

## 2. Redundant / Non-Essential Components (Candidate for Removal)
These components either overlap significantly with others or do not align with the streamlined agent-driven workflow.

- **`IdeDashboard.tsx` (REDUNDANT)**: 
    - *Reasoning*: It attempts to provide a general-purpose IDE view that overlaps with `EngineerWorkspace`. 
    - *Recommendation*: Remove `IdeDashboard` and merge any unique features (like the multi-file tab view) into `EngineerWorkspace` if the agent supports multi-file editing.
- **`Layout.tsx` (REDUNDANT)**:
    - *Reasoning*: Older, simplified layout component replaced by `AppLayout.tsx`.
    - *Recommendation*: Delete.
- **Mocked File Explorer in IDE**: 
    - *Reasoning*: Without integration with the actual worker container's filesystem, a mocked file tree adds unnecessary complexity.
    - *Recommendation*: Remove until a real Filesystem API is implemented.

## 3. Missing Components (Needed for Architectural Alignment)
These elements are defined in the architecture but are currently missing or under-represented in the UI.

- **Scannable Journal / Scannable Summaries**: 
    - *Architecture Ref*: "Journals and 'scannable' summaries of the execution."
    - *Current State*: Basic text area.
    - *Need*: A structured, chronological view of "Intent -> Result -> Reflection" as described in the Journal section of the spec.
- **Skill Explorer / Skill.md Viewer**:
    - *Architecture Ref*: "Skills acquired during execution... SKILL.md files and their assets."
    - *Need*: A dedicated view or panel to browse learned skills, their reference documentation, and associated scripts.
- **Code Evolution View (Git Diffs)**:
    - *Architecture Ref*: "We want to track code evolution. Hence, we will use `git add . && git commit` during every successful `simulate` call."
    - *Need*: A way to see the diffs between simulation attempts to understand how the agent is iterating on the CAD code.
- **Worker/Container Status Dashboard**:
    - *Architecture Ref*: "Execution runs in isolated containers... The container will download/pull skills... before every run."
    - *Need*: Visibility into the health and status of the underlying worker nodes/containers (e.g., skill sync status, resource usage).

## Summary Table
| Component | Status | Action |
| :--- | :--- | :--- |
| `EngineerWorkspace` | Essential | Keep & Polishing |
| `BenchmarkGeneration` | Essential | Keep & Polishing |
| `IdeDashboard` | Redundant | **Remove** |
| `Layout.tsx` | Redundant | **Remove** |
| `AppLayout` / `Sidebar` | Essential | Keep |
| `JournalViewer` | Missing | **Implement** |
| `SkillExplorer` | Missing | **Implement** |
| `CodeHistory` (Diffs) | Missing | **Implement** |

## Recommendation
About 20-30% of the current "IDE-centric" mockup code can be removed to focus on the **Agent-centric** workflow. The frontend should prioritize the "Reasoning Trace" and "Scannable Journal" over traditional IDE features like file trees, as the primary users are researchers and developers evaluating agent performance rather than humans manually editing CAD in this specific dashboard.
