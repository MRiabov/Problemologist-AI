# Research Report: WP4 Adaptability & Steerability

## Overview

Work Package 4 (Adaptability) aims to evolve the AI from a "Generator" to a "Co-pilot". The core mechanism is **Steerability**, allowing human engineers to provide precise feedback on geometric features (parts, faces, edges) to guide iterative design modifications.

### Steerability Features

The architecture defines four primary ways to steer the agent:

1. **Exact Pointing (Interactive)**: Selecting faces, edges, or vertices in the CAD viewer and "attaching" them to the prompt.
2. **Code Steering**: Referencing specific file line ranges (e.g., `@model.py:120-125`) for targeted modifications.
3. **@-Mentions**: Referencing parts or subassemblies from the BOM/Part Tree directly in chat.
4. **Prompt Intervention**: Direct human correction of incorrect plans or traces to manually adjust the agent's trajectory.

## Key Findings

### 1. Topological Naming & Selection

* **Mechanism**: The system uses a custom GLB export process in `worker/simulation/builder.py` (`export_topology_glb`).
* **Naming Convention**: Every face of a `build123d` part is exported as a separate mesh named `face_{idx}`, where `idx` is the 0-based index in the part's face list.
* **Frontend Integration**: `ModelViewer.tsx` detects these naming prefixes and allows users to click and select specific geometric features.
* **Feedback Loop**: When a face is selected, its identifier (e.g., `face_12`) is attached to the conversation metadata, allowing the agent to know exactly which feature the user is referring to.

### 2. Semantic Selectors vs. Hardcoded IDs

* **The Problem**: CAD topology is unstable; indices change when geometry is modified (the "Topological Naming Problem").
* **The Solution**: The system encourages the use of **Semantic Selectors** (e.g., `faces().sort_by(Axis.Z)[-1]`) instead of hardcoded indices.
* **Agent Role**: The agent is trained to translate a selected face index (e.g., `face_12`) into a robust semantic selector by analyzing the face's properties (normal, position, size) in a "scratchpad" or temporary execution environment.

### 3. "Partially Executed" / Undocumented Logic

* **Tagger Tool**: While mentioned in architecture docs as a standalone tool, the "tagging" logic is currently integrated into the `SimulationBuilder`'s export path.
* **Direct Execution**: The "undocumented" logic refers to the agent's ability to run inspection scripts `directly` via the worker's runtime to query topology. For example, an agent might run:

    ```python
    part = build_part()
    face = part.faces()[12]
    print(f"Face 12 normal: {face.normal_at()}, center: {face.center()}")
    ```

    This allows the agent to find a semantic description (e.g., "the top vertical face") that matches the user's selection.
* **Geometric Analysis**: `worker/workbenches/analysis_utils.py` contains logic like `check_undercuts` that identifies and returns face indices, demonstrating how physics/manufacturing constraints are mapped back to topology.

### 4. Intent Extraction

* **Stateful Interaction**: The system is designed for a Redis-backed session model where the CAD state is persistent across turns.
* **Iterative Refinement**: The agent is intended to "load" the existing STEP/code, apply a "Geometric Diff" based on feedback, and verify the change with "Metric Diffs" (cost, weight, stress).

## Gaps Identified

* **Line-Targeted Edits**: Implementation of "Steering code directly" using the `@file:line-range` syntax (L1031-1047 in `desired_architecture.md`).
* **Assembly tree @-mentions**: Integration of the part tree with the chat interface for easy targeting.
* **Prompt Backend for Intervention**: A dedicated loop for handling human corrections to intermediate agent plans.
* **Automated Tagger**: There is no high-level tool that automatically generates semantic descriptions for all faces (this is currently an agent-led reasoning task).
* **Visual Diffs**: The infrastructure for "Geometric Diffs" (visualizing before/after changes) is planned in `ModelViewer.tsx` but depends on the agent providing the `previous_model` URL.

## Conclusion

The "Steerability" intent is successfully implemented in the **plumbing** (GLB tagging + Frontend selection). The **intelligence layer** (translating selection to code) relies on the agent's ability to use the provided tools and inspection scripts to discover robust selectors.
