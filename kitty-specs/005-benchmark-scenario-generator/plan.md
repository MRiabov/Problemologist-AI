# Implementation Plan - Benchmark Scenario Generator

**Feature**: 005-benchmark-scenario-generator
**Goal**: Build an automated pipeline to generate, validate, and export physics benchmark scenarios.
**Architecture**: Standalone utility package driven by `deepagents` (LangGraph).

## 1. Technical Context

This feature implements a "Scenario Architect" agent—a specialized automation that uses an LLM to author Python code. Unlike the main VLM CAD Agent (which *solves* problems), this agent *creates* problems.

### Technology Stack

* **Orchestration**: `deepagents` (LangGraph) for the "Plan → Code → Validate" loop.
* **LLM Interface**: Reuse `src/agent/utils/llm.py` (Gemini/OpenAI).
* **Geometry**: `build123d` for parametric CAD generation.
* **Physics**: `mujoco` for XML generation and stability checks.
* **File System**: Output to `datasets/benchmarks/`.

### Key Components

1. **Generator Agent**: A LangGraph state machine with nodes for `Prompting`, `Coding`, and `Reviewing`.
2. **Validator**: A headless script that loads generated XMLs into MuJoCo to check for instant explosions (instability).
3. **Randomizer**: Logic injected into generated scripts to vary parameters (±40% bounds) via a seed.

## 2. Constitution Check

* **No-Go**: No runtime dependency on the generator for the end-user agent. This is a build-time tool.
* **Conventions**: Python scripts must follow project linting rules. Generated assets (STL/XML) must be deterministic for a given seed.

## 3. High-Level Design

### 3.1 Data Model (`data-model.md`)

* **ScenarioManifest**: Metadata for a benchmark (name, tier, parameters, seed).
* **GenerationRequest**: Input prompt + config (e.g., "10 levers, tier 2").
* **ValidationReport**: Result of the stability check (pass/fail, max velocity).

### 3.2 Directory Structure

```
src/
  generators/
    benchmark/
      __init__.py
      agent.py       # LangGraph definition
      prompts.py     # System prompts for scenario authoring
      validator.py   # Headless MuJoCo check
      manager.py     # CLI entry point
```

## 4. Phase 1: Core Generator Logic

**Goal**: Can generate a *single* valid scenario from a text prompt.

1. **Scaffold Generator Package**: Create `src/generators/benchmark/` structure.
2. **Implement Validator**: Write `validator.py` to load an MJCF string and step the sim.
3. **Implement Generator Agent**:
    * **Planner Node**: Breaks down "Tier 1 Peg-in-Hole" into geometric requirements.
    * **Coder Node**: writes a `build123d` script with a `build(seed)` function.
    * **Critique Node**: Uses the `Validator` output to feedback errors to the Coder.

## 5. Phase 2: Batch Processing & Randomization

**Goal**: Can generate *40+* diverse scenarios and manage the dataset.

1. **Implement Randomization Wrapper**: A harness that runs the generated `build(seed)` function multiple times with different seeds to verify stability across the parameter range.
2. **CLI Tool**: Implement `python -m src.generators.benchmark.manager generate --tier 1 --count 10`.
3. **Asset Export**: Ensure the pipeline correctly organizes STLs and XMLs into `datasets/benchmarks/`.

## 6. Phase 3: Interactive Pipeline Support

**Goal**: Support human-in-the-loop co-creation via the Dashboard.

1. **Enhance Planner Node**: Update system prompts to include Learning Objectives and Self-Verification strategies.
2. **Expose Interactive Hooks**: Modify `agent.py` or create wrapper utilities to allow external (UI) injection of plan edits and code overrides.
3. **Rendering Integration**: Implement `src/generators/benchmark/renderer.py` to produce visual previews for each generation attempt to support UI-based review.

## 7. Success Criteria Verification

* **Efficiency**: Run `manager generate --count 10` and measure runtime (Target: < 5 mins).
* **Yield**: Count valid vs. invalid outputs in the `staging` folder.
* **Quality**: Manual review of the generated "Tier 2" lever mechanisms.
