# Architecture Delta: Feb 8 Round 2

**Date**: 2026-02-08  
**Base Commit**: `d5b5d6dfa838a5f152a7d5dca2f0ad292e9af955` ("cleanup and spec")  
**Diff Stats**: +479 lines, -39 lines across 16 commits

---

## Summary of Changes

The `desired_architecture.md` document received significant clarifications across **agent architecture**, **data exchange contracts**, **frontend specifications**, and **infrastructure**. This document categorizes all new or modified requirements for implementation.

---

<!--
## 1. Agent Purpose & Subagent Structure *(NEW)*

### 1.1 Benchmark Generator Agent

**Added sections**: Agent purpose, subagents breakdown, output requirements.

| Component | Description |
|-----------|-------------|
| **Planner** | Composes description of benchmark behavior, learning goal, and designs the challenge |
| **CAD Coder** | Implements the benchmark from the plan |
| **Reviewer** | Reviews for feasibility, constraint violations, proper randomization |

**Sample output**: A complete example benchmark plan showing all required sections (learning objective, static geometry, input object, forbid zones, goal zone, rescale limits, randomization, python script structure).
-->

<!-- note: pretty sure build zones weren't implemented -->

### 1.2 Engineer Agent

**Added sections**: Purpose clarification, build zone constraints.

> "The agent should operate in the most closely real-life environment. The agent shouldn't even understand it works in a 'fake' environment."

**New constraint**: Engineer is constrained on `where the agent can actually build their solution` (build zones).

---

## 2. Agent Handover Contracts *(NEW - MAJOR)*

### 2.1 Full Handover Chain Specified

```
User prompt →
  Benchmark Planner ↔ Benchmark CAD Agent ↔ Benchmark Reviewer →
    (accepts) → Engineering Planner ("Lead Engineer") ↔ CAD Engineer ↔ Engineering Reviewer
```

**Key flows**:

- Benchmark CAD agent can **refuse plan** only if geometry is invalid (not due to skill failure)
- Benchmark Reviewer can **refuse CAD** if geometry has intersections or is impossible to solve
- Engineering CAD agent can **refuse plan** if price/weight is too low
- Engineering Reviewer must **confirm refusal** before replanning

<!--
### 2.2 Benchmark Planner → CAD Agent Files

| File | Contents |
|------|----------|
| `plan.md` | Learning objective, geometry coordinates, moving parts, input/goal/forbid objectives |
| `todo.md` | TODO list from planner |
| `objectives.yaml` | **Draft** with rough values filled in |
-->

### 2.3 Engineer Planner → CAD Agent Files

| File | Contents |
|------|----------|
| `plan.md` | Solution overview, parts list, assembly strategy, cost/weight budget, risk assessment |
| `objectives.yaml` | Stripped-down version with planner's **own** max price/weight (under benchmark ceiling) |
| `todo.md` | TODO list |

**Critical clarification**: The benchmark generator sets **ceiling** price/weight. The engineering planner sets operating constraints **under** that ceiling.

---

## 3. `objectives.yaml` Structure *(NEW - COMPREHENSIVE)*

*MISSING: Programmatic validation against this schema.*

Full schema now specified:

```yaml
objectives:
  goal_zone: {min: [x,y,z], max: [x,y,z]}
  forbid_zones: [{name, min, max}, ...]
  build_zone: {min, max}  # CONSTRAINT: Design must fit within

simulation_bounds:
  min: [-50, -50, 0]
  max: [50, 50, 100]

moved_object:
  label: "projectile_ball"
  shape: "sphere"
  static_randomization:
    radius: [min, max]  # Varies between benchmark runs
  start_position: [x, y, z]
  runtime_jitter: [±x, ±y, ±z]  # Per-simulation variation

moving_parts:
  - name: str
    type: "motor" | "passive"
    position: [x, y, z]
    dof: "rotate_x" | "rotate_y" | "rotate_z" | "slide_x" | "slide_y" | "slide_z"
    control: {mode, speed, frequency}
    description: str
```

**Key clarifications**:

- **Static randomization**: Shape varies between benchmark runs
- **Runtime randomization**: Small jitter per simulation run
- Engineer receives YAML with exact positions, moving parts, DOFs, **motors included**

---

## 4. `plan.md` Structure for Engineering *(NEW)*

*MISSING: `plan.md` is not in the `engineer/` template repo.*

```markdown
# Engineering Plan
## 1. Solution Overview
## 2. Parts List
  - Part name, function, geometry, mating points, manufacturing method, material, dimensions
## 3. Assembly Strategy
  - How parts connect, mounting points
## 4. Cost & Weight Budget
  - max_unit_cost, max_weight from objectives.yaml
  - Per-part breakdown
## 5. Risk Assessment
  - Failure modes, mitigations, runtime randomization considerations
```
<!-- note: also validation for markdown! -->
---

<!--
## 5. Review Structure Hardened *(NEW)*

Reviews require **YAML frontmatter**:

```yaml
decision: accepted | rejected | confirm_plan_refusal | reject_plan_refusal
```

- `confirm_plan_refusal` / `reject_plan_refusal` ONLY valid if CAD agent refused the plan
- Reviews stored in `/reviews/` folder
-->

---

## 6. File Validation *(NEW)*

### 6.1 Markdown Files

*MISSING: No static validation of markdown structure.*

- Statically validated for structure (bullet points, subheadings)
- Plans refuse submission if they don't match template

### 6.2 Python Files

*PARTIAL: Syntax check is implemented, but full linting (Ruff/Pyright) is missing in routes.*

- Linted, won't pass execution if red errors exist

---

## 7. Tools Updates

### 7.1 New Tool: `preview_design`

*MISSING: Tool not implemented in worker.*

**Purpose**: Render CAD files for visual inspection by Engineer agent  
**Parameters**: `pitch`, `yaw` to look from specific side  
**Note**: Doesn't need to render all 24 pictures

### 7.2 `validate_and_price` Clarification

*MISSING: Build zone bounds check.*

Step 4 changed: "Validate for being in **build zone bounds**" (not just generically "in bounds")

---

## 8. Skill Management *(NEW)*

### 8.1 Git-based Skill Versioning

*PARTIAL: Uses GitPython instead of git2. No merge conflict resolution logic.*

Skills versioned via **public git repo**. Workers handle git operations:

1. `git commit && git push`
2. If push fails (merge conflict) → `git merge`
3. If merge fails → skill creator LLM handles it

**Library**: Use `git2` instead of shell commands  
**Security note**: Workers have GitHub PAT (exception to controller-holds-secrets rule)

### 8.2 Skill Agent Details

*MISSING: No Skill Agent implementation.*

- Runs **asynchronous** to main execution
- Filesystem stored on Railway bucket
- Has endpoint to update skills without container restart
- Skills are **read-only** - for agents

---

## 9. Infrastructure Clarifications

<!--
### 9.1 Controller vs Worker Responsibilities *(CLARIFIED)*

| Node | Responsibilities |
|------|-----------------|
| **Controller** | LLM orchestration, all secrets (except GitHub PAT), job scheduling |
| **Worker** | Linting, formatting, heavy lifting, ephemeral storage |

### 9.2 Repo Code Structure *(NEW)*

*PARTIAL: Directory structure exists, but separate `pyproject.toml` files are missing.*

```
frontend/
worker/
controller/
```

- OpenAPI schema shared
- Worker and controller have **separate** `pyproject.toml` and `uv` locks
-->

<!--
### 9.3 Container Communication *(NEW)*

- Frontend → Controller only
- Worker → Controller only
- Frontend does NOT communicate with Worker **EXCEPT**: GET requests for GLB files from CAD viewer
-->

<!--
### 9.4 Multiple Agents per Machine *(NEW)*

- 4 agents per machine recommended
- Only 1 simulation at a time (prevent OOM/CPU overload)
- Agents write to different `/tmp/` directories
- Cleanup required after finish
-->

### 9.5 Type Safety *(NEW)*

*PARTIAL: Pydantic is used, Beartype is available but usage is sparse.*

- Use **Pydantic** for data models
- Use **Beartype** for hard type checking

---

## 10. Frontend Specifications *(NEW - DETAILED)*

### 10.1 Benchmark Generation Workflow

*MISSING: "+ Create new" button and clearing UI logic.*

1. Add **"+ Create new"** button (replaces history icon) next to "benchmark runs"
2. Clicking clears current UI data (not DB)
3. Enter "benchmark creation" mode
4. Traces section shows textbox for prompting
5. `plan.md` created with **"Start implementation"** button at top
6. UI auto-updates with new models as build123d progresses

### 10.2 Viewing Code, Plans, Reviews, Simulation

Both benchmark generator and engineer viewers should show:

1. Markdown plans
2. Generated code (Python artifacts, not MJCF)
3. Final/starting renders (artifacts)
4. Reasoning traces (streamed in realtime)
5. 3D view

Plus: implementation history

<!--
### 10.3 Component Layout

- 3-column layout with ratio **3:3:6**:
-->

<!--
### 10.4 CAD Viewer *(NEW)*

- Use "Yet Another CAD Viewer" **server-side rendering**
- Integrate into worker
- File format: **GLB** (not STL) - smaller volume

**Frontend exception**: Frontend queries worker directly for GLB files (GET only)
-->

**Future**: WASM + build123d viewer in browser

### 10.5 Frontend Architecture *(UPDATED)*

> "Vite, React. Autogenerated types on git hooks from Controller. **Super-modern, powerful look**"

---

## 11. Typo/Wording Fixes (Minor)

*MOSTLY DONE*

| Before | After |
|--------|-------|
| `paralel` | `parallel` |
| `outweights` | `outweighs` |
| `probaly` | `probably` |
| `asyncronous` | `asynchronous` |
| `separte` | `separate` |
| `Mulitple` | `Multiple` |
| `throught` | `through` |
| `continous` | `continuous` |
| `prerender` | `pre-render` |
| `Norably` | `Notably` |

---

## Implementation Priorities

Based on the scope of changes, suggested implementation order:

### P0 - Critical for Agent Flow

1. `objectives.yaml` full schema implementation
2. Handover file validation (`plan.md`, `todo.md`, `objectives.yaml`)
3. Review YAML frontmatter parsing and validation
4. Build zone bounds validation in `validate_and_price`

### P1 - Frontend

1. Benchmark creation workflow UI
2. 3-column layout with reasoning traces
3. Artifact viewer with file tree
4. CAD viewer integration (GLB via worker)

### P2 - Infrastructure

1. Repo structure alignment (separate pyproject.toml)
2. Pydantic + Beartype enforcement
3. Skill git repo integration
4. Multi-agent per machine with simulation semaphore

### P3 - Agent Improvements

1. `preview_design` tool implementation
2. File structure validation (markdown, Python linting)
3. Plan template enforcement

---

## Files Affected by These Changes

| Area | Files to Create/Modify |
|------|----------------------|
| Schema | `objectives.yaml` schema, `plan.md` templates |
| Validation | Markdown validators, Python linting integration |
| Tools | `preview_design` implementation |
| Frontend | Benchmark creation flow, 3-column layout, CAD viewer |
| Worker | GLB endpoint, simulation semaphore |
| Infrastructure | Repo restructure, type enforcement |
