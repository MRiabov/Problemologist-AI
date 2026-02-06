# Phase 0# Research: VLM CAD Agent (Engineer)

## Agent Role & Personas

The "Engineer" is an agent graph (LangGraph) consisting of three primary roles to ensure high-quality, manufacturable solutions.

### 1. The Architect (Planner)

- **Responsibility**: Designs the high-level solution and decomposes it into a sequence of actionable tasks.
- **Output**: Writes to `todo.md` and `plan.md`.
- **Constraint**: Must use standard engineering principles and follow the user's cost/weight constraints.

### 2. The Engineer (Implementer)

- **Responsibility**: Translates the plan into `build123d` Python code.
- **Tools**: Standard OS tools (`ls`, `write`) and domain utilities (`simulate`, `validate_and_price`).
- **Autonomy**: After proving a plan is impossible, the Engineer can refuse the plan, forcing an Architect re-plan.

### 3. The Critic (Reviewer)

- **Responsibility**: Reviews the code and simulation results (video, telemetry).
- **Criterion**: Assesses stability, manufacturability, and goal achievement.
- **Power**: Can reject the solution and send it back to the Engineer with feedback.

## Operational Paradigms

### Episodic Memory: The Journal

**Decision**: Use a structured `journal.md` for episodic memory.
**Rationale**:

- Stores **Intent -> Result -> Reflection -> Next Step**.
- Delimited by Markdown headings to allow "progressive disclosure" for learner agents and human debugging.
- Prevents context overflow by serving as the source for "token compression" (summarization).

### Continuous Improvement: Skills

**Decision**: Sidecar **Learner Agent**.
**Rationale**:

- Runs asynchronously after successful episodes.
- Scrutinizes the `journal.md` to identify struggles and breakthroughs.
- Updates repository-level `SKILL.md` files (Git versioned).

### Agent-Native Feedback

**Decision**: Feedback in Markdown.
**Rationale**:

- Agents perform significantly better with Markdown than JSON.
- Simulation results, cost reports, and linting errors are formatted as readable Markdown reports.
