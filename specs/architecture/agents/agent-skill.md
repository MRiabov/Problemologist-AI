# Agent Skills

## Scope summary

This document consolidates the architecture guidance for skills across prompt assembly, runtime loading, workspace materialization, evaluation, and observability.

It defines the canonical skill source, the runtime copy model, and the boundary between skills and prompts.

## Skill contract

Skills own the durable, reusable guidance that prompts should not have to carry: procedures, repair patterns, domain heuristics, concrete examples, and repeated operational reminders.

Skills are not the prompt. Prompt text should stay short and delegate long workflows, troubleshooting branches, and reusable reasoning patterns to skills.

When a role needs explicit skill support, the role guidance or handoff artifact should name the relevant skill files instead of embedding the procedure again.

The runtime may append a compact generated skill catalog for discoverability when needed, but it must not re-author skill bodies or turn skills into a second prompt system.

## Canonical sources

The checked-in `skills/` tree is the canonical skill source.

Runtime copies are read-only inputs:

1. Codex workspaces materialize the tree into `.agents/skills/`.
2. Controller-backed runtime surfaces expose the same content through `/skills`.
3. `suggested_skills/` is writable staging, not canonical source.
4. Any compact generated skill index is a discoverability aid only, not a second source of truth.

## Loading and access

The runtime reads the mounted or materialized skill tree when constructing an agent session.

Skill sync is startup-configurable, and integration tests may use deterministic local skill paths.

Role prompts and handoff artifacts should identify required skill dependencies when the task depends on them. Common examples include CAD drafting, manufacturing knowledge, and task-specific helper skills.

## Lifecycle

Skill updates are explicit and reviewable, not an implicit side effect of normal agent runs.

The skill-learning loop runs asynchronously from the main execution flow.

Skill revisions should be versioned so observability can correlate a skill version with later task outcomes.

## Recursive improvement loop

Codex-backed skill learning may run as a bounded recursive loop.

The loop keeps the same Codex session alive across follow-up turns instead of rebuilding conversation context from scratch.

The first follow-up turn performs self-analysis. A later follow-up turn drafts or repairs the skill content.

Follow-up output is written to `suggested_skills/` first. Promotion into the checked-in `skills/` tree remains a separate reviewable step.

The loop stays asynchronous relative to the main eval or task run, and it uses durable workspace artifacts such as `journal.md` to preserve the minimum state needed for later turns.

## Authoring and change policy

Skill bodies should be stable, reusable, and specific enough to survive repeated use.

When a rule needs long examples, troubleshooting branches, or repeated operational reminders, it belongs in a skill rather than in prompt text.

Skill updates should be incremental. Avoid overwriting an existing skill from scratch when a narrow edit can preserve the rest of the contract.

Generated skills should remain valid markdown/YAML and conform to the expected skill schema.

Generated skills should also avoid duplicating existing skills or churning the same rows repeatedly.

## Validation and observability

Skill quality is measured by more than syntax.

Relevant signals include:

1. validity of generated `SKILL.md` content,
2. whether later tasks actually reuse the skill,
3. whether the skill adds new capability instead of duplicating an existing one,
4. skill edit and skill-file-read events,
5. performance delta before and after a new skill version,
6. whether the skill version is being exercised by later tasks after a revision.

The observability layer can use confusion signals and downstream outcomes to identify where a skill needs improvement.

## Related architecture surfaces

- [agent-harness.md](./agent-harness.md): workspace materialization, mounts, and runtime startup behavior.
- [artifacts-and-filesystem.md](./artifacts-and-filesystem.md): role read/write surfaces and read-only skill copies in workspaces.
- [prompt-management.md](./prompt-management.md): prompt/skill boundary and compact generated skill index behavior.
- [tools.md](./tools.md): reusable workflows belong in skills instead of expanding prompts.
- [roles.md](./roles.md): role-specific skill dependencies such as CAD drafting and manufacturing knowledge.
- [handover-contracts.md](./handover-contracts.md): handoff artifacts that reference required skills or validation steps.
- [../../migrations/minor/codex-self-improving-skill-loop.md](../../migrations/minor/codex-self-improving-skill-loop.md): migration note for the bounded Codex session-resume loop.
- [../evals-architecture.md](../evals-architecture.md): skill validity, utility, and non-duplication checks.
- [../observability.md](../observability.md): skill edit/read events, effectiveness metrics, and skill-learning feedback.
- [../distributed-execution.md](../distributed-execution.md): startup-configurable skill sync across worker topologies.

## Boundaries

- Prompt assembly lives in `prompt-management.md`.
- Filesystem access rules live in `artifacts-and-filesystem.md`.
- Skill execution and learning signals live in `evals-architecture.md` and `observability.md`.
