# Agent Skills

## Scope summary

- Primary focus: canonical skill-tree source, authoring contract, and workspace materialization.
- Defines the skill repository boundary, lifecycle, and improvement loop.
- Use this file for anything that changes how skills are stored, loaded, or learned by agents.

## Skills

Skills own the durable, reusable guidance that prompts should not have to carry: procedures, repair patterns, domain heuristics, and concrete examples.

The rule of thumb is simple: prompts frame the work; skills explain the work.

## Skill source contract

Skill loading is runtime-managed, not prompt-authored.

For Codex CLI-backed sessions, the repo-local canonical skill tree is `.agents/skills/`. Do not confuse that checkout directory with the runtime workspace path the agent sees after materialization; the backend exposes the workspace-visible skill mount separately. The prompt does not need to explain where skills live or restate the discovery workflow.

The skill repository boundary is explicit:

1. `.agents/skills/` is the repo-local source tree for Codex CLI-backed runs, not the runtime workspace mount.
2. `skills/` remains the workspace-visible compatibility mount used by controller-backed or legacy integrations until that path is retired.
3. `.codex/skills/` is a repo-local Codex-only overlay for local debugging/editorial workflows and is not part of the runtime agent skill contract.
4. `suggested_skills/` is a writable sidecar output area for proposed/new skills, not the canonical skill mount that normal agents should read from.
5. Runtime agents should not be taught to treat `.codex/skills/`, `suggested_skills/`, or any other agent-specific skill store as interchangeable with the canonical tree.

## Skill lifecycle

1. The agent can read skills that exist in the canonical tree.
2. The skill creator / learner may update the canonical tree subject to safety limits.
3. Learned skills are versioned and persisted to observability.
4. The skill agent runs asynchronously from the main execution flow.

## Prompt boundary

PromptManager may append a compact generated catalog for discoverability when a backend needs it, but it must not re-author skill bodies or turn skills into a second prompt system.
