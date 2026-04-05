---
title: Prompt Management Unification
status: investigation
agents_affected:
  - benchmark_planner
  - benchmark_plan_reviewer
  - benchmark_coder
  - benchmark_reviewer
  - engineer_planner
  - engineer_plan_reviewer
  - engineer_coder
  - engineer_execution_reviewer
  - electronics_planner
  - electronics_reviewer
added_at: '2026-04-01T13:35:34Z'
---

# Prompt Management Unification

<!-- Investigation doc. No behavior change yet. -->

## Purpose

This migration unifies prompt assembly across controller/API agents and Codex CLI agents.

The target architecture is described in [prompt-management.md](../../architecture/agents/prompt-management.md). This migration describes the work needed to move from the current split prompt builders to that single source model.

The goals are:

1. one PromptManager owns prompt assembly for every backend family,
2. role prompts stay short and role-focused,
3. backend-specific differences live in appendices,
4. shared prompt-context templates are treated as prompt inputs, and
5. skills remain the owner of detailed procedures and domain guidance.

## Problem Statement

The current prompt system has drifted into two separate prompt-building paths:

1. the controller/API runtime renders prompts through `controller/agent/prompt_manager.py` and `controller/prompts.py`, and
2. the Codex workspace runtime still participates in prompt materialization today, but that prompt assembly should move out of `evals/logic/codex_workspace.py` and into `PromptManager`.

That split is manageable for bootstrap work, but it is the wrong long-term shape. It makes prompt optimization happen twice, encourages duplicate prompt text, and hides which pieces are source-of-truth prompt content versus runtime materialization context.

The repository also still mixes several prompt source shapes in `config/prompts.yaml`:

- role prompts for controller-facing agents,
- `codex.*.role_lines` for Codex eval workspaces,
- the legacy `common.code_template` block,
- skill-loading prose that should be runtime-managed instead of hand-authored, and
- helper guidance that should really live in backend appendices or prompt-context files.

At the same time, workspace materialization already copies starter files and prompt-context files from `shared/agent_templates/` and role-scoped starter material from `shared/assets/template_repos/`, but those files are not yet treated as first-class prompt inputs in the architecture.

The result is a prompt surface that is functionally working but structurally duplicated.

## Current-State Inventory

| Area | Current behavior | Why it must change |
| -- | -- | -- |
| `controller/prompts.py` | Loads `config/prompts.yaml` and exposes direct dot-path lookup helpers. | It is only a loader today; the prompt source model still lives in multiple shapes. |
| `controller/agent/prompt_manager.py` | Maps agent names to `config/prompts.yaml` entries, renders Jinja templates, and appends a compact generated skill index when needed. | It should become the single merge point for the unified source model instead of one of several prompt worlds. |
| `evals/logic/codex_workspace.py` | Builds the Codex workspace prompt string directly and still carries hand-authored skill-loading prose. | Prompt assembly should move out of this module; it should only supply runtime context or template variables to `PromptManager`. |
| `controller/agent/nodes/base.py` | Injects runtime context into the controller-side prompt flow. | Runtime context should stay separate from canonical prompt content and be inserted after the shared prompt layers. |
| `config/prompts.yaml` | Mixes controller role prompts, Codex role lines, legacy `common.code_template`, and assorted helper text. | The file needs a unified schema with role prompts plus appendices. |
| `shared/agent_templates/` | Holds prompt-context files and helper scripts, but not as a first-class prompt source. | These files should be treated as prompt inputs because they materially change the agent's starting context. |
| `shared/assets/template_repos/` | Materializes starter workspace content. | It should remain a workspace-context source, not a second prompt source. |
| `worker_light/agent_files/` | Mirrors starter content for compatibility and local inspection. | It is legacy compatibility material and should not be canonical. |
| `shared/skills/` and installed `SKILL.md` files | Hold the detailed procedures and domain guidance. | These are the right place for business logic; prompts should not duplicate them. |

## Target State

01. A single PromptManager owns prompt assembly for both controller/API and Codex runtime families.
02. `config/prompts.yaml` uses one unified prompt source model with structured role prompts and appendices.
03. Prompt blocks in `config/prompts.yaml` are ordered by runtime call order, not by historical file layout.
04. The active prompt inventory is organized as:
    - benchmark planner -> benchmark plan reviewer -> benchmark coder -> benchmark reviewer,
    - engineer planner -> engineer plan reviewer -> engineer coder -> engineer execution reviewer,
    - electronics planner -> electronics reviewer,
    - helper agents after the core graphs in runtime invocation order.
05. `shared/agent_templates/` is treated as first-class prompt-context material.
06. `shared/assets/template_repos/` continues to provide copied starter workspace content, but it is not a competing prompt source.
07. `worker_light/agent_files/` remains a legacy compatibility mirror only.
08. `common.code_template` no longer acts as the canonical prompt source once equivalent content lives in the managed prompt fragments and prompt-context templates.
09. Backend-specific differences are expressed through appendices for `cli_based` and `api_based` families.
10. Prompt optimization happens once against the unified source model, with derived DSPy-optimized prompts treated as a later layer rather than a separate authoring system.

## Required Work

### 1. Restructure the YAML source model

- Replace the current mixed prompt structure in `config/prompts.yaml` with unified role prompts and appendices.
- Keep the role prompts short and role-focused.
- Move backend-specific operational reminders into the appendix layer.
- Reorder prompt blocks so the file mirrors runtime call order.
- Remove or deprecate `common.code_template` as an active source once the same material exists in the managed prompt-context locations.

### 2. Make PromptManager the single merge point

- Update `controller/agent/prompt_manager.py` so it renders from the unified source model for every backend family.
- Keep `controller/prompts.py` as the loader for the YAML source, but stop using the file as a collection of unrelated prompt worlds.
- Ensure the same merge order is used for controller/API and Codex prompt materialization.
- Keep any generated skill index in one place only, so both paths use the same backend-managed skill source.

### 3. Delegate Codex prompt assembly to PromptManager

- Refactor `evals/logic/codex_workspace.py` so it consumes the same base prompt model and backend appendix structure as the controller path.
- Preserve the current Codex-style prompt tone: compact, operational, and workspace-aware.
- Keep the runtime facts in the workspace prompt, but treat them as injected context rather than prompt source.
- Keep the Codex prompt builder aligned with the same source order as the controller renderer.

### 4. Promote prompt-context files to first-class prompt inputs

- Treat `shared/agent_templates/` as prompt-context material, not just a starter-file directory.
- Keep `shared/assets/template_repos/` as the source of copied starter workspace content.
- Keep `worker_light/agent_files/` as a compatibility mirror only.
- Ensure prompt docs and runtime docs describe these files as part of the prompt-context contract.

### 5. Keep skills as the owner of detailed behavior

- Preserve the generated skill index as the discovery layer for deeper procedures when a backend needs one.
- Avoid duplicating skill bodies in prompt text.
- Move any role guidance that is long-lived and reusable into a skill rather than growing the prompt.
- Keep prompts focused on role identity, workspace contract, backend differences, and invocation guidance.

### 6. Align docs and tests with the unified model

- Update the prompt-management architecture doc to describe the unified source model and the backend appendices.
- Update the agent harness and filesystem docs so they point at the unified prompt-context contract.
- Update integration coverage to verify that both controller/API and Codex materialize prompts from the same base model.
- Preserve prompt block order as a migration invariant: the unified source should stay ordered by runtime call order, including any active electronics/helper roles if they remain first-class.

## Detailed Checklist

### Prompt source model

- [x] Replace the current mixed prompt structure in `config/prompts.yaml` with a unified `role_prompts` plus `appendices` layout.
- [x] Keep the benchmark and engineering role prompts compact, direct, and role-specific.
- [x] Place the role prompts in the same order the runtime calls them.
- [x] Move backend-specific reminders out of the role prompts and into the appropriate appendix branch.
- [x] Remove `common.code_template` from active prompt assembly once the equivalent content lives in managed prompt-context files or is no longer needed.
- [x] Confirm that helper-agent prompt entries and any active electronics roles remain included in the same unified model if they remain first-class.

### PromptManager and controller path

- [x] Update `controller/prompts.py` so it remains the YAML loader, not a second prompt architecture.
- [x] Update `controller/agent/prompt_manager.py` so it is the single merge point for role prompts, appendices, runtime context, and any backend-managed skill index.
- [x] Keep controller/API rendering on the unified source model rather than on controller-only prompt branches.
- [x] Make sure the controller path and Codex path derive any skill index from the same backend-managed skill source.

### Codex workspace path

- [x] Remove prompt assembly from `evals/logic/codex_workspace.py` so it only supplies runtime context, adapter-specific variables, or other non-prompt state to `PromptManager`.
- [x] Preserve the current Codex prompt tone by keeping the rendered text compact and operational inside `PromptManager`.
- [x] Keep runtime facts out of prompt-construction logic outside `PromptManager`; they should be injected as runtime context or template variables.
- [x] Ensure the Codex runtime and controller runtime both rely on the same merge order, with no separate prompt-building logic in Codex workspace code.

### Prompt-context files and workspace material

- [x] Treat `shared/agent_templates/` as first-class prompt-context material.
- [x] Keep `shared/assets/template_repos/` as copied starter workspace content, not as a competing prompt source.
- [x] Keep `worker_light/agent_files/` as legacy compatibility material only.
- [x] Update the architecture docs so they describe prompt-context files as part of the prompt contract.

### Skills and prompt boundaries

- [ ] Preserve skills as the place where detailed behavior, procedures, and domain guidance live.
- [ ] Avoid duplicating skill bodies in prompt text.
- [ ] Move any long-lived reusable role guidance into a skill instead of growing the prompt.
- [ ] Keep prompts focused on role identity, workspace contract, backend differences, and invocation guidance.

### Docs, tests, and validation

- [x] Update `specs/architecture/agents/prompt-management.md` to describe the unified prompt source model and appendix structure.
- [x] Update `specs/architecture/agents/agent-harness.md` and `specs/architecture/agents/artifacts-and-filesystem.md` so they point at the unified prompt-context contract.
- [x] Update integration coverage to verify that controller/API and Codex materialize prompts from the same base model.
- [x] Confirm any active electronics/helper roles remain included in the same unified model if they remain first-class.
- [x] Add or refresh tests that detect legacy `common.code_template` usage in active prompt assembly.
- [x] Validate the migration against the narrowest relevant integration slice before widening to broader coverage.

## Non-Goals

- Do not change the actual agent roles or backend selection behavior.
- Do not fold DSPy optimization into the base prompt migration.
- Do not expand prompts to carry business logic that belongs in skills.
- Do not turn `shared/agent_templates/` into an execution layer.
- Do not remove compatibility templates before the new source model is in place.

## Sequencing

The safe order is:

1. Introduce the unified prompt schema and runtime ordering rules in `config/prompts.yaml`.
2. Update `shared/agent_templates/` and related workspace material so prompt-context files are clearly part of the contract.
3. Refactor `controller/agent/prompt_manager.py` to be the shared renderer, and remove prompt building from `evals/logic/codex_workspace.py`.
4. Move legacy prompt prose out of `common.code_template` and into the unified prompt-context sources or remove it.
5. Update docs and integration tests to prove the new assembly shape in both backend families.

## Acceptance Criteria

1. `config/prompts.yaml` is organized around unified role prompts and appendices, with prompt blocks ordered by runtime call order.
2. The controller/API and Codex prompt paths share the same base prompt source model.
3. The only meaningful backend differences are the backend appendix and runtime-generated context.
4. `shared/agent_templates/` is treated as prompt-context input and appears in the documented contract.
5. `shared/assets/template_repos/` remains a starter-workspace source, not a second prompt-authoring system.
6. `worker_light/agent_files/` is not treated as canonical prompt source.
7. `common.code_template` is no longer part of active prompt assembly.
8. Skills remain the place where detailed procedures and domain guidance live.
9. The integration suite can prove that the unified prompt model renders correctly in both controller/API and Codex paths.

## File-Level Change Set

The implementation should touch the smallest set of files that actually enforce the new contract:

- `config/prompts.yaml`
- `controller/prompts.py`
- `controller/agent/prompt_manager.py`
- `evals/logic/codex_workspace.py`
- `controller/agent/nodes/base.py`
- `shared/agent_templates/**`
- `shared/assets/template_repos/**`
- `shared/skills/**` or the installed `SKILL.md` sources, if any skill references need to move from prompt text into skills
- `specs/architecture/agents/prompt-management.md`
- `specs/architecture/agents/agent-harness.md`
- `specs/architecture/agents/artifacts-and-filesystem.md`
- prompt-materialization integration tests and seed-workspace checks
