# Feb 10 Round 4 - Prompt/Planning Validity Review

## Scope

This review records the planning and prompt infrastructure risks that could prevent an agent from producing a valid planning output or meeting downstream validation expectations.

## Findings

<!-- ### 1. Skill Path Mismatch in Planner Prompt

- The planner prompt in `config/prompts.yaml` instructs reading:
- `skills/build123d_cad_drafting_skill/SKILL.md`
- `skills/manufacturing-knowledge/SKILL.md`
- The repo only contains `skills/references/` and the skill files live under `/home/maksym/.codex/skills/...`.
- Impact: If the planner follows the prompt literally, required skill guidance will be missing or unreadable, increasing plan quality risk.
- Suggested fix: Align prompt paths to the actual skill locations, or vendor the skills into the repo under `skills/` to match the prompt. -->

<!-- Incorrect - the agent actually fetches skills from a git repo. That said, isn't this already done? -->
<!-- 
### 2. Absolute Path Mismatch for Manufacturing Config and Available Parts

- The planner and engineer prompts reference `/config/manufacturing_config.yaml` and `/skills/references/available_parts_and_tooling.md`.
- In this repo, the paths are `config/manufacturing_config.yaml` and `skills/references/available_parts_and_tooling.md`.
- Impact: If the agent’s FS root is the repo root, absolute paths will fail. Missing config can lead to invalid cost/weight estimates and unavailable-part usage.
- Suggested fix: Use repo-relative paths in prompts or ensure the runtime FS root maps `/` to the repo root. -->

<!-- Incorrect again - the agent actually (should) fetch skills from a git repo. That said, isn't this already done? -->

Response: Skills are synced in the worker on startup via `worker/skills/sync.py` (called in `worker/app.py`), gated by `GIT_REPO_URL`. If that env var is not set or git is unreachable, sync is skipped. The runtime filesystem mounts skills at `/skills`, so prompts should target `/skills/...` to align with the mounted path.

### 3. objectives.yaml Schema Mismatch

- The prompt example in `config/prompts.yaml` shows a flat schema with top-level fields like `goal_zone`, `forbid_zones`, `build_zone`, `max_unit_cost`, and `max_weight`.
- The enforced schema in `shared/models/schemas.py` requires nested structure:
- `objectives.goal_zone`, `objectives.forbid_zones`, `objectives.build_zone`
- `constraints.max_unit_cost`, `constraints.max_weight`
- Plus required `moved_object`, `simulation_bounds`, and `randomization`.
- Impact: A planner or benchmark generator that follows the prompt example will produce an `objectives.yaml` that fails validation in `worker/utils/file_validation.py`.
- Suggested fix: Update the prompt example to match the Pydantic schema or relax validation to accept the legacy flat schema.

User review: it should be as in desired_architecture.md document, search for "```yaml" in that document. That template should be pre-populated at startup for the planner.
Response: Updated `config/prompts.yaml` to use the objectives template from `desired_architecture.md` and to state that `objectives.yaml` is pre-populated at startup.

### 4. COTS Search Requirement Not Operationalized in Planner Prompt

- The planner prompt requires invoking a `cots_search` subagent for each COTS item.
- There is no explicit tool invocation interface provided in the planner prompt itself.
- Impact: The planner may skip this step or be unable to execute it, leading to missing part numbers and catalog pricing in `plan.md`.
- Suggested fix: Provide clear tool invocation instructions or integrate the cots tool into the planner’s tool list.

### 5. plan.md Validation Ambiguity

- `worker/utils/file_validation.py` validates plan structure by checking for section names anywhere in headings, which is lenient.
- `worker/utils/markdown_validator.py` expects stricter numbered headings like `## 1. Solution Overview`, etc.
- Impact: A plan that passes the lenient validator may fail the stricter validator if it is used in other parts of the pipeline.
- Suggested fix: Standardize on one validator and align the prompt template to that validator’s expectations.

User review: should be strict. Additionally, where the lists are required, enforce lists too.
Response: Updated the planner prompt to require exact `## 1..5` headings and to use explicit list/table formats for parts, assembly steps, cost breakdown, and risks. If you want hard enforcement, we still need to update the markdown validator logic.

### 6. todo.md Checkbox Format Not Enforced in Prompt

- The TODO validator in `worker/utils/markdown_validator.py` requires checkbox patterns like `- [ ]`.
- The planner prompt does not explicitly require checkbox formatting.
- Impact: A planner output without checkboxes may fail validation if the TODO validator is active.
- Suggested fix: Add checkbox format requirements to the planner prompt and/or provide a TODO template.

User review: must be enforced. Additionally, the CAD agent solution can not be submitted if any of checkboxes is "[ ]". Must be either "[x]"(completed) or "[-]" (skipped)
Response: Updated planner + engineer prompts to require that no `[ ]` remain before submission and to use `[x]` or `[-]`. Current validator allows `[ ]`, `[x]`, `[X]`, `[/]`, so enforcement will require a code change.

## Summary

- Planning can run, but several mismatches can cause invalid output or missing data that later steps require.
- The highest risk issues are the `objectives.yaml` schema mismatch and path mismatches to skills and config.

## Suggested Next Steps

1. Update `config/prompts.yaml` to correct filesystem paths.
2. Update `objectives.yaml` examples in prompts to match the Pydantic schema.
3. Decide which plan/todo validators are authoritative, then align templates accordingly.
4. Expose or document the `cots_search` tool for planner usage.
