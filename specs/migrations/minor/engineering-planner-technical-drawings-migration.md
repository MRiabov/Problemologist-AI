# Engineering Planner Technical Drawings Migration

<!-- Migration tracker. Check items conservatively as implementation lands. -->

## Purpose

This migration implements the planner-authored technical drawing layer
described in [Engineering Planner Technical Drawings](../../architecture/agents/engineering-planner-technical-drawings.md).

The target contract is:

1. the Engineering Planner can emit a structured drafting layer as part of the
   handoff, and the Benchmark Planner mirrors the same artifact pattern with
   benchmark-prefixed filenames,
2. `plan.md` remains the human-readable mechanism explanation,
3. `assembly_definition.yaml` carries the machine-readable drafting section,
4. `solution_plan_evidence_script.py` / `solution_plan_technical_drawing_script.py`
   carry the engineering drafting evidence and drawing companion, while
   `benchmark_plan_evidence_script.py` / `benchmark_plan_technical_drawing_script.py`
   carry the benchmark mirror,
5. `preview_drawing()` renders the drafting package for inspection,
6. the drafting layer is enabled or suppressed by prompt policy in
   `config/agents_config.yaml`,
7. `inspect_media(...)` remains the only visual-evidence tool,
8. full GD&T is deferred, and
9. the migration is verified end to end through real integration tests.

The migration is intentionally narrow. The planner is not becoming the final
CAD author; it is becoming more specific about the geometry that the engineer
must preserve.

## Problem Statement

The current Engineering Planner handoff is strong on narrative and weak on
geometric binding.

1. The planner can explain the mechanism, but it often leaves the engineer to
   guess the critical interface geometry.
2. The handoff files already separate `plan.md`, `todo.md`, and
   `assembly_definition.yaml`, but they do not yet provide a dedicated drafting
   layer.
3. 3D preview is useful, but it does not answer the planner-specific question
   of which dimensions and views are binding.
4. A raw SVG or ad hoc vector sketch would be the wrong abstraction. The repo
   already uses build123d, so the runtime should derive technical drawings from
   structured intent instead of asking the model to hand-author low-level
   vector geometry.
5. The feature must stay prompt-policy driven. If PromptManager does not
   inject the drafting appendix, the planner does not use the drafting
   contract.

The practical outcome is simple: the planner needs a drafting contract, not a
new CAD language.

## Current-State Inventory

| Area | Current behavior | Why it must change |
| -- | -- | -- |
| `config/agents_config.yaml` | Owns execution and render policy, but has no planner drafting mode contract. | The drafting layer must be config-gated so it can be enabled or omitted without changing the underlying handoff files. |
| `config/prompts.yaml` | Holds role prompts and appendices, but no dedicated planner drafting appendix branch. | PromptManager needs a place to inject drafting instructions without duplicating role prompts. |
| `controller/agent/prompt_manager.py` | Merges prompt fragments, but does not yet conditionally inject drafting instructions. | It must become the single place that adds or omits the drafting appendix based on mode. |
| `evals/logic/codex_workspace.py` | Materializes workspace context without a drafting-specific prompt payload. | It should remain prompt-source agnostic while still passing the required runtime context through. |
| `worker_heavy/utils/preview.py` | Renders 3D preview bundles only. | The runtime needs a 2D technical-drawing companion path. |
| `worker_renderer/api/routes.py` | Persists the current preview bundle contract. | It must accept the drawing preview bundle and any vector sidecars. |
| `scripts/validate_eval_seed.py` and `controller/agent/node_entry_validation.py` | Validate seed and workspace contracts, but not drafting-specific requirements. | They need fail-closed enforcement when the mode requires drafting artifacts, including the planner-owned evidence and drawing scripts. |
| `tests/integration/**` | Exercise the current planner and preview flows, but not the 2D drafting contract. | The migration needs a narrow integration slice for draft mode, preview, and review. |
| `shared/agent_templates/` and `shared/assets/template_repos/` | Hold prompt-context and starter-workspace material. | Any drafting guidance or examples must live there if they are part of the workspace contract, not as ad hoc prompt text. |
| `benchmark_plan_evidence_script.py`, `benchmark_plan_technical_drawing_script.py`, `solution_plan_evidence_script.py`, `solution_plan_technical_drawing_script.py` | Planner-authored drafting evidence and drawing companions. | These files make the build123d plan and technical drawings explicit and readable by downstream roles. |

## Proposed Target State

1. `config/agents_config.yaml` controls whether PromptManager injects the
   drafting appendix. `off` means the planner is not instructed to use
   drafting; `minimal` and `full` mean the appendix is present and the
   reviewer expects it.
2. `assembly_definition.yaml` contains a strict `drafting` section with
   views, datums, binding dimensions, callouts, and notes.
3. `plan.md` remains the narrative explanation; the drafting section is the
   structured geometry contract.
4. `preview_drawing()` renders the drafting package, produces reviewable
   image output and vector sidecars, and persists them under the same
   role-scoped preview bucket model used for other evidence.
5. The implementation may use build123d technical drawing primitives,
   `TechnicalDrawing`, `ExportSVG`, `ExportDXF`, and `project_to_viewport()`
   as implementation details.
6. If the build123d export path is sufficient, `preview_drawing()` may stay
   thin and act mostly as a conversion and attachment bridge. It does not
   need to become a second authoring surface.
7. `inspect_media(...)` remains the review tool for the resulting render
   artifacts.
8. The first release does not add full GD&T.
9. The migration improves plan specificity, reduces coder ambiguity, and
   gives the reviewer something concrete to reject or approve.

## Required Work

### 1. Add the drafting section to the handoff contract

- Add a typed drafting section to `assembly_definition.yaml` and
  `benchmark_assembly_definition.yaml`.
- Keep the schema strict and fail closed.
- Model view families, datums, binding dimensions, callouts, and notes.
- Do not let the drafting section introduce new parts, joints, or motions.
- Do not let it invent geometry that is absent from `plan.md` or the rest of
  the assembly handoff.
- Add the planner-authored evidence and drawing scripts with role-specific
  prefixes: `solution_plan_evidence_script.py` / `solution_plan_technical_drawing_script.py`
  for engineering and `benchmark_plan_evidence_script.py` /
  `benchmark_plan_technical_drawing_script.py` for benchmark planning.

### 2. Gate the appendix in prompt assembly

- Add a planner drafting appendix to `config/prompts.yaml`.
- Make `PromptManager` inject that appendix only when
  `config/agents_config.yaml` says the mode is active.
- Keep the switch boring: if the appendix is omitted, the planner does not
  know about drafting.
- Do not add a second prompt world or a new runtime tool for the mode.
- Apply the same prompt-policy gate to both the Engineering Planner and the
  Benchmark Planner prompt paths.

### 3. Add the 2D preview path

- Add `preview_drawing()` as the planner-facing companion to `preview()`.
- The helper may be a thin render or convert bridge if build123d exports are
  sufficient.
- If SVG or DXF output is enough, keep the implementation thin and let the
  renderer derive images from those vector artifacts.
- Persist the render result under the same role-scoped preview bucket model
  used for other evidence.
- Keep 3D preview semantics unchanged.
- Keep the preview contract compatible with both planner graphs and their
  prefixed drafting scripts.

### 4. Validate the drafting contract

- Update the planner and reviewer gates so drafting artifacts are required
  only when the mode requires them.
- Reject drawings that contradict the plan, invent geometry, or omit binding
  dimensions for critical interfaces.
- Keep the Engineering Coder intake read-only.
- Keep the Benchmark Coder and both execution reviewers read-only for the
  planner drafting scripts.
- Keep validation fail closed if the drafting section is missing or malformed
  when the mode requires it.

### 5. Update docs and integration coverage

- Update the prompt, tool, and architecture docs so they describe the drafting
  layer consistently.
- Add a narrow integration slice that proves the planner can produce the
  drafting contract and the reviewer can inspect it.
- Add assertions for the mode-off path so the planner does not accidentally
  learn the drafting contract when it should not.
- Add coverage for benchmark drafting parity so the benchmark planner and
  benchmark reviewer see the same artifact split.

## Non-Goals

- No full GD&T.
- No replacement for `plan.md`.
- No separate raw-SVG authoring system.
- No change to 3D preview ownership.
- No change to the existing visual-inspection policy.

## Sequencing

The safe order is:

1. Add the architecture and prompt contract for planner drafting.
2. Wire the config-gated appendix into PromptManager.
3. Add `preview_drawing()` and the vector and raster evidence path.
4. Tighten validation and reviewer checks.
5. Add integration coverage and then tune the prompt policy by mode.

## Acceptance Criteria

1. When drafting is off, the planner is not instructed to use the drafting
   contract.
2. When drafting is on, the planner emits valid drafting scripts and a valid
   `assembly_definition.yaml.drafting` or `benchmark_assembly_definition.yaml.drafting`
   section as appropriate.
3. `preview_drawing()` produces inspectable image output and vector sidecars.
4. The reviewer can reject unsupported or contradictory drafting content.
5. The coder can read the drafting layer as read-only context and still
   implement the handoff.
6. The integration suite proves the end-to-end path through the real runtime
   entrypoint.
7. The feature can be rolled back to `off` without changing the underlying
   plan contract.

## Migration Checklist

Use this checklist to track implementation from prompt policy through runtime
validation. Do not close the migration until every unchecked item is either
completed or explicitly waived with a written rationale.

### Contract and docs

- [x] Add the architecture spec for planner technical drawings.
- [x] Add the prompt-side `preview_drawing()` contract to the tools doc.
- [x] Add the desired-architecture index link for the new planner drafting
  spec.
- [ ] Keep the migration note aligned with the architecture doc as the contract
  evolves.

### Prompt policy and handoff schema

- [x] Add the planner drafting appendix to `config/prompts.yaml`.
- [x] Wire the mode gate into `PromptManager` so the appendix is injected or
  omitted from the final prompt.
- [x] Add the strict `drafting` section to `assembly_definition.yaml` and
  `benchmark_assembly_definition.yaml`.
- [x] Ensure the drafting section cannot introduce unsupported parts, joints,
  motions, or geometry claims.
- [x] Add the planner-authored evidence and drawing scripts for both planner
  graphs with the agreed role prefixes.

### 2D preview plumbing

- [x] Add `preview_drawing()` to the runtime tool surface.
- [x] Decide whether the implementation is a dedicated renderer branch or a
  thin conversion bridge over build123d export output.
- [x] Persist raster inspection images and vector sidecars for drawing
  previews.
- [x] Keep the current 3D preview path unchanged.
- [x] Keep the preview contract compatible with the benchmark and engineering
  planner script pairs.

### Validation and review

- [x] Update the planner and reviewer gates so drafting is required only when
  the mode requires it.
- [x] Keep `inspect_media(...)` as the only visual-evidence path.
- [x] Reject drafting packages that contradict the plan or omit binding
  geometry.
- [x] Keep the Engineering Coder, Benchmark Coder, and execution reviewers
  read-only for the drafting scripts.

### Per-role eval seed checklist

The items below are the eval-side additions to `evals/logic/specs.py` and the
seeded workspace fixtures. They keep `scripts/validate_eval_seed.py` and the
eval preflight path fail closed before node execution starts.

- [x] Engineer Planner eval rows include `assembly_definition.yaml.drafting`,
  `solution_plan_evidence_script.py`, and
  `solution_plan_technical_drawing_script.py`; they also include
  `benchmark_plan_evidence_script.py` and
  `benchmark_plan_technical_drawing_script.py` when benchmark drafting mode is
  active.
- [x] Engineer Plan Reviewer eval rows include the same solution drafting
  scripts when engineering drafting mode is active.
- [x] Engineer Coder eval rows include the same solution drafting scripts when
  engineering drafting mode is active, plus the benchmark drafting scripts
  when benchmark drafting mode is active.
- [x] Engineer Execution Reviewer eval rows include the same solution drafting
  scripts when engineering drafting mode is active.
- [x] Electronics Planner eval rows stay on the base engineer planning files;
  they do not gain drafting-specific eval artifacts in this migration.
- [x] Benchmark Planner eval rows include
  `benchmark_assembly_definition.yaml.drafting`,
  `benchmark_plan_evidence_script.py`, and
  `benchmark_plan_technical_drawing_script.py` when benchmark drafting mode is
  active.
- [x] Benchmark Plan Reviewer eval rows include the same benchmark drafting
  scripts when benchmark drafting mode is active.
- [x] Benchmark Coder eval rows include the same benchmark drafting scripts
  when benchmark drafting mode is active.
- [x] Benchmark Reviewer eval rows include the same benchmark drafting scripts
  when benchmark drafting mode is active.

### Integration coverage

- [x] Add a narrow integration slice for the mode-off path.
- [x] Add a narrow integration slice for the mode-on path.
- [x] Add an inspection assertion for the persisted drawing bundle.
- [ ] Add an end-to-end assertion that the reviewer can judge the drawing
  contract from real render artifacts.
- [x] Add benchmark drafting parity coverage.

## File-Level Change Set

The implementation should touch the smallest set of files that actually
enforce the new contract:

- `config/agents_config.yaml`
- `config/prompts.yaml`
- `controller/agent/prompt_manager.py`
- `evals/logic/codex_workspace.py`
- `worker_heavy/utils/preview.py`
- `worker_renderer/api/routes.py`
- `shared/rendering/*`
- `scripts/validate_eval_seed.py`
- `controller/agent/node_entry_validation.py`
- `benchmark_plan_evidence_script.py`
- `benchmark_plan_technical_drawing_script.py`
- `solution_plan_evidence_script.py`
- `solution_plan_technical_drawing_script.py`
- `shared/agent_templates/**`
- `shared/assets/template_repos/**`
- `tests/integration/**`
- `specs/architecture/agents/engineering-planner-technical-drawings.md`
- `specs/architecture/agents/prompt-management.md`
- `specs/architecture/agents/tools.md`
- `specs/desired_architecture.md`
