# Agent artifacts and filesystem

## Scope summary

- Primary focus: runtime-agnostic workspace artifacts, file ownership, starter templates, and path permissions.
- Defines which files exist in an agent workspace and which roles may read or write them.
- Covers immutable control files, review manifest naming, and the static file-validation contract.
- Use this file for filesystem layout, artifact ownership, and permission-policy decisions.
- Runtime harness, debug CLI-provider mode, prompt generation, and skill loading live in [agent-harness.md](./agent-harness.md) and [agent-skill.md](./agent-skill.md).

## Filesystem

Agent workspaces live inside sandboxed container filesystems.

The filesystem rules are:

1. Agent access is sandbox-bound. An agent must not have read, write, or execution access outside its assigned sandbox/workspace root, except through explicitly mounted read-only runtime inputs that are part of that sandbox contract.
2. Path traversal outside the workspace root is a deterministic error.
3. Workspace-relative paths are the canonical file contract for paths inside the sandbox.

## Templates

Reusable starter files and prompt-context artifacts are defined once in `shared/agent_templates/`, which is part of the prompt-context contract.
The shared starter set includes `.admin/clear_env.py`, which resets the current
seeded workspace in place without changing the conversation context.
PromptManager consumes these prompt-context inputs when it materializes the runtime prompt.
For CLI-provider-backed sessions, the runtime materializes the checked-in skill tree into `.agents/skills/` in the workspace. That copy is read-only runtime context, not canonical source. All CLI-provider implementations follow this same workspace-materialization contract. See [agent-skill.md](./agent-skill.md) for the source-tree and promotion contract.

Skill-training sessions may additionally materialize `suggested_skills/` as a writable session-local worktree/checkpoint seeded from the approved `skills/` tree. That overlay is session-scoped, not canonical source, and the training loop should read it first when it exists. Publication back into canonical `skills/` happens through a separate promotion flow.

Role-specific planner scaffolds remain in `shared/assets/template_repos/` and are copied into each workspace before node entry.
`worker_light/agent_files/` is a legacy compatibility mirror for bootstrap and local inspection, not the canonical source of truth.

Template files are intentional source artifacts, not ad hoc runtime defaults
or symlinks.

## Initial files for each agent and read-write permissions

The agent-specific workspace surface is role-scoped.

Representative examples:

- Engineering Planner:
  - read: `skills/**`, `utils/**`, `benchmark_definition.yaml`, `benchmark_assembly_definition.yaml`, `benchmark_script.py`, `benchmark_plan_evidence_script.py`, `benchmark_plan_technical_drawing_script.py`, `solution_plan_evidence_script.py`, `solution_plan_technical_drawing_script.py`, `plan.md`, `todo.md`, `journal.md`, `renders/**`
  - write: `plan.md`, `todo.md`, `journal.md`, `assembly_definition.yaml`, `benchmark_definition.yaml`, `solution_plan_evidence_script.py`, `solution_plan_technical_drawing_script.py`
- Engineering Coder:
  - read: `skills/**`, `utils/**`, `benchmark_script.py`, `benchmark_plan_evidence_script.py`, `benchmark_plan_technical_drawing_script.py`, `solution_plan_evidence_script.py`, `solution_plan_technical_drawing_script.py`, `plan.md`, `todo.md`, `benchmark_definition.yaml`, `assembly_definition.yaml`, `reviews/**`, `renders/**`
  - write: `solution_script.py`, additional `*.py` implementation files, `todo.md`, `journal.md`, `renders/**`, `plan_refusal.md`
- Benchmark Planner:
  - read: `skills/**`, `utils/**`, `benchmark_plan_evidence_script.py`, `benchmark_plan_technical_drawing_script.py`, `plan.md`, `todo.md`, `journal.md`, `renders/**`
  - write: `plan.md`, `todo.md`, `journal.md`, `benchmark_definition.yaml`, `benchmark_assembly_definition.yaml`, `benchmark_plan_evidence_script.py`, `benchmark_plan_technical_drawing_script.py`
- Benchmark Coder:
  - read: `skills/**`, `utils/**`, `plan.md`, `todo.md`, `benchmark_definition.yaml`, `benchmark_assembly_definition.yaml`, `benchmark_script.py`, `benchmark_plan_evidence_script.py`, `benchmark_plan_technical_drawing_script.py`, `reviews/**`, `renders/**`
  - write: `benchmark_script.py`, additional `*.py` implementation files, `todo.md`, `journal.md`, `renders/**`, `plan_refusal.md`
- Reviewer roles:
  - read: the stage-owned planner/coder artifacts plus the authored source and planner drafting scripts for that stage once they exist (`benchmark_script.py`, `benchmark_plan_evidence_script.py`, and `benchmark_plan_technical_drawing_script.py` for benchmark execution review; `solution_script.py`, `solution_plan_evidence_script.py`, and `solution_plan_technical_drawing_script.py` for engineering execution review), `renders/**`, and `journal.md`
  - write: stage-scoped `reviews/*.yaml` files only
- COTS Search subagent:
  - read: `parts.db`, COTS query helpers/CLI, and the caller-provided request string
  - write: structured COTS result payload returned to the caller

## System-only metadata

`.manifests/**` is reserved for deterministic handover/state metadata written
by backend utilities.

Manifest ownership summary:

| Artifact | Writer / owner | Trigger | Path |
| -- | -- | -- | -- |
| Render bundle index | rendering producer | Render job completion for a published preview or simulation evidence bundle | `renders/render_index.jsonl` |
| Render metadata manifest | rendering producer | Render job completion for a published preview or simulation evidence bundle | `renders/<bundle>/render_manifest.json` |
| Benchmark plan evidence script | Benchmark Planner | planner drafting submission | `benchmark_plan_evidence_script.py` |
| Benchmark technical drawing script | Benchmark Planner | planner drafting submission | `benchmark_plan_technical_drawing_script.py` |
| Engineering plan evidence script | Engineering Planner | planner drafting submission | `solution_plan_evidence_script.py` |
| Engineering technical drawing script | Engineering Planner | planner drafting submission | `solution_plan_technical_drawing_script.py` |

<!-- FIXME: consider moving render metadata manifests into `.manifests/` in a future refactor so render metadata and handoff metadata share one backend-owned manifest bucket. The root renders/render_manifest.json path stays a compatibility alias only. -->

| Benchmark plan-review manifest | backend runtime utility invoked by `submit_plan()` | Successful `Benchmark Planner` `submit_plan()` | `.manifests/benchmark_plan_review_manifest.json` |
| Engineering plan-review manifest | backend runtime utility invoked by `submit_plan()` | Successful `Engineering Planner` `submit_plan()` | `.manifests/engineering_plan_review_manifest.json` |
| Benchmark review manifest | backend runtime utility invoked by `submit_for_review(...)` | Successful benchmark `submit_for_review(...)` | `.manifests/benchmark_review_manifest.json` |
| Engineering execution-review handoff manifest | backend runtime utility invoked by `submit_for_review(...)` | Successful engineering `submit_for_review(...)` | `.manifests/engineering_execution_handoff_manifest.json` |
| Electronics review manifest | backend runtime utility invoked by `submit_for_review(...)` | Successful electronics `submit_for_review(...)` | `.manifests/electronics_review_manifest.json` |

The agent-facing tools are the submission triggers. The actual manifest write
happens in the backend runtime utility, and `.manifests/**` remains
inaccessible to LLM roles.

The engineering execution-review manifest is the same file on both sides of
the handoff:

- the coder writes it by successfully calling `submit_for_review(...)`
- the Engineering Execution Reviewer later reads it as the latest-revision
  entry gate

Reviewer-stage manifest filenames are explicit and role-scoped:

1. `.manifests/benchmark_plan_review_manifest.json`
2. `.manifests/benchmark_review_manifest.json`
3. `.manifests/engineering_plan_review_manifest.json`
4. `.manifests/engineering_execution_handoff_manifest.json`
5. `.manifests/electronics_review_manifest.json`

Reviewer persistence filenames are explicit and role-scoped:

01. `reviews/benchmark-plan-review-decision-round-<n>.yaml`
02. `reviews/benchmark-plan-review-comments-round-<n>.yaml`
03. `reviews/benchmark-execution-review-decision-round-<n>.yaml`
04. `reviews/benchmark-execution-review-comments-round-<n>.yaml`
05. `reviews/engineering-plan-review-decision-round-<n>.yaml`
06. `reviews/engineering-plan-review-comments-round-<n>.yaml`
07. `reviews/engineering-execution-review-decision-round-<n>.yaml`
08. `reviews/engineering-execution-review-comments-round-<n>.yaml`
09. `reviews/electronics-review-decision-round-<n>.yaml`
10. `reviews/electronics-review-comments-round-<n>.yaml`

## Locking rule

Before planner submission, planner roles may edit planner-owned files.

After planner submission is accepted:

1. benchmark-side `benchmark_definition.yaml` and `benchmark_assembly_definition.yaml` become read-only for benchmark Coder/Reviewer,
2. `benchmark_plan_evidence_script.py` and `benchmark_plan_technical_drawing_script.py` become read-only for benchmark Coder/Reviewer and are also available in engineer intake as read-only context,
3. the same `benchmark_assembly_definition.yaml` and `benchmark_script.py` are also available in engineer intake as read-only context,
4. engineer-side `assembly_definition.yaml` becomes read-only for engineering Coder/Reviewer,
5. `solution_plan_evidence_script.py` and `solution_plan_technical_drawing_script.py` become read-only for engineering Coder/Reviewer and the Engineering Execution Reviewer,
6. only replanning can mutate planner-owned files.

## Template auto-validation

Where possible, templates have a validation schema.

YAML templates are schema-validated when they are materialized into a
workspace, so starter drift is caught at source rather than after a node has
already started.

## `agents_config.yaml` (path permissions policy)

To prevent permission drift between the static file contract and runtime behavior, we define a centralized policy file at `config/agents_config.yaml`.

The policy uses gitignore-style glob patterns (`*`, `**`, path prefixes) for filesystem access control.

Rules:

1. Every read/write/edit/upload/download operation is checked against this policy in `FilesystemMiddleware`.
2. `deny` takes precedence over `allow`.
3. If a path is not matched by `allow`, access is denied by default.
4. Agent-specific rules override `defaults`.
5. Reviewer roles get `write/edit` tools, but policy only allows writes to their stage-specific persisted review YAML pairs.
6. `.manifests/**` is non-overridable deny for all LLM agent roles; only backend runtime utilities may access it.
7. No agent role may be granted access to filesystem paths outside its sandbox/workspace root.
8. `config/agents_config.yaml` also owns preview-render modality policy under top-level `render: {rgb, depth, segmentation}` and the motion cadence/tolerance policy for benchmark planner, engineer planner, and engineer coder layers.
9. Those flags control whether build123d/VTK-backed preview artifacts are persisted into `renders/**` for each modality; they do not change worker routing or backend selection policy. The motion policy controls waypoint frequency and tolerance budgets but does not relocate the contract structure out of the YAML artifacts. The bundle subdirectory still reflects the workflow that produced it.

## Immutability validation

We assert that control files are not edited by coder agents and are edited only by planner agents.

Control-file ownership split:

01. `benchmark_definition.yaml` owns benchmark/task definition and benchmark fixture metadata (`benchmark_parts`).
02. `benchmark_assembly_definition.yaml` owns benchmark-owned fixture structure, motion metadata, and benchmark-side implementation details.
03. `benchmark_script.py` owns benchmark-owned geometry composition and read-only benchmark preview context.
04. `benchmark_plan_evidence_script.py` owns benchmark planner drafting evidence geometry.
05. `benchmark_plan_technical_drawing_script.py` owns benchmark planner technical-drawing exports.
06. `solution_script.py` owns engineer-planned solution geometry and implementation code.
07. `solution_plan_evidence_script.py` owns engineering planner drafting evidence geometry.
08. `solution_plan_technical_drawing_script.py` owns engineering planner technical-drawing exports.
09. `assembly_definition.yaml` owns engineer-planned solution structure, costing inputs, and motion metadata.
10. `payload_trajectory_definition.yaml` owns engineer-coder higher-resolution payload trajectory and contact proof; it refines the coarse planner forecast, must not contradict it, and must preserve the approved build-zone start and goal-zone finish semantics.
11. We do not duplicate engineer solution metadata into `benchmark_definition.yaml`.

## File updates

Files are written directly to the worker container and persisted to the observability database, not to the controller filesystem.

The controller parses the tool call locally and then dispatches the request to the worker that is responsible for the relevant file operation.

## Utils files

The agent has a set of utils - python scripts (files) that the agent can import from.

These are explicitly unwritable by the filesystem middleware.
