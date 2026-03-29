# Agent artifacts and filesystem

## Scope summary

- Primary focus: runtime-agnostic workspace artifacts, file ownership, starter templates, and path permissions.
- Defines which files exist in an agent workspace and which roles may read or write them.
- Covers immutable control files, review manifest naming, and the static file-validation contract.
- Use this file for filesystem layout, artifact ownership, and permission-policy decisions.
- Runtime harness, debug Codex mode, prompt generation, and skill loading live in [agent-harness.md](./agent-harness.md).

## Filesystem

Agent workspaces live inside sandboxed container filesystems.

The filesystem rules are:

1. Agent access is sandbox-bound. An agent must not have read, write, or execution access outside its assigned sandbox/workspace root, except through explicitly mounted read-only runtime inputs that are part of that sandbox contract.
2. Path traversal outside the workspace root is a deterministic error.
3. Workspace-relative paths are the canonical file contract for paths inside the sandbox.

## Templates

Reusable starter files are defined once in `shared/agent_templates/common/`.

Role-specific planner scaffolds remain in their existing template-repo locations and are copied into each workspace before node entry. `worker_light/agent_files/` mirrors the same starter content for runtime bootstrap and local inspection.

Template files are intentional source artifacts, not ad hoc runtime defaults or symlinks.

## Initial files for each agent and read-write permissions

The agent-specific workspace surface is role-scoped.

Representative examples:

- Engineering Planner:
  - read: `skills/**`, `utils/**`, `benchmark_definition.yaml`, `plan.md`, `todo.md`, `journal.md`
  - write: `plan.md`, `todo.md`, `journal.md`, `assembly_definition.yaml`, `benchmark_definition.yaml`
- Engineering Coder:
  - read: `skills/**`, `utils/**`, `plan.md`, `todo.md`, `benchmark_definition.yaml`, `assembly_definition.yaml`, `reviews/**`, `renders/**`
  - write: `script.py`, additional `*.py` implementation files, `todo.md`, `journal.md`, `renders/**`, `plan_refusal.md`
- Benchmark Planner:
  - read: `skills/**`, `utils/**`, `plan.md`, `todo.md`, `journal.md`
  - write: `plan.md`, `todo.md`, `journal.md`, `benchmark_definition.yaml`, `benchmark_assembly_definition.yaml`
- Benchmark Coder:
  - read: `skills/**`, `utils/**`, `plan.md`, `todo.md`, `benchmark_definition.yaml`, `benchmark_assembly_definition.yaml`, `reviews/**`, `renders/**`
  - write: `script.py`, additional `*.py` implementation files, `todo.md`, `journal.md`, `renders/**`, `plan_refusal.md`
- Reviewer roles:
  - read: the stage-owned planner/coder artifacts plus `renders/**` and `journal.md`
  - write: stage-scoped `reviews/*.yaml` files only
- COTS Search subagent:
  - read: `parts.db`, COTS query helpers/CLI, and the caller-provided request string
  - write: structured COTS result payload returned to the caller

## System-only metadata

`.manifests/**` is reserved for deterministic handover/state metadata written by backend utilities.

Reviewer-stage manifest filenames are explicit and role-scoped:

1. `.manifests/benchmark_plan_review_manifest.json`
2. `.manifests/benchmark_review_manifest.json`
3. `.manifests/engineering_plan_review_manifest.json`
4. `.manifests/engineering_execution_review_manifest.json`
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
2. the same `benchmark_assembly_definition.yaml` is also available in engineer intake as read-only context,
3. engineer-side `assembly_definition.yaml` becomes read-only for engineering Coder/Reviewer,
4. only replanning can mutate planner-owned files.

## Template auto-validation

Where possible, templates have a validation schema.

YAML templates are schema-validated when they are materialized into a workspace, so starter drift is caught at source rather than after a node has already started.

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
8. `config/agents_config.yaml` also owns preview-render modality policy under top-level `render: {rgb, depth, segmentation}`.
9. Those flags control whether build123d/VTK-backed preview artifacts are persisted into `renders/` for each modality; they do not change worker routing or backend selection policy.

## Immutability validation

We assert that control files are not edited by coder agents and are edited only by planner agents.

Control-file ownership split:

1. `benchmark_definition.yaml` owns benchmark/task definition and benchmark fixture metadata (`benchmark_parts`).
2. `benchmark_assembly_definition.yaml` owns benchmark-owned fixture structure, motion metadata, and benchmark-side implementation details.
3. `assembly_definition.yaml` owns engineer-planned solution structure, costing inputs, and motion metadata.
4. We do not duplicate engineer solution metadata into `benchmark_definition.yaml`.

## File updates

Files are written directly to the worker container and persisted to the observability database, not to the controller filesystem.

The controller parses the tool call locally and then dispatches the request to the worker that is responsible for the relevant file operation.

## Utils files

The agent has a set of utils - python scripts (files) that the agent can import from.

These are explicitly unwritable by the filesystem middleware.
