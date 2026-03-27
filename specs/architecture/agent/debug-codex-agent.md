# Debug Codex Agent Mode

## Scope summary

- This document defines the local Codex-backed backend used for eval debugging and repeated agent runs.
- It is an application mode, not a new benchmark graph or a replacement for the controller-backed runtime.
- It covers prompt generation, workspace materialization, submission contracts, filesystem rules, and failure behavior.
- It applies to the same seeded eval rows that run through `dataset/evals/run_evals.py` and the shared runner in `evals/logic/runner.py`.
- The generalized runtime/backend/model abstraction is documented in [\_bmad-output/planning-artifacts/architecture.md](/home/maksym/Work/proj/Problemologist/Problemologist-AI/_bmad-output/planning-artifacts/architecture.md); this document stays focused on the Codex-specific implementation details and workspace contract.

## Purpose

The debug Codex mode exists to reduce the cost of repeated eval iteration.
Local Codex execution is the default eval-debug path, while the controller-backed paid-provider path remains available when a run must call OpenRouter or Gemini.

This mode preserves the same workspace contract as the rest of the application:

1. Seeded files are materialized into a real workspace on disk.
2. The agent reads a workspace-local prompt file.
3. Planner roles submit through a local shell helper that invokes Python validation under the hood.
4. Reviewer and coder roles persist the same stage-owned artifacts that the normal runtime expects.
5. Verification fails closed if the workspace violates the contract.

## Backend selection

The runner exposes the backend as an explicit mode:

1. `codex` launches a local Codex workspace and runs the task there.
2. `controller` uses the existing HTTP orchestration path and paid model providers.
3. `codex` is the default backend.
4. The backend can be selected through `--call-paid-api`, `--runner-backend`, or `EVAL_RUNNER_BACKEND`.
5. `--call-paid-api` flips the runner to the paid-provider/controller path; `--runner-backend` remains the explicit override.

The debug Codex mode is implemented in the shared runner, so the `dataset/evals/run_evals.py` wrapper inherits the same behavior without a second code path.

## Workspace materialization

The Codex backend materializes a run-local workspace before the agent starts.
The materialized workspace is the source of truth for the session, not the repository root.

The workspace contract is:

1. The workspace root is the current directory for the Codex process.
2. The initial prompt is written to `prompt.md` at the workspace root.
3. Shared boilerplate starter files come from `shared/agent_templates/common/`.
4. Role-specific planner starter files come from the role template repositories under `shared/assets/template_repos/`.
5. Planner workspaces include `scripts/submit_plan.sh` from `shared/agent_templates/codex/`.
6. Planner workspaces do not copy `result.py`, because that file is runtime-owned and not part of the seeded handoff surface.
7. Seed-row artifacts are copied into the workspace before prompt generation.
8. The materialized workspace remains local to the run and is not promoted into a canonical shared root.

The workspace materializer in [dataset/evals/materialize_seed_workspace.py](../../../dataset/evals/materialize_seed_workspace.py) is the inspection helper for this same workspace contract.

## Prompt contract

The prompt is a contract, not a free-form suggestion.
It must describe the workspace in relative-path terms and must not teach the agent to address the filesystem as `/workspace`.

The canonical prompt rules are:

1. The prompt says `Workspace: current directory`.
2. The prompt tells the agent to use workspace-relative paths only.
3. The prompt does not mention `/workspace` as the workspace root.
4. Planner prompts instruct `bash scripts/submit_plan.sh` as the submission command.
5. Coder prompts instruct editing `script.py` and supporting `*.py` files.
6. Reviewer prompts instruct writing stage-specific review artifacts under `reviews/`.
7. The prompt includes the task text, agent name, task ID, and seed dataset name when available.

The prompt builder in [evals/logic/codex_workspace.py](../../../evals/logic/codex_workspace.py) is the canonical definition of that prompt text.

## Role behavior

### Planner roles

Planner roles are the only roles that use the local submission helper.

Planner behavior is:

1. Edit the planner-owned workspace files for the current stage.
2. Keep the workspace relative-path contract intact.
3. Run `bash scripts/submit_plan.sh` from the materialized workspace.
4. Iterate until the helper reports `ok=true` and `status=submitted`.
5. Treat `.manifests/` as system-owned output, not as editable planner input.

Benchmark planners write `benchmark_definition.yaml` and `benchmark_assembly_definition.yaml`. The latter is benchmark-owned handoff context that is copied into downstream engineer sessions as read-only input.
Engineering and electronics planners write `assembly_definition.yaml` plus the shared planning artifacts.

The local helper in [shared/agent_templates/codex/scripts/submit_plan.sh](../../../shared/agent_templates/codex/scripts/submit_plan.sh) invokes the Python submission function that validates the planner files and writes the stage-specific manifest:

- `.manifests/benchmark_plan_review_manifest.json`
- `.manifests/engineering_plan_review_manifest.json`

### Coder roles

Coder roles use the same Codex workspace contract, but they do not submit plans through the planner helper.
The prompt tells them to work in `script.py` and supporting implementation files.

Coder workspaces keep the normal runtime artifact contract:

1. `script.py` is the primary implementation entrypoint.
2. Supporting `*.py` files may be added when needed.
3. `todo.md` and `journal.md` remain writable progress artifacts.
4. Review handoff artifacts are written only when the runtime contract requires them.

### Reviewer roles

Reviewer roles operate on the same workspace but write review artifacts instead of planner output.
The codex prompt must direct reviewers to the stage-specific `reviews/` files and must not ask them to rewrite planner-owned source files.

## Filesystem contract

The Codex backend must follow the same filesystem rules as the rest of the runtime.
Path handling is fail-closed.

The filesystem rules are:

1. Workspace-relative paths are canonical.
2. `/workspace` is only a compatibility alias in runtime plumbing.
3. Prompt text must not expand the alias into the canonical contract.
4. Path traversal outside the workspace root is a deterministic error.
5. The local Codex client and the shared filesystem backend both resolve paths by containment against the resolved workspace root.
6. String-prefix checks are not sufficient and are not accepted as the path-safety rule.

The local containment checks live in [evals/logic/codex_workspace.py](../../../evals/logic/codex_workspace.py).
The shared backend uses the same rule in [shared/workers/filesystem/backend.py](../../../shared/workers/filesystem/backend.py).

## Submission contract

Planner submission is explicit and local in Codex mode.
The agent does not call the controller to submit the handoff.

The submission contract is:

1. The helper validates the required planner files for the active agent role.
2. Benchmark planner submissions canonicalize benchmark constraints before validation.
3. The helper infers the planner variant from the workspace files and does not require `AGENT_NAME`, so the Codex launch environment stays generic.
4. A successful submission writes the stage manifest to `.manifests/`.
5. The helper returns structured `PlannerSubmissionResult` JSON on stdout.
6. Success requires `ok=true` and `status=submitted`.
7. Failure remains local to the workspace and does not masquerade as an accepted handoff.

The helper script is intentionally simple: it is a local shell/Python command, not a controller API.

## Runner behavior

The runner behavior for Codex mode is:

1. Materialize the workspace for the selected eval row.
2. Launch `codex exec` in that workspace with the prompt text.
3. Verify the workspace locally after Codex exits.
4. Persist session metadata including workspace path, launch return code, verification result, and failure reason.
5. Fail closed if the local Codex CLI is missing or the workspace verification fails.

The runner does not require controller/worker orchestration for the agent loop in Codex mode.
The controller-backed preflight and health checks remain part of the paid-provider/controller path only.

The shared runner implementation is in [evals/logic/runner.py](../../../evals/logic/runner.py).

## Observability

Codex mode still emits deterministic session metadata.
That metadata is the debugging record for the run.

The recorded fields include:

01. `agent_name`
02. `task_id`
03. `session_id`
04. `workspace_dir`
05. `prompt_path`
06. `runner_backend`
07. `launch_return_code`
08. `verification_name`
09. `verification_errors`
10. `verification_details`
11. `failure_reason`

The local materializer also prints the workspace and prompt paths for manual inspection.

## Validation contract

The accepted Codex-mode behavior is defined by integration coverage, not by unit-only checks.

The current validation contract is:

1. `run_evals --help` exposes the smoke-test defaults (`benchmark_planner`, `--limit 1`, `--concurrency 1`) and the paid-provider `--call-paid-api` handle.
2. Materialized planner workspaces contain the relative-path prompt contract.
3. Materialized planner workspaces do not contain `/workspace` in the prompt text.
4. Planner submission succeeds from the local workspace helper.
5. Path traversal outside the workspace root is rejected.
6. The Codex launcher uses the native `workspace-write` sandbox with the legacy landlock backend, an isolated `CODEX_HOME` seeded with the active auth bundle and a minimal generated `config.toml`, and a workspace-local `PYTHONPATH` so model-generated shell commands can only mutate the materialized workspace and any explicitly added writable dirs.
7. The controller backend still executes tasks after its preflight step.

The integration test file that exercises this contract is [tests/integration/architecture_p0/test_codex_runner_mode.py](../../../tests/integration/architecture_p0/test_codex_runner_mode.py).

## Non-goals

This mode does not create a new agent graph.
It does not replace the controller-backed production path.
It does not silently support judge/reviewer-chain execution in Codex mode.

<!-- Future: add codex-backed judge/reviewer execution only after the local review-helper contract is defined and tested. -->
