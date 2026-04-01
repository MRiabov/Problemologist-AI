# Agent harness

## Scope summary

- Primary focus: the runtime harness for agent workflows, including the prompt/workspace contract, submission helpers, debug Codex mode, and skill-loading policy.
- Defines the DSPy/LangGraph/LangFuse runtime shape, backend selection, and local eval-debug behavior.
- Covers agent memory, journal/review artifacts, and the skill lifecycle used by runtime agents.
- Use this file for anything that changes how an agent starts, runs, submits, or learns in the workspace.

## Runtime model

We use DSPy.ReAct as the primary agent runtime and LangGraph to manage agent orchestration. LangFuse is used for observability.

The controller can run either:

1. an API-backed model path, or
2. a Codex CLI-backed local workspace path.

Codex is the default eval-debug backend, and the controller-backed API path remains available when a run must use a paid API provider or needs controller-specific orchestration traces.

## DSPy adapter contract

Adapter choice is an explicit runtime contract:

1. Preferred/default adapter is `dspy.ChatAdapter`.
2. `ChatAdapter` is preferred over `JSONAdapter` for ReAct-style agent nodes because it is the baseline behavior in this dependency/runtime stack and is the most stable path for mixed reasoning + tool-call trajectories.
3. `JSONAdapter` is exception-only and must be justified by a node-specific structured-output requirement that cannot be met by normal typed parsing/validation after model output.
4. Do not silently swap adapters per request. Adapter overrides must be explicit in code/config and visible in observability metadata.
5. Fail closed if adapter selection is invalid or unsupported for a node.

## Debug Codex mode

The debug Codex mode exists to reduce the cost of repeated eval iteration.

It is a local Codex-backed backend for eval debugging and repeated agent runs. It is an application mode, not a new benchmark graph and not a replacement for the controller-backed runtime.

The conceptual split is:

1. `controller` is the orchestration layer and remains the same application responsibility.
2. The LLM/tool substrate can be either API-backed or Codex CLI-backed.
3. Judge and reviewer logic can run against the same deterministic workspace artifacts in Codex mode when the local contracts are satisfied.

## Backend selection

The runner exposes the backend as an explicit mode:

1. `codex` launches a local Codex workspace and runs the task there.
2. `controller` uses the existing HTTP orchestration path and paid model providers.
3. `codex` is the default backend.
4. The backend can be selected through `--call-paid-api`, `--runner-backend`, or `EVAL_RUNNER_BACKEND`.
5. `--call-paid-api` flips the runner to the paid-provider/controller path; `--runner-backend` remains the explicit override.

## Workspace materialization

The Codex backend materializes a run-local workspace before the agent starts.

The materialized workspace is the source of truth for the session, not the repository root.

The workspace contract is:

01. The workspace root is the current directory for the Codex process.
02. The initial prompt is written to `prompt.md` at the workspace root.
03. Shared boilerplate starter files come from `shared/agent_templates/common/`.
04. Role-specific planner starter files come from the role template repositories under `shared/assets/template_repos/`.
05. Planner workspaces include `scripts/submit_plan.sh` from `shared/agent_templates/codex/`.
06. Coder workspaces include `scripts/submit_for_review.sh` from `shared/agent_templates/codex/`.
07. Reviewer workspaces include `scripts/submit_review.sh` from `shared/agent_templates/codex/`.
08. Benchmark planner workspaces do not receive `benchmark_script.py`. Benchmark coder and benchmark reviewer workspaces copy `benchmark_script.py` as read-only geometry context after plan approval, and engineer workspaces copy `solution_script.py` as the authored implementation source. Runtime-owned wrappers remain separate from both.
09. Seed-row artifacts are copied into the workspace before prompt generation.
10. Shared starter templates also include `.admin/clear_env.py`, a local helper that wipes and re-materializes the same seeded row in place so the conversation can continue after a retry.
11. The materialized workspace remains local to the run and is not promoted into a canonical shared root.

The workspace materializer in `dataset/evals/materialize_seed_workspace.py` is the inspection helper for this same workspace contract.

## Prompt contract

The prompt is a contract, not a free-form suggestion.

It must describe the workspace in relative-path terms and must not teach the agent to address the filesystem as `/workspace`.

The canonical prompt rules are:

01. The prompt says `Workspace: current directory`.
02. The prompt tells the agent to use workspace-relative paths only.
03. The prompt does not mention `/workspace` as the workspace root.
04. Planner prompts instruct `bash scripts/submit_plan.sh` as the submission command.
05. Benchmark planner prompts do not include `benchmark_script.py`; that file is introduced only after benchmark plan approval. Coder prompts instruct editing the role-owned authored source file (`solution_script.py` for engineer roles, `benchmark_script.py` for benchmark coder roles) and supporting `*.py` files, then either running `bash scripts/submit_for_review.sh` or using the Python submission utility from `utils.submission` in a supporting script. In that route, `validate` and `simulate` are intermediate checks before `submit_for_review`.
06. Reviewer prompts instruct writing stage-specific review artifacts under `reviews/`, then running `bash scripts/submit_review.sh`.
07. The prompt also advertises `python .admin/clear_env.py` as the in-workspace reset helper for clean retries.
08. The prompt includes the task text, agent name, task ID, and seed dataset name when available.
09. The prompt does not need to describe repository-level import paths or module layout.
10. All starter, non-DSPy-optimized prompt fragments live in `config/prompts.yaml` and `shared/agent_templates/`; see [prompt-management.md](./prompt-management.md) for the unified prompt-source model and backend appendices.

The prompt builder in `evals/logic/codex_workspace.py` is the canonical definition of that prompt text.

## Role behavior

Planner roles are the only roles that use the local submission helper.

Planner behavior is:

1. Edit the planner-owned workspace files for the current stage.
2. Keep the workspace relative-path contract intact.
3. Run `bash scripts/submit_plan.sh` from the materialized workspace.
4. Iterate until the helper reports `ok=true` and `status=submitted`.
5. Treat `.manifests/` as system-owned output, not as editable planner input.

Coder roles use the same Codex workspace contract, but they do not submit plans through the planner helper.
The prompt tells them to work in the role-owned authored source file, supporting implementation files, and the local execution-review helper.

Reviewer roles operate on the same workspace but write review artifacts instead of planner output.
The Codex prompt must direct reviewers to the stage-specific `reviews/` files and must not ask them to rewrite planner-owned source files.

## Filesystem contract

The Codex backend must follow the same filesystem rules as the rest of the runtime.
Path handling is fail closed.

The filesystem rules are:

1. Workspace-relative paths are canonical.
2. `/workspace` is only a compatibility alias in runtime plumbing.
3. Prompt text must not expand the alias into the canonical contract.
4. Path traversal outside the workspace root is a deterministic error.
5. The local Codex client and the shared filesystem backend both resolve paths by containment against the resolved workspace root.
6. String-prefix checks are not sufficient and are not accepted as the path-safety rule.

The local containment checks live in `evals/logic/codex_workspace.py`.
The shared backend uses the same rule in `shared/workers/filesystem/backend.py`.

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

## Skill source contract

Runtime agents read skills from the canonical skill repository mounted into the workspace as `skills/`.

The skill repository boundary is explicit:

1. `skills/` is the canonical runtime skill repository mounted into agent workspaces as `/skills`.
2. `.codex/skills/` is a Codex-only overlay for local debugging/editorial workflows and is not part of the runtime agent skill contract.
3. `suggested_skills/` is a writable sidecar output area for proposed/new skills, not the canonical skill mount that normal agents should read from.
4. Runtime agents should not be taught to treat `.codex/skills/`, `suggested_skills/`, or any other agent-specific skill store as interchangeable with `/skills`.

The skill lifecycle is:

1. The agent can read skills that exist in the canonical mount.
2. The skill creator / learner may update `skills/**` subject to safety limits.
3. Learned skills are versioned and persisted to observability.
4. The skill agent runs asynchronously from the main execution flow.

## Agent memory and review artifacts

Agents keep structured runtime memory in `journal.md`, task progress in `todo.md`, and reviewer outputs in `reviews/**`.

Rules:

1. `journal.md` is the episodic memory for the run.
2. `todo.md` is a writable execution plan/progress artifact.
3. `plan_refusal.md` is only created when a coder refuses a planner handoff.
4. Reviewer outputs are stage-scoped YAML pairs and the decision YAML is the routing source of truth.
5. Token compression is configured by `config/agents_config.yaml` and keeps canonical context telemetry available for compaction.
6. Feedback from simulation, cost checks, and manufacturability checks is recorded in markdown for downstream debugging and skill learning.

## Runner behavior

The runner behavior for Codex mode is:

1. Materialize the workspace for the selected eval row.
2. Launch `codex exec` in that workspace with the prompt text.
3. Verify the workspace locally after Codex exits.
4. Optionally run local judge/reviewer passes against the same workspace artifacts when the run requests judge/reviewer mode and the local stage contracts are satisfied.
5. Persist session metadata including workspace path, launch return code, verification result, judge/reviewer outcomes when run, and failure reason.
6. Fail closed if the local Codex CLI is missing or the workspace verification fails.
7. Controller-backed eval runs apply the `eval` stack profile so the eval bootstrap does not tear down or probe the integration stack, the profile skips the frontend dev server because evals do not need it, and the render path still uses the containerized `worker-renderer` service rather than a host-launched Xvfb fallback.
8. Long-running controller-backed eval runs emit the audible reminder `eval setup running` every five minutes until the run exits.

The runner does not require controller/worker orchestration for the agent loop in Codex mode.
The controller-backed preflight and health checks remain part of the paid-provider/controller path only.

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

## Validation contract

The accepted Codex-mode behavior is defined by integration coverage, not by unit-only checks.

The current validation contract is:

1. `run_evals --help` exposes the smoke-test defaults (`benchmark_planner`, `--limit 1`, `--concurrency 1`) and the paid-provider `--call-paid-api` handle.
2. Materialized planner workspaces contain the relative-path prompt contract.
3. Materialized planner workspaces do not contain `/workspace` in the prompt text.
4. Planner submission succeeds from the local workspace helper.
5. Path traversal outside the workspace root is rejected.
6. The Codex launcher uses the native `workspace-write` sandbox with the legacy landlock backend, an isolated `CODEX_HOME` seeded with the active auth bundle and a minimal generated `config.toml`, and a `PYTHONPATH` that prefers the materialized workspace while still appending the repo root so shared repo modules import correctly during local execution.
7. Codex judge/reviewer mode can run locally from the same workspace artifacts when requested.
8. The controller backend still executes tasks after its preflight step.

The integration test file that exercises this contract is `tests/integration/architecture_p0/test_codex_runner_mode.py`.

## Non-goals

This mode does not create a new agent graph.
It does not replace the controller-backed production path.
It does not change the controller's orchestration responsibility.
