---
title: Agent CLI Provider Contract
status: investigation
agents_affected:
  - benchmark_coder
  - benchmark_reviewer
  - engineer_coder
  - engineer_execution_reviewer
added_at: '2026-04-04T08:10:08Z'
---

# Agent CLI Provider Contract

<!-- Investigation doc. No behavior change yet. -->

## Purpose

This migration introduces a neutral CLI provider contract for agent workspace backends so the runtime can support Codex today and add other CLI backends such as Gemini, Claude, or a `QwenCliProvider` later without hard-coding the binary name or cloning the runner/test seam. The provider contract must cover command-shape differences as well as provider selection: some CLIs launch through `codex exec`, some through `--prompt`, some through a positional prompt, and some may need a file-backed or stdin-backed prompt transport.

The target architecture is described in [devtools.md](../../devtools.md), [agent-harness.md](../../architecture/agents/agent-harness.md), [distributed-execution.md](../../architecture/distributed-execution.md), [evals-architecture.md](../../architecture/evals-architecture.md), and [artifacts-and-filesystem.md](../../architecture/agents/artifacts-and-filesystem.md). The long-term shape keeps Codex as the default CLI provider implementation, but the orchestration code speaks to a provider protocol rather than to `codex` directly and can also route through a `QwenCliProvider` implementation.

## Problem Statement

The current Codex-backed eval path historically treated the native CLI name as the contract:

1. `evals/logic/codex_workspace.py` still owns the subprocess launch path, but the command shape is Codex-shaped today: the batch flow assumes a single argv builder plus a fixed stdin prompt transport, while the interactive flow assumes a prompt-text positional argument.
2. `evals/logic/runner.py` imports provider-aware helpers, but the orchestration still has to carry provider identity, workspace roots, session naming, and trace paths explicitly so the CLI backend does not leak into the runner.
3. The current Codex integration tests validate behavior by spawning fake native CLI binaries or by monkeypatching the provider seam directly.
4. The operational docs still describe the backend as a Codex CLI path instead of a provider-backed CLI family.
5. The repository already contains another prompt-shape example in `scripts/spec_kitty_multi_implement.py`, which launches Gemini through `gemini --prompt`; that is evidence that the command contract must be provider-owned instead of assuming stdin is universal.

That shape works for a single backend, but it makes adding `gemini`, `claude`, or `qwen` style backends expensive because every new CLI would need a copy of the same command-building logic, environment setup, and test seam. It also makes the tests brittle by coupling them to a native CLI binary instead of a provider abstraction. This migration therefore treats `QwenCliProvider` as a first-class follow-on implementation, not just a naming example.

## Current-State Inventory

| Area | Current behavior | Why it must change |
| -- | -- | -- |
| `evals/logic/codex_workspace.py` | Exposes provider-neutral workspace helpers backed by `CliProvider`; Codex remains the default implementation. | The provider seam should own command construction, prompt transport, and provider-specific environment setup so non-Codex CLIs can plug in without duplicating this module. |
| `evals/logic/runner.py` | Routes the local backend through provider-aware helpers and provider selection flags. | Runner orchestration should depend on a provider protocol, not a single CLI name or a hard-coded prompt transport. |
| `tests/integration/architecture_p0/test_codex_runner_mode.py` | Uses fake native CLI scripts, direct subprocess calls, and monkeypatched provider internals. | The tests should mock the provider seam rather than the native CLI binary. |
| `tests/integration/architecture_p0/test_codex_session_trace_capture.py` | Captures Codex session artifacts under Codex-specific filenames and session roots. | Trace capture may remain Codex-shaped for now, but the tests need to stay aligned with the provider seam and avoid assuming the binary is the contract. |
| `specs/devtools.md` | Describes Codex as the default eval-debug backend and documents the Codex CLI path as the operational contract. | The developer docs need provider-neutral wording and an explicit note that Codex is the current default implementation, not the only possible backend. |
| `specs/architecture/agents/agent-harness.md` | Describes the workspace/runtime path as Codex CLI-backed and names `codex exec` directly. | The harness contract should name a CLI-provider-backed path and reserve `codex` for the default implementation. |
| `specs/architecture/distributed-execution.md` | Describes the LLM/tool substrate as API-backed or Codex CLI-backed. | The distributed-execution contract should say API-backed or CLI-provider-backed so the backend split stays generic. |
| `specs/architecture/agents/artifacts-and-filesystem.md` | Says Codex CLI-backed sessions materialize the skill tree into `.agents/skills/`. | That wording should become provider-neutral if the workspace contract remains shared across CLI backends. |
| `specs/architecture/evals-architecture.md` | Refers to the local Codex path and the Codex skill-loop replay flow. | The eval architecture should describe the CLI-provider seam rather than imply Codex is the only replayable CLI backend. |

## Provider Command Surface

The provider abstraction is not just a binary-name shim. The code already uses a small family of backend-specific handles that need to stay behind the provider interface:

| Surface | Current code path | Required contract |
| -- | -- | -- |
| Provider identity and runtime roots | `provider_name`, `binary_name`, `home_dir_name`, `runtime_root_name`, `session_prefix` in `evals/logic/cli_provider.py`, plus session/artifact roots in runner and skill-training paths | Provider-specific names must stay the source of truth for session IDs, home dirs, runtime roots, and artifact roots |
| Home bootstrap | `prepare_home(...)` | Copy auth material, write config, apply reasoning-effort translation, and normalize trust/project settings |
| Environment bootstrap | `build_env(...)` | Set home/cache/config/temp paths, stack profile env, session ID, repo root, PATH/PYTHON_BIN, and any provider-specific env vars |
| Batch launch / resume | `launch_cli_exec(...)`, `resume_cli_exec(...)`, `_run_cli_exec_command(...)` | The provider must be able to express stdin-based prompts, prompt flags, or positional prompts, plus any resume-specific syntax |
| Interactive UI launch | `open_cli_ui(...)` | The provider must be able to express a distinct interactive command shape when the UI mode differs from batch launch |
| Capability probe | `build_help_command()` | Smoke tests and install probes should be provider-owned and should not hard-code `codex exec --help` |
| Reasoning-effort mapping | `translate_reasoning_effort(...)` | Shared config tokens must be mapped to provider-supported values inside the provider implementation |

The target contract should therefore be a command adapter, not just a `list[str]` argv builder. The runner may pass a normalized prompt payload, but the provider decides whether that prompt is delivered via stdin, `--prompt`, a positional argument, or a file-backed handoff. That keeps `QwenCliProvider` compatible with Gemini-style prompt flags and leaves room for future Claude-style direct prompt commands without runner changes.

Observed Qwen surface from `qwen --help`: default one-shot runs use positional prompt text, interactive UI mode uses `-i/--prompt-interactive`, resumable sessions use `-c/--continue` or `-r/--resume`, and `--chat-recording` must be enabled if the runtime expects to resume a prior session.

## Proposed Target State

1. A typed `CliProvider` protocol, or a similarly named neutral abstraction, owns CLI backend behavior such as binary resolution, help/probe invocation, prompt-transport-aware command construction for batch launch, resume command construction, interactive UI launch, and provider-specific environment bootstrapping.
2. The runtime chooses a provider implementation explicitly. Codex remains the default provider implementation, but the orchestration code does not hard-code `codex` outside the Codex provider module.
3. `evals/logic/codex_workspace.py` and `evals/logic/runner.py` depend on the provider protocol instead of on the native CLI binary name, while preserving the existing workspace contract and observable artifacts.
4. `QwenCliProvider` exists as a second concrete CLI-provider implementation with the same contract surface as Codex, even if its backend-specific command details differ.
5. Devtools that can target more than one CLI backend expose the choice explicitly, for example `--provider qwen` or an equivalent selector, rather than inferring the backend from the binary name. This is orthogonal to `--runner-backend`, which remains the controller-vs-local execution-mode selector.
6. Tests exercise the provider seam with provider mocks or fake provider objects instead of creating fake native CLI binaries. A tiny real-CLI smoke check may remain if the repository still wants one, but it should not be the main test seam.
7. The docs describe the CLI-provider contract consistently and state that Codex is the current default implementation rather than the only supported backend family.
8. The abstraction must be expressive enough to model prompt delivery through stdin, prompt flags, or positional prompt text without runner branches.

## Required Work

### 1. Add the provider seam

- Define a neutral `CliProvider` protocol or equivalent in a shared runtime location.
- Add a normalized command-spec or invocation object that carries argv, prompt transport, cwd, env overrides, and any resume metadata so the runner never has to guess whether a backend wants stdin, a flag, or a positional prompt.
- Move Codex-specific command assembly into a `CodexCliProvider` implementation.
- Add a `QwenCliProvider` implementation that follows the same provider contract and keeps Qwen-specific command assembly isolated from the runner. If Qwen uses Gemini-style prompt flags, model that in the provider adapter rather than in the runner.
- Keep provider-specific flags, resume semantics, prompt transport, and environment normalization inside the provider implementation.
- Keep Codex as the default provider implementation for now.

### 2. Route the runtime through the provider

- Refactor `evals/logic/codex_workspace.py` so launch, resume, and UI flows delegate to the provider invocation object instead of building `codex` commands inline or assuming stdin prompt transport.
- Refactor `evals/logic/runner.py` so the local backend is selected through the provider abstraction, not through hard-coded binary-name assumptions or command-shape assumptions.
- Ensure the provider selection path can instantiate `QwenCliProvider` without introducing Qwen-specific branches in the runner.
- Keep the workspace/materialization contract stable while the provider seam is introduced.
- Preserve the current session-trace and workspace-artifact outputs unless a provider-specific difference is unavoidable.

### 3. Update the docs

- Update `specs/devtools.md` to describe the provider seam and the default Codex implementation.
- Update `specs/devtools.md` to describe `QwenCliProvider` as a supported concrete backend alongside Codex.
- Update `specs/devtools.md` to say multi-backend devtools should expose an explicit provider selector such as `--provider qwen`, and that prompt transport is provider-owned.
- Update `specs/architecture/agents/agent-harness.md` to use CLI-provider language instead of a Codex-only backend description.
- Update `specs/architecture/distributed-execution.md` and `specs/architecture/agents/artifacts-and-filesystem.md` so the CLI backend wording is provider-neutral.
- Update `specs/architecture/evals-architecture.md` if it still implies that Codex is the only replayable CLI backend.

### 4. Refactor the Codex tests

- Replace fake native `codex` binaries in `tests/integration/architecture_p0/test_codex_runner_mode.py` with provider mocks or fake provider objects.
- Update the command-assembly assertions so they inspect provider invocations instead of shelling out to a temporary CLI script.
- Add coverage for a `QwenCliProvider` instantiation or selection path if the backend registry exposes one.
- Add coverage for at least one non-stdin prompt transport shape so the provider interface proves it can model `--prompt` or positional-prompt style backends.
- Keep the existing Codex-specific observable contracts, but verify them through the provider seam.
- Update `tests/integration/architecture_p0/test_codex_session_trace_capture.py` only as needed to match any renamed provider-neutral helpers or artifact paths.

## Non-Goals

- Do not change controller/API model-provider selection or conflate it with the CLI provider seam.
- Do not change Codex session trace formats, skill-loop semantics, or workspace artifact naming beyond the new provider abstraction.
- Do not introduce a large plugin framework for arbitrary shells before the provider protocol exists.
- Do not remove Codex as the default CLI provider implementation.
- Do not turn the provider seam migration into a prompt-management or workspace-template refactor.

## Sequencing

The safe order is:

1. Introduce the provider protocol plus the normalized command/invocation spec without changing observable eval behavior.
2. Route the Codex runner and workspace helpers through provider injection so launch, resume, and UI flows consume the invocation spec.
3. Add a Qwen provider implementation and a fake provider test that exercises a non-stdin prompt transport.
4. Update the docs to use provider-neutral CLI wording.
5. Convert the Codex tests from fake native CLI binaries to provider mocks and then widen to any optional real-CLI smoke checks.
6. Add additional CLI providers only after the seam is stable.

## Acceptance Criteria

1. The runtime can construct a Codex-backed eval run through a provider instance without hard-coding `codex` outside the provider module.
2. Adding a future CLI backend does not require copying Codex command-building logic or prompt-transport assumptions into the eval runner.
3. The provider abstraction can represent prompt delivery through stdin, prompt flags, or positional prompt text without changes to runner code.
4. The Codex provider still produces the same observable artifacts and session behavior that the current tests assert.
5. The Codex test suite no longer depends on temporary fake native CLI binaries as its primary seam.
6. The docs describe a CLI-provider contract and clearly state that Codex is the default implementation, not the only backend.

## Migration Checklist

Use this checklist to track the provider seam from the initial abstraction through test refactoring and doc updates. Do not close the migration until every unchecked item is either completed or explicitly waived with a written rationale.

### Provider seam

- [ ] Define `CliProvider` or an equivalent neutral protocol for CLI-backed agent runs.
- [ ] Define a normalized `CliInvocation` or `CliCommandSpec` so prompt transport and resume placement are explicit backend responsibilities.
- [ ] Move Codex command construction, resume handling, UI launch behavior, and help/probe behavior behind `CodexCliProvider`.
- [ ] Add `QwenCliProvider` behind the same provider contract.
- [ ] Route `evals/logic/codex_workspace.py` and `evals/logic/runner.py` through provider injection.
- [ ] Keep Codex as the default provider implementation.
- [ ] Document explicit provider selection for multi-backend devtools, e.g. `--provider qwen`, and call out that prompt transport is provider-owned.

### Docs

- [ ] Update `specs/devtools.md`.
- [ ] Update `specs/devtools.md` to mention `QwenCliProvider`.
- [ ] Update `specs/architecture/agents/agent-harness.md`.
- [ ] Update `specs/architecture/distributed-execution.md`.
- [ ] Update `specs/architecture/agents/artifacts-and-filesystem.md`.
- [ ] Update `specs/architecture/evals-architecture.md` if it still names Codex as the only CLI backend.

### Tests

- [ ] Replace fake native `codex` binaries in `tests/integration/architecture_p0/test_codex_runner_mode.py` with provider mocks.
- [ ] Update the provider-command assertions for `exec`, `resume`, and `help` flows.
- [ ] Add a provider-selection assertion that covers `QwenCliProvider`.
- [ ] Add a fake provider that exercises non-stdin prompt transport.
- [ ] Keep any real-CLI smoke coverage as a tiny provider-availability check if the repository still wants one.
- [ ] Update `tests/integration/architecture_p0/test_codex_session_trace_capture.py` only as needed for provider-neutral helpers or naming.

### Validation

- [ ] Verify the narrow Codex slice first with `./scripts/run_integration_tests.sh tests/integration/architecture_p0/test_codex_runner_mode.py`.
- [ ] Widen to adjacent provider-aware coverage only after the seam is stable.

## File-Level Change Set

The implementation should touch the smallest set of files that actually enforce the new contract:

- `evals/logic/codex_workspace.py`
- `evals/logic/runner.py`
- `evals/logic/cli_provider.py` or an equivalent new provider module/package
- `tests/integration/architecture_p0/test_codex_runner_mode.py`
- `tests/integration/architecture_p0/test_codex_session_trace_capture.py`
- `specs/devtools.md`
- `specs/architecture/agents/agent-harness.md`
- `specs/architecture/distributed-execution.md`
- `specs/architecture/agents/artifacts-and-filesystem.md`
- `specs/architecture/evals-architecture.md`

## Open Questions

- Should the neutral abstraction be named `CliProvider`, `AgentCliProvider`, or `CliBackend` so it does not collide with the existing model-provider terminology in `controller/agent/config.py`?
- Should the initial provider registry live in `evals/logic/` or a shared runtime package so future non-eval callers can reuse it without another refactor?
