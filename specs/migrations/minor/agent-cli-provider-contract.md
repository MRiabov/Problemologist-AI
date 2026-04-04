# Agent CLI Provider Contract

<!-- Investigation doc. No behavior change yet. -->

## Purpose

This migration introduces a neutral CLI provider contract for agent workspace backends so the runtime can support Codex today and add other CLI backends such as Gemini, Claude, or Qwen later without hard-coding the binary name or cloning the runner/test seam.

The target architecture is described in [devtools.md](../../devtools.md), [agent-harness.md](../../architecture/agents/agent-harness.md), [distributed-execution.md](../../architecture/distributed-execution.md), [evals-architecture.md](../../architecture/evals-architecture.md), and [artifacts-and-filesystem.md](../../architecture/agents/artifacts-and-filesystem.md). The long-term shape keeps Codex as the default CLI provider implementation, but the orchestration code speaks to a provider protocol rather than to `codex` directly.

## Problem Statement

The current Codex-backed eval path still treats the native CLI name as the contract:

1. `evals/logic/codex_workspace.py` builds `codex` commands directly for launch, resume, and UI flows.
2. `evals/logic/runner.py` imports Codex-specific helpers and routes the Codex backend through those functions.
3. The current Codex integration tests validate behavior by spawning fake `codex` binaries or calling `subprocess.run(["codex", ...])` directly.
4. The operational docs still describe the backend as a Codex CLI path instead of a provider-backed CLI family.

That shape works for a single backend, but it makes adding `gemini`, `claude`, or `qwen` style backends expensive because every new CLI would need a copy of the same command-building logic, environment setup, and test seam. It also makes the tests brittle by coupling them to a native CLI binary instead of a provider abstraction.

## Current-State Inventory

| Area | Current behavior | Why it must change |
| -- | -- | -- |
| `evals/logic/codex_workspace.py` | Constructs `codex exec`, `codex exec resume`, and `codex` UI commands directly; also owns Codex-specific home/config bootstrapping. | The provider seam needs to own command construction and provider-specific environment setup so non-Codex CLIs can plug in without duplicating this module. |
| `evals/logic/runner.py` | Routes the Codex backend through Codex-specific helpers and Codex-oriented flags. | Runner orchestration should depend on a provider protocol, not a single CLI name. |
| `tests/integration/architecture_p0/test_codex_runner_mode.py` | Uses fake native CLI scripts, direct `subprocess.run(["codex", ...])` calls, and monkeypatched Codex internals. | The tests should mock the provider seam rather than the native CLI binary. |
| `tests/integration/architecture_p0/test_codex_session_trace_capture.py` | Captures Codex session artifacts under Codex-specific filenames and session roots. | Trace capture may remain Codex-shaped for now, but the tests need to stay aligned with the provider seam and avoid assuming the binary is the contract. |
| `specs/devtools.md` | Describes Codex as the default eval-debug backend and documents the Codex CLI path as the operational contract. | The developer docs need provider-neutral wording and an explicit note that Codex is the current default implementation, not the only possible backend. |
| `specs/architecture/agents/agent-harness.md` | Describes the workspace/runtime path as Codex CLI-backed and names `codex exec` directly. | The harness contract should name a CLI-provider-backed path and reserve `codex` for the default implementation. |
| `specs/architecture/distributed-execution.md` | Describes the LLM/tool substrate as API-backed or Codex CLI-backed. | The distributed-execution contract should say API-backed or CLI-provider-backed so the backend split stays generic. |
| `specs/architecture/agents/artifacts-and-filesystem.md` | Says Codex CLI-backed sessions materialize the skill tree into `.agents/skills/`. | That wording should become provider-neutral if the workspace contract remains shared across CLI backends. |
| `specs/architecture/evals-architecture.md` | Refers to the local Codex path and the Codex skill-loop replay flow. | The eval architecture should describe the CLI-provider seam rather than imply Codex is the only replayable CLI backend. |

## Proposed Target State

1. A typed `CliProvider` protocol, or a similarly named neutral abstraction, owns CLI backend behavior such as binary resolution, help/probe invocation, exec command construction, resume command construction, and provider-specific environment bootstrapping.
2. The runtime chooses a provider implementation explicitly. Codex remains the default provider implementation, but the orchestration code does not hard-code `codex` outside the Codex provider module.
3. `evals/logic/codex_workspace.py` and `evals/logic/runner.py` depend on the provider protocol instead of on the native CLI binary name, while preserving the existing workspace contract and observable artifacts.
4. Tests exercise the provider seam with provider mocks or fake provider objects instead of creating fake native CLI binaries. A tiny real-CLI smoke check may remain if the repository still wants one, but it should not be the main test seam.
5. The docs describe the CLI-provider contract consistently and state that Codex is the current default implementation rather than the only supported backend family.

## Required Work

### 1. Add the provider seam

- Define a neutral `CliProvider` protocol or equivalent in a shared runtime location.
- Move Codex-specific command assembly into a `CodexCliProvider` implementation.
- Keep provider-specific flags, resume semantics, and environment normalization inside the provider implementation.
- Keep Codex as the default provider implementation for now.

### 2. Route the runtime through the provider

- Refactor `evals/logic/codex_workspace.py` so launch, resume, and UI flows delegate to the provider instead of building `codex` commands inline.
- Refactor `evals/logic/runner.py` so the Codex backend is selected through the provider abstraction, not through hard-coded binary-name assumptions.
- Keep the workspace/materialization contract stable while the provider seam is introduced.
- Preserve the current session-trace and workspace-artifact outputs unless a provider-specific difference is unavoidable.

### 3. Update the docs

- Update `specs/devtools.md` to describe the provider seam and the default Codex implementation.
- Update `specs/architecture/agents/agent-harness.md` to use CLI-provider language instead of a Codex-only backend description.
- Update `specs/architecture/distributed-execution.md` and `specs/architecture/agents/artifacts-and-filesystem.md` so the CLI backend wording is provider-neutral.
- Update `specs/architecture/evals-architecture.md` if it still implies that Codex is the only replayable CLI backend.

### 4. Refactor the Codex tests

- Replace fake native `codex` binaries in `tests/integration/architecture_p0/test_codex_runner_mode.py` with provider mocks or fake provider objects.
- Update the command-assembly assertions so they inspect provider invocations instead of shelling out to a temporary CLI script.
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

1. Introduce the provider protocol and Codex implementation without changing observable eval behavior.
2. Route the Codex runner and workspace helpers through provider injection.
3. Update the docs to use provider-neutral CLI wording.
4. Convert the Codex tests from fake native CLI binaries to provider mocks.
5. Add additional CLI providers only after the seam is stable.

## Acceptance Criteria

1. The runtime can construct a Codex-backed eval run through a provider instance without hard-coding `codex` outside the provider module.
2. Adding a future CLI backend does not require copying Codex command-building logic into the eval runner.
3. The Codex provider still produces the same observable artifacts and session behavior that the current tests assert.
4. The Codex test suite no longer depends on temporary fake native CLI binaries as its primary seam.
5. The docs describe a CLI-provider contract and clearly state that Codex is the default implementation, not the only backend.

## Migration Checklist

Use this checklist to track the provider seam from the initial abstraction through test refactoring and doc updates. Do not close the migration until every unchecked item is either completed or explicitly waived with a written rationale.

### Provider seam

- [ ] Define `CliProvider` or an equivalent neutral protocol for CLI-backed agent runs.
- [ ] Move Codex command construction, resume handling, and UI launch behavior behind `CodexCliProvider`.
- [ ] Route `evals/logic/codex_workspace.py` and `evals/logic/runner.py` through provider injection.
- [ ] Keep Codex as the default provider implementation.

### Docs

- [ ] Update `specs/devtools.md`.
- [ ] Update `specs/architecture/agents/agent-harness.md`.
- [ ] Update `specs/architecture/distributed-execution.md`.
- [ ] Update `specs/architecture/agents/artifacts-and-filesystem.md`.
- [ ] Update `specs/architecture/evals-architecture.md` if it still names Codex as the only CLI backend.

### Tests

- [ ] Replace fake native `codex` binaries in `tests/integration/architecture_p0/test_codex_runner_mode.py` with provider mocks.
- [ ] Update the provider-command assertions for `exec`, `resume`, and `help` flows.
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
