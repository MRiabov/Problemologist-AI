# Unexpected Functionality List

## Scope summary

- This file is an append-only running list of functionality that exists in the
  codebase but is easy to miss during prompt, runtime, or architecture review.
- Use it to record helper surfaces, runtime hooks, and other repo-level
  capabilities that should be remembered explicitly.
- Keep entries short and factual.

## Current list

### 2026-04-01

- `worker_heavy.utils.validation.define_fluid(...)` exists and is re-exported
  through `worker_heavy.utils`.
- `worker_heavy.utils.validation.preview_stress(...)` exists and is re-exported
  through `worker_heavy.utils`.

## Deferred items

- `shared.utils.agent.refuse_plan(...)` exists, but it is intentionally kept out
  of scope for this list for now.
