---
name: runtime-script-contract
description: Canonical import and execution contract for authored benchmark and engineer scripts. Use when writing `script.py` or solution scripts that call validation, simulation, or submission helpers.
---

# Runtime Script Contract

Use this skill when authoring build123d scripts that the runtime will execute or inspect.

## Canonical imports

Use top-level `utils` imports in authored scripts:

```python
from utils.submission import validate, simulate, submit_for_review
from utils.metadata import PartMetadata, CompoundMetadata
```

`shared.*` paths are implementation internals. Do not use them in authored agent scripts.

For ordinary mechanical engineer scripts, keep imports minimal:

- default to `build123d` plus `utils.metadata`
- add other `utils.*` imports only when the final geometry or task contract actually uses them
- do not leave unused helper imports in the final file

## Canonical file contract

`script.py` is the primary authored deliverable.

- Prefer exposing the final object at module scope as `result = ...`.
- `build()` remains an optional compatibility helper.
- Keep the primary geometry and assembly definition in `script.py`; do not split core authored state into scratch verifier files.
- For planner/coder handoffs, `script.py` should implement the full required assembly for that turn, not just an initial subset of parts.
- For routine passive/freestanding mechanical handoffs, draft `script.py` from the planner files first; do not pause to reread general CAD references unless you hit a concrete modeling blocker.
- In benchmark and engineer assemblies, every top-level child label must be unique and must not be `environment` or start with `zone_`. Those names are reserved for the scene root and simulator-generated objective bodies, and duplicate labels collide with MJCF mesh/body names.

## Final assembly exposure

The runtime accepts either of these patterns:

1. Bind the final build123d object at module scope, for example `result = benchmark`.
2. Define `build()` and return the final build123d object.

`build()` is optional compatibility, not the preferred contract.

## Evaluation Playbook

Use this sequence for engineer-coder evals:

01. Read the seeded handoff files first: `plan.md`, `todo.md`, `assembly_definition.yaml`, and `benchmark_definition.yaml`.
02. Read `skills/build123d_cad_drafting_skill/SKILL.md` before the first geometry draft.
03. Only read `skills/electronics-engineering/SKILL.md` if the approved handoff explicitly contains an `electronics` section or the benchmark declares `electronics_requirements`. Motors alone do not imply an electronics task, and mechanical wire-routing placeholders do not qualify.
04. Keep the first `script.py` draft compact and complete for the handoff. Prefer a direct `result = ...` binding and avoid extra helper files.
05. Run one cheap syntax check first, then one real probe against the authored `script.py`:
    - `py_compile` or equivalent
    - `from script import result`
    - `from utils.submission import validate, simulate`
    - `validate(result)`
    - `simulate(result)` if validation passes
06. If validation fails, fix the geometry or placement in `script.py`, not the execution contract.
07. If the same issue persists after one targeted fix, record the blocker in `journal.md` and stop diagnostics instead of widening into repo spelunking.
08. For engineer-owned scripts, do not copy benchmark-only `fixed=True` examples into the implementation unless the current task explicitly says you are authoring benchmark fixtures.
09. If the workspace includes a reviewer-approved seed artifact for the same mechanism family, mirror its placement pattern and datums before inventing a new layout. Adapt imports and metadata to the current runtime contract, but keep the proven geometry conventions. For the sideways-transfer family, ensure the roller bed and goal tray do not overlap in X; if the tray width is 150 mm, a center near `x=505` mm is safer than the approximate seed center `x=485` mm.
10. If the task includes explicit electronics requirements, keep the electrical design in the approved handoff artifacts and direct geometry implementation. Do not import `utils.electronics`, `shared.*`, or `worker_heavy.*` from `script.py`; those are runtime internals, not the authored submission-script API.

## Placement contract

Place top-level authored parts with `Location(...)` through one of these patterns:

- `part.move(Location(...))`
- `part.moved(Location(...))`
- `part.location = Location(...)`

For parts that must sit on a datum surface or on top of another part, make the vertical alignment explicit:

- Prefer `align=(Align.CENTER, Align.CENTER, Align.MIN)` for box-like solids that rest on a floor or base plate.
- Then place the part with the bottom face at the intended datum using `Location((x, y, z))`.
- Do not rely on the default `Box(...)` alignment if the part must avoid interpenetrating a support part.
- If `validate(result)` reports a geometric intersection, inspect the Z datum alignment between the base plate and every child part before redesigning the mechanism.

Do not rely on `part.position = (...)` as the runtime placement contract for authored assemblies.

## Execution modes

Choose the narrowest mode that matches the task contract.

### Import-safe mode

Use when the workspace, validator, or seeded handoff expects the module to be imported safely.

- expose `result` or `build()`
- avoid validation/simulation/submission side effects on import
- keep the file reusable by external validators

### Script-execution mode

Use only when the task explicitly allows direct execution side effects from the authored script.

- helper calls such as `validate(...)`, `simulate(...)`, and `submit_for_review(...)` may live in the module body only when that execution model is explicitly part of the task
- do not add wrapper files just to trigger those helpers

## Contract boundary

- This skill defines the authored-script contract, not a preferred shell tactic.
- Prompts should not hardcode `python -c`-style workflows here.
- Validation should operate on the real `script.py` artifact rather than alternate reconstructed geometry.
- For engineer or benchmark self-checks, prefer dedicated runtime helper tools that operate on `script.py` when they are exposed. If they are not exposed in the current tool surface, use one short workspace-root `execute_command(...)` probe that imports `result` from `script.py` and calls `utils.submission.validate(...)` / `simulate(...)` instead of a fragile `python -c "from script import result ..."` one-liner.
- `execute_command(...)` already runs from the seeded workspace root. Do not prepend `cd /workspace` or other host-specific workspace paths before those checks.

## Safety rules

01. Do not use `if __name__ == "__main__":` in authored scripts.
02. Use workspace-relative paths such as `script.py`, `todo.md`, and `renders/preview.png`.
03. Do not add a leading slash to workspace artifacts. Use `script.py`, `journal.md`, `todo.md`, `renders/...`, and `simulation_result.json`, not `/script.py`, `/journal.md`, `/todo.md`, `/renders/...`, or `/simulation_result.json`.
04. Keep long operational guidance in skills like this one rather than copying it into planner/coder prompts.
05. Keep authored scripts terse. Avoid long module docstrings, banner comment blocks, and other decorative prose in files that will be emitted through `write_file(...)`.
06. If a tool call would require a very large or quote-heavy `content` string, simplify the script content first so the `write_file(...)` arguments remain valid JSON.
07. If `write_file("script.py", ...)` is rejected for invalid JSON arguments, do not retry the same large payload. Write a compact import-safe stub first, then expand the file with smaller `edit_file(...)` replacements.
08. Do not import visualization-only helpers such as `ocp_vscode`, and do not call `show(...)` from authored workspace scripts.
09. Updating `todo.md` is not a reason to stop early. Finish the required assembly unless a concrete blocker forces a partial handoff or refusal.
10. For simple freestanding/passive mechanical seeds, start with the fewest primitives and booleans that satisfy the planner handoff. Avoid decorative fillets, chamfers, and optional detailing unless functionally required.
11. For simple passive seeds, the normal read set is the planner handoff plus `objectives.yaml`. Do not treat general CAD skills or large helper references as mandatory pre-reads before the first draft.
12. If the task or seeded `todo.md` explicitly requires validation or simulation before handoff, a syntax-only `py_compile` check is not enough. Keep `script.py` import-safe, then run one real probe against `result` and fix any runtime failure before stopping.
13. If the task expects review handoff, the final workflow step is a Python helper call to `submit_for_review(result)` after validation and simulation succeed. This is not a shell helper and it is not a replacement for the validation/simulation checks.
14. Any extra command-like workflow that is not already exposed as a native helper belongs in a checked-in shell script, not a prompt-only pseudo-tool or ad hoc ReAct trick.
