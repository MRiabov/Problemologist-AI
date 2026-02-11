# Feb 10 Round 5 - Prompt/Planning/CAD Validity Review

## Scope

Assess whether the current `config/prompts.yaml` and runtime file layout allow (a) valid planning, and (b) successful CAD execution for benchmark + engineering agents. This review assumes the worker container mounts valid sub-filesystems (e.g., `/skills`, `/config`, and session root) as you described.

## Planning Viability

### 1. objectives.yaml is auto-created, but may be template-only

- `LocalFilesystemBackend.create()` writes `objectives.yaml` from `worker/objectives_template.py` on session init.
- That file contains placeholder values (`x_min`, `x_max`, etc.).
- Impact: If upstream does not overwrite placeholders with real values before planning, the planner can only produce a vague plan or accidentally treat placeholders as real coordinates. This isn’t a path issue, but a data-population dependency.

User review: we have to:

1. validate all yaml with hard types (e.g. beartype/pydantic) and raise if the type mismatches (string instead of a float)
2. Enforce that file must be edited at all, and has a mismatch between the template.

Otherwise the submission must not work.

### 2. Planner constraint override is not persisted

- Planner is told to set specific `max_unit_cost`/`max_weight` for the engineer, but the engineer reads constraints from `objectives.yaml`.
- There is no requirement/instruction for the planner to update `objectives.yaml` with those values.
- Impact: The plan can disagree with what the engineer is required to obey.

User review: Yes, have to update them. However, the exact mechanism of this is underspecified. I.e. - how would the planner plan for every bit of costs? they would need to probably use a python script and calculate it with a big YAML file with all parts; then it would automatically calculate the "guessed" unit cost.
I'll specify this later, nobody touch it for now.

### 3. Renders path depends on `RENDERS_DIR`

- Planner prompt references `renders/images/`.
- `prerender_24_views()` writes to `RENDERS_DIR` (default `./renders`).
- Impact: If the container does not set `RENDERS_DIR=/renders/images`, the planner will look in a path that isn’t populated.

**Conclusion:** Planning is viable **if** objectives.yaml is populated with real values and `RENDERS_DIR` matches the prompt’s expectation. Otherwise, planning can run but may be low-fidelity or misaligned.

User review: we need to assert (and I tink we do) that the values are populated before they are confirmed.

## CAD Agent Risks (Benchmark + Engineering)

### Engineering CAD Prompt Mismatches

1. **Fastener helper import path**
   - Prompt: `from utils.fasteners import fastener_hole, HoleType`
   - Code: `fastener_hole` and `HoleType` are in `worker/utils/cad.py` and **not** re-exported by `worker/utils/__init__.py`.
   - Impact: ImportError if the engineer follows the prompt.

2. **Fastener helper signature mismatch**
   - Prompt uses `pos`, `depth`, `diameter` kwargs.
   - Actual signature: `fastener_hole(part, location, hole_id, size="M3", length=10.0, hole_type=...)`.
   - Impact: TypeError if the engineer follows the prompt example.

3. **HoleType enum mismatch**
   - Prompt allows `CounterSinkHole`, but `HoleType` enum has `FlatHeadHole`, `CounterBoreHole`, `SimpleHole`.
   - Impact: AttributeError if engineer uses `CounterSinkHole`.

4. **validate_and_price signature mismatch**
   - Prompt: `validate_and_price(part)` with no args.
   - Code: `validate_and_price(part, method, config, build_zone=None)` in `worker/utils/dfm.py`.
   - Impact: TypeError on call unless the engineer adds method/config manually.

User review: update it.

### Benchmark Coder Prompt Mismatches

1. **`to_mjcf` function is referenced but not implemented**
   - Prompt requires returning `to_mjcf(...)`.
   - No `to_mjcf` function exists in the codebase.
   - Impact: Benchmark coder cannot complete prompt as written.

2. **`validate(compound)` does not generate renders**
   - Prompt claims `validate(compound)` “also triggers benchmark render capture.”
   - `worker/utils/validation.validate()` performs intersection/bounds checks only; no renders are created.
   - Impact: Agents may think renders exist when they don’t.

User review: already updated.

### Additional Contextual Risks

- `simulate()` in `worker/utils/validation.py` is a generic stability check and does **not** incorporate environment constraints or goal/forbid zones. If the engineer assumes it validates the task objective, it doesn’t.

User review: WHAT? this is critical. It must simulate.

## Recommendations

1. Update engineering prompt to match `fastener_hole` import path and signature, and correct `HoleType` names.
2. Update engineering prompt or `utils.__init__` to provide a correct `validate_and_price` call wrapper.
3. Fix benchmark coder prompt to use the actual API (e.g., `SimulationBuilder`) or implement `to_mjcf`.
4. Align render output path with the prompt, or update the prompt to point at `RENDERS_DIR`.
5. If planner must set max_unit_cost/max_weight, require it to update `objectives.yaml` explicitly.
