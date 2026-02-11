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

**RESOLVED**:

- Added Pydantic validation to `worker/utils/file_validation.py`.
- Added template check (ensuring no `x_min`, etc. remain) to `validate_objectives_yaml`.
- Enforced these checks in `submit_for_review` (handover logic).

### 2. Planner constraint override is not persisted

- Planner is told to set specific `max_unit_cost`/`max_weight` for the engineer, but the engineer reads constraints from `objectives.yaml`.
- There is no requirement/instruction for the planner to update `objectives.yaml` with those values.
- Impact: The plan can disagree with what the engineer is required to obey.

User review: Yes, have to update them. However, the exact mechanism of this is underspecified. I.e. - how would the planner plan for every bit of costs? they would need to probably use a python script and calculate it with a big YAML file with all parts; then it would automatically calculate the "guessed" unit cost.
I'll specify this later, nobody touch it for now.

**Note**: Per user instruction, this is skipped for now.

### 3. Renders path depends on `RENDERS_DIR`

- Planner prompt references `renders/images/`.
- `prerender_24_views()` writes to `RENDERS_DIR` (default `./renders`).
- Impact: If the container does not set `RENDERS_DIR=/renders/images`, the planner will look in a path that isn’t populated.

**Conclusion:** Planning is viable **if** objectives.yaml is populated with real values and `RENDERS_DIR` matches the prompt’s expectation. Otherwise, planning can run but may be low-fidelity or misaligned.

User review: we need to assert (and I tink we do) that the values are populated before they are confirmed.

## CAD Agent Risks (Benchmark + Engineering)

### Engineering CAD Prompt Mismatches

1. **Fastener helper import path resolved**
   - Prompt updated to `from utils import fastener_hole, HoleType`.
   - `worker/utils/__init__.py` now re-exports `cad` utilities.

2. **Fastener helper signature fixed**
   - Prompt updated to match `fastener_hole(part, location=Location((20, 25)), size="M5", length=10.0, hole_id="mount_1", ...)`.

3. **HoleType enum aligned**
   - Prompt now uses `FlatHeadHole`, `CounterBoreHole`, `SimpleHole`.

4. **validate_and_price signature aligned**
   - Prompt updated to `validate_and_price(part, method, config)`.
   - `worker/utils/__init__.py` re-exports `ManufacturingMethod` and `ManufacturingConfig`.

**RESOLVED**: Prompts and re-exports fully aligned with codebase.

### Benchmark Coder Prompt Mismatches

1. **`to_mjcf` function implemented**
   - Implemented `to_mjcf(component, model_name="scene")` in `worker/utils/validation.py`.
   - Re-exported in `worker/utils/__init__.py`.
   - Benchmark prompt updated to include it in utility list.

2. **`validate(compound)` now generates renders**
   - `worker/utils/validation.validate()` now calls `prerender_24_views()`.
   - Logic: Ensuring agent expectation for render availability is met immediately after validation.

**RESOLVED**: Benchmark prompt and API logic aligned.

### Additional Contextual Risks

- `simulate()` in `worker/utils/validation.py` is WRITTEN to incorporate environment constraints:
  - Reads `objectives.yaml` automatically.
  - Checks for contact with `forbid_zones`.
  - Checks for `goal_zone` hit using `moved_object` position.
  - Returns `SimulationResult` with fail reason (e.g., "Forbid zone hit: X").

**RESOLVED**: `simulate()` is now objective-aware and physics-enforced.

## Recommendations (Status)

1. Update engineering prompt: **DONE**
2. Update validate_and_price call: **DONE**
3. Fix benchmark coder prompt / to_mjcf: **DONE**
4. Align render output path: **DONE**
5. require planner to update objectives.yaml: **SKIPPED PER USER**
