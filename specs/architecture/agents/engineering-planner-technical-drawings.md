# Engineering Planner Technical Drawings

## Scope summary

- Primary focus: a planner-authored technical-drawing layer for the Engineering Planner.
- The same drafting artifact pattern is mirrored for the Benchmark Planner with benchmark-prefixed filenames.
- Defines when the planner produces drawings, what the drawing package must say, and how the reviewer validates it.
- Treats technical drawings as a derived planning artifact, not as a replacement for `plan.md`, `todo.md`, or `assembly_definition.yaml`.
- Uses build123d-native projection and export capabilities as implementation details, not as architecture terminology.
- Migration mechanics live in [Engineering Planner Technical Drawings Migration](../../migrations/minor/engineering-planner-technical-drawings-migration.md).

## Purpose

The Engineering Planner currently over-relies on prose and coarse structure when the engineer needs geometric specificity.

This spec adds a constrained drafting layer so the planner can communicate:

- critical dimensions,
- datums and reference faces,
- view intent,
- section intent,
- clearance and fit constraints,
- motion envelopes,
- interface callouts.

The goal is not to make the planner do final CAD implementation.
The goal is to make the handoff specific enough that the engineer can implement without guessing the geometry.

## Why this exists

The current handoff shape is strong on budgets and general mechanism intent, but weak on geometric specificity.
That causes three recurring failure modes:

1. The plan is too abstract and the engineer infers the wrong mechanism.
2. The plan is technically plausible but underspecified at the interfaces that matter.
3. The engineer overbuilds because the planner did not mark the few dimensions that are actually binding.

Technical drawings are a better fit than freeform sketches for this repo because the system already uses build123d and already supports native 2D projection and vector export.
The architecture should therefore prefer standardized drawing views over decorative sketch output.

## Non-goals

- This spec does not introduce full GD&T in v1.
- This spec does not require the planner to author every cosmetic dimension.
- This spec does not replace the 3D preview pipeline.
- This spec does not create a separate engineer implementation stage.
- This spec does not expand beyond the planner drafting artifact split described here; the same pattern is mirrored for the benchmark graph, but benchmark-specific planning semantics still live in the benchmark architecture docs.

## Activation policy

The drafting layer is config-gated in `config/agents_config.yaml`.

The mode is a planner-facing prompt policy, not an agent-discoverable feature.
If the prompt does not instruct the planner to use drafting, the planner does not use drafting.
The same config gate applies to the mirrored Benchmark Planner drafting appendix when that planner graph is active.

The mode values are:

- `off`
- `drafting`
- `drawing`

Behavior by mode:

1. `off`
   - PromptManager omits the drafting appendix.
   - The planner is not expected to produce drafting annotations or drawing views.
   - Review does not fail because drafting artifacts are absent.
2. `drafting`
   - PromptManager instructs the planner to add a minimal technical-drawing layer for the critical interfaces only.
   - Review expects those critical interfaces to be covered by structured drawing views or callouts.
3. `drawing`
   - PromptManager instructs the planner to produce a fuller technical drawing package with standardized views and annotations.
   - Review expects stronger coverage of interface geometry, including section or detail views when needed.

The switch is intentionally boring:

- it is not a new tool,
- it is not a new prompt world,
- it is not a hidden fallback,
- it is not an agent-side capability toggle,
- it is only a prompt and validation policy gate.

## Source of truth

The source of truth remains the existing handoff package:

1. `plan.md`
2. `todo.md`
3. `benchmark_definition.yaml`
4. `assembly_definition.yaml`
5. `solution_plan_evidence_script.py`
6. `solution_plan_technical_drawing_script.py`

The technical drawing layer is a structured extension of `assembly_definition.yaml`, not a competing source of truth.

The benchmark mirror uses `benchmark_assembly_definition.yaml` as the analogous structured contract, with `benchmark_plan_evidence_script.py` and `benchmark_plan_technical_drawing_script.py` replacing the `solution_`-prefixed files.

Planner drafting is therefore treated as derived intent:

- `plan.md` explains the mechanism and the rationale.
- `assembly_definition.yaml` carries the machine-readable structure and budgets.
- `assembly_definition.yaml.drafting` carries the technical-drawing intent.
- `preview_drawing()` renders that intent for inspection.

## Planner-owned drafting contract

The drafting layer belongs to the Engineering Planner and the Engineering Plan Reviewer.

The Engineering Coder reads it as read-only context, but does not rewrite it.

The same planner drafting contract applies to the Benchmark Planner and Benchmark Plan Reviewer with benchmark-prefixed script filenames.

The drafting layer must be explicit about the following items:

- which subassembly or interface the view describes,
- which faces, edges, axes, or datums define the view,
- which dimensions are binding,
- which dimensions are approximate,
- which notes are only explanatory,
- which callouts correspond to plan statements,
- which geometry is intended to be revised by the engineer and which geometry is fixed by the planner.

The drafting layer must not invent geometric claims that are absent from `plan.md` or `assembly_definition.yaml`.

If the drawing says a face is at a given angle, that angle must already be supported by the planner handoff.
If the drawing says a hole pattern exists, the hole pattern must already exist in the machine-readable assembly intent.

Planner-authored drafting scripts are read-only evidence, not a second source of truth:

- `solution_plan_evidence_script.py` / `solution_plan_technical_drawing_script.py` for the engineering graph
- `benchmark_plan_evidence_script.py` / `benchmark_plan_technical_drawing_script.py` for the benchmark graph

## Drafting package structure

The drafting content lives in a dedicated section under `assembly_definition.yaml`.
The exact schema should remain strict and typed, with no open-ended freeform fields unless a field is explicitly modeled as a notes map.

Minimal structure:

```yaml
drafting:
  sheet_id: "sheet_1"
  title: "Engineering interface drawing"
  units: "mm"
  projection_standard: "orthographic"
  views:
    - view_id: "front"
      target: "ramp_main"
      projection: "front"
      scale: 1.0
      datums: ["A", "B"]
      dimensions:
        - kind: "angle"
          value: 18.0
          tolerance: "+/- 1.0"
          note: "Slope must redirect the object to the goal zone."
      callouts:
        - id: 1
          label: "critical slope"
          plan_ref: "plan.md#assembly-strategy"
          target: "ramp_main.slope_face"
```

Required semantics:

- `sheet_id` identifies the drawing sheet.
- `views` contains the technical views that matter for implementation.
- `target` identifies the assembly object or interface being documented.
- `projection` identifies the view family, such as front, top, side, section, detail, or isometric.
- `datums` identify the references the engineer must preserve.
- `dimensions` contain only machine-checkable geometry facts.
- `callouts` connect numbered or named drawing markers to the plan text.

## View contract

The drawing package should prefer standardized engineering views over ad hoc perspective sketches.

The common view families are:

- front
- top
- side
- section
- detail
- isometric

Rules:

1. Standard views are preferred for interface-critical geometry.
2. Section views are required when an interface is hidden in the assembled state.
3. Detail views are required when a small region carries the binding geometry.
4. Isometric views are optional and should support comprehension, not carry critical dimensions by themselves.
5. A view without a target or datum reference is not sufficiently reviewable.

## Dimension and annotation contract

The drawing layer is about precision, but not maximal precision.
The spec keeps the first version narrow on purpose.

Allowed annotation classes:

- linear dimensions,
- angular dimensions,
- radius or diameter dimensions,
- fit or clearance notes,
- datum identifiers,
- view labels,
- numbered callouts,
- section markers,
- feature references.

Not required in v1:

- full GD&T symbol coverage,
- standards-complete tolerance stacks,
- advanced profile controls,
- full manufacturing release annotation sets.

Binding rules:

1. If a dimension matters for fit, motion, collision, or attachment, it must appear in the drafting layer.
2. If a dimension is only cosmetic, it should stay out of the drafting layer unless it helps the engineer understand the part.
3. If a callout exists, it must point at a real part of the planner handoff.
4. If a datum is named, it must be stable enough for the engineer to use as a reference.
5. If a note is marked critical, it must be traceable back to the plan or assembly data.

The planner may use the drafting layer to say "this slope must be steep enough" or "this hole center must align with the bracket datum", but the planner must still provide the numerical value or bound.

## Relationship to plan.md

`plan.md` remains the human-readable explanation.

The drafting layer does three things that `plan.md` does not:

1. It binds the geometry to named drawing views.
2. It exposes the few dimensions that are actually decisive.
3. It makes later review easier because the intended geometry is not only described in prose.

The drawing layer should not duplicate every sentence from `plan.md`.
It should extract the geometry-relevant statements and restate them as a structured interface contract.

Recommended cross-reference style:

- `plan.md` references `callout 1`, `callout 2`, or `section A-A`.
- The drafting section references the same labels.
- The reviewer checks that the references match.

## Relationship to assembly_definition.yaml

The drafting section is a child of the assembly contract because it describes geometry that the engineer will actually build.

That means:

- the assembly structure still owns parts, reuse, joints, and motion metadata,
- the drafting layer only owns 2D communication of that assembly,
- the drafting layer must not contradict the motion or budgeting data in the same YAML file,
- the drafting layer must not introduce new parts, joints, or motions.

The drafting layer is therefore a view of the mechanism, not a second mechanism.

## preview_drawing() contract

The public preview contract should gain a companion to `preview()`:

- `preview()` continues to render 3D geometry and assembly context.
- `preview_drawing()` renders the technical drawing package.

The implementation may use build123d technical drawing primitives, `project_to_viewport()`, `TechnicalDrawing`, `ExportSVG`, and `ExportDXF`.
Those names are implementation details, not architecture requirements.

The preview contract may consume either planner graph's drafting scripts, but the generated output remains role-scoped and read-only once materialized.

The tool contract should support:

- raster preview images for inspection,
- vector sidecars for exactness,
- bundle-local render manifests,
- the same render-bucket ownership model used by other preview artifacts.

Rendering rules:

1. A drawing preview is not a substitute for 3D preview.
2. A 3D preview is not a substitute for the technical drawing package when drafting mode is enabled.
3. The vector export is the authoritative drawing artifact.
4. The raster preview is the inspection surface for agents and reviewers.
5. The preview tool should not silently downgrade a failed drawing into an unrelated 3D render.

If the renderer can export both SVG and DXF, the architecture may store either or both as vector sidecars.
The review process still uses image inspection for human or multimodal judgment, because the inspect path already exists in the agent contract.

## Review contract

The Engineering Plan Reviewer validates the drafting layer when the mode is not `off`.

Reviewer responsibilities:

1. Reject drawings that contradict `plan.md`.
2. Reject drawings that introduce unsupported geometry or unstated interfaces.
3. Reject drawings that omit binding dimensions for critical interfaces.
4. Reject drawings that claim datums, views, or sections that do not map to the actual assembly intent.
5. Reject drawings that are technically formatted but not useful for implementation.
6. Reject drawings when the mode requires them and they are absent.

When render images exist for the current revision, the reviewer must inspect them through `inspect_media(...)` before approval, following the existing visual-inspection policy.

The reviewer should answer two questions:

1. Is the drawing specific enough to prevent an implementation guess?
2. Is the drawing still narrow enough that the engineer retains freedom over manufacturable realization?

If the answer to either question is no, the plan is not ready.

## Engineer intake contract

The Engineering Coder reads the drafting layer as read-only context.

The coder may use the drawing to:

- verify interfaces,
- understand intended datums,
- confirm the few dimensions that are binding,
- avoid overbuilding irrelevant geometry,
- align the final CAD implementation with the planner's intent.

The coder may not treat the drawing as a license to change the planner's stated geometry.
If the drawing is ambiguous, the correct response is to raise a plan refusal or request a corrected plan, not to infer a better geometry silently.

## Validation rules

Validation should be strict and fail closed.

Minimum validation checks:

1. The drafting section is schema-valid.
2. Every view targets a real assembly element or interface.
3. Every binding dimension references a valid target.
4. Every callout maps back to the plan or assembly contract.
5. The drafting layer does not add unsupported parts, joints, or motions.
6. The drafting layer is absent when the mode is `off`.
7. The drafting layer is present when the mode requires it.
8. The planner-authored evidence and technical-drawing scripts exist when the drafting mode requires them.

Optional but recommended checks:

- view count sanity,
- dimension completeness for critical interfaces,
- section-view presence for hidden interfaces,
- drawing-to-plan keyword consistency,
- vector-export generation sanity.

## Rollout strategy

This feature should be treated as a controlled planner-output experiment.

Start narrow:

- engineering planner only,
- simple mechanisms first,
- non-electronics tasks first,
- one or two critical interfaces per plan.

Measure:

- plan-review rejection rate,
- coder refusal rate,
- number of turns to first valid implementation,
- implementation deviation rate,
- end-to-end solve rate.

If the drafting layer makes plans more precise but also overconstrains the engineer, tighten the drafting guidance rather than expanding the artifact scope.

If the drafting layer does not improve solve rate or review quality, keep the prompt policy at `off`.

## Future work

Possible follow-ons are intentionally deferred:

- full GD&T,
- deeper cross-checking between callouts and YAML notes,
- richer section/detail automation,
- future benchmark-specific drafting semantics beyond the mirrored artifact split.
- limited GD&T for hole position and fit control,
- automatic dimension extraction from model features,
- detail-view templates for common mechanisms,
- section-view auto-selection,
- vector-only review where it is clearly better than raster inspection,
- benchmark-side adoption if the mirrored artifact split proves useful.

The first version should prove that technical drawings reduce ambiguity.
Only after that should the architecture grow additional drafting semantics.
