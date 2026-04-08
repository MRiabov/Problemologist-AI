# Engineering Planner Technical Drawings

## Scope summary

- Primary focus: a planner-authored technical-drawing layer for the Engineering Planner.
- The same drafting artifact pattern is mirrored for the Benchmark Planner with benchmark-prefixed filenames.
- Defines when the planner produces drawings, what the drawing package must say, and how the reviewer validates it.
- Treats technical drawings as a derived planning artifact, not as a replacement for `engineering_plan.md`, `todo.md`, or `assembly_definition.yaml`.
- Uses build123d-native projection and export capabilities as implementation details, not as architecture terminology.
- Migration mechanics live in [Engineering Planner Technical Drawings Migration](../../migrations/minor/engineering-planner-technical-drawings-migration.md).

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
- `minimal`
- `full`

Behavior by mode:

1. `off`
   - PromptManager omits the drafting appendix.
   - The planner is not expected to produce drafting annotations or drawing views.
   - Review does not fail because drafting artifacts are absent.
2. `minimal`
   - PromptManager instructs the planner to add a minimal technical-drawing layer for the critical interfaces only.
   - Review expects those critical interfaces to be covered by structured drawing views or callouts.
3. `full`
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

1. `engineering_plan.md`
2. `todo.md`
3. `benchmark_definition.yaml`
4. `assembly_definition.yaml`
5. `solution_plan_evidence_script.py`
6. `solution_plan_technical_drawing_script.py`

The technical drawing layer is a structured extension of `assembly_definition.yaml`, not a competing source of truth.

The benchmark mirror uses `benchmark_plan.md` and `benchmark_assembly_definition.yaml` as the analogous structured contract, with `benchmark_plan_evidence_script.py` and `benchmark_plan_technical_drawing_script.py` replacing the `solution_`-prefixed files.

Planner drafting is therefore treated as derived intent:

- `engineering_plan.md` explains the mechanism and the rationale.
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
- which assumptions or calculation IDs the view depends on,
- which dimensions are binding,
- which dimensions are approximate,
- which notes are only explanatory,
- which callouts correspond to plan statements,
- which geometry is intended to be revised by the engineer and which geometry is fixed by the planner.

The drafting layer must not invent geometric claims that are absent from `engineering_plan.md` or `assembly_definition.yaml`.

If the drawing says a face is at a given angle, that angle must already be supported by the planner handoff.
If the drawing says a hole pattern exists, the hole pattern must already exist in the machine-readable assembly intent.

Planner-authored drafting scripts are read-only evidence, not a second source of truth:

- `solution_plan_evidence_script.py` / `solution_plan_technical_drawing_script.py` for the engineering graph
- `benchmark_plan_evidence_script.py` / `benchmark_plan_technical_drawing_script.py` for the benchmark graph

The evidence script stays compact and reviewable; if the planner wants exploded or other layout-oriented presentation, that belongs in the technical-drawing companion instead of the evidence script.

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
          plan_ref: "engineering_plan.md#assembly-strategy"
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

## Relationship to engineering_plan.md

`engineering_plan.md` remains the human-readable explanation.
The planner should expose source-backed assumptions, detailed calculations, and the distilled operating envelope so the drafting layer can reference the exact basis for any binding numeric claim.

The drafting layer does three things that `engineering_plan.md` does not:

1. It binds the geometry to named drawing views.
2. It exposes the few dimensions that are actually decisive.
3. It makes later review easier because the intended geometry is not only described in prose.

The drawing layer should not duplicate every sentence from `engineering_plan.md`.
It should extract the geometry-relevant statements and restate them as a structured interface contract.

Every planner-authored object label and selected COTS `part_id` that appears in the drafting package must also appear in `engineering_plan.md` as an exact identifier mention. Backticks are preferred for the first mention, but the exact string match is the validator input.

Recommended cross-reference style:

- `engineering_plan.md` references `CALC-001`, `ASSUMP-001`, `callout 1`, `callout 2`, or `section A-A`.
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

## Inventory exactness

The drafting layer is not a second BOM.

The planner-authored evidence script and technical-drawing script must preserve the same multiset of labels, repeated quantities, and COTS identities as the assembly contract they accompany. The planner should self-validate this before handoff, and the downstream coder/reviewer must compare against the approved inventory rather than accepting a visually similar but relabeled model.

## Starter template

The starter workspace may ship a default 3-view technical-drawing template for the common case.

The template should present a conventional orthographic trio such as front,
top, and side, leave room for a small number of binding dimensions and
callouts, and be usable unchanged when it already matches the mechanism. It is
a convenience scaffold, not a separate source of truth, and it should stay
lightweight enough to seed workspaces without extra authoring.

## preview_drawing() contract

The public preview contract should gain a companion to `preview()`:

- `preview()` continues to render 3D geometry and assembly context, and it may optionally overlay motion context when `payload_path=True`.
- `preview_drawing()` renders the technical drawing package.

The implementation may use build123d technical drawing primitives, `project_to_viewport()`, `TechnicalDrawing`, `ExportSVG`, and `ExportDXF`.
Those names are implementation details, not architecture requirements.

The preview contract may consume either planner graph's drafting scripts, but the generated output remains role-scoped and read-only once materialized.

Drafting-enabled revisions have a mandatory usage gate:

1. The planner that authors the drafting package must call `preview_drawing()` at least once on the current revision before `submit_plan()`.
2. Any coder or reviewer that is expected to read the drafting package must also call `preview_drawing()` at least once on the current revision before its own completion or approval gate.
3. Missing recorded calls are deterministic validation failures and route the node back through the normal retry loop.
4. The same gate applies to the mirrored Benchmark Planner drafting package and its downstream coder/reviewer stages.

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

1. Reject drawings that contradict `engineering_plan.md` or the assembly inventory, including labels, repeated quantities, and COTS identities.
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

01. The drafting section is schema-valid.
02. Every view targets a real assembly element or interface.
03. Every binding dimension references a valid target.
04. Every callout maps back to the plan or assembly contract.
05. The drafting layer does not add unsupported parts, joints, or motions.
06. The drafting layer is absent when the mode is `off`.
07. The drafting layer is present when the mode requires it.
08. The planner-authored evidence and technical-drawing scripts exist when the drafting mode requires them.
09. Every projected outline or section curve is geometrically valid: no self-intersections, no zero-length segments, and no zero-area closed loops.
10. Every binding dimension is measurable against authored geometry: dimension endpoints, faces, edges, or datums must resolve to real drawing targets, and numeric values must be finite and positive where the dimension type requires it.
11. Every section or detail view must reference a real cut, parent view, or target region, and the referenced view must actually reveal geometry that is hidden or ambiguous in the base view.
12. Every stated clearance, fit, or envelope note must be consistent with the geometry it describes; the drawing may be approximate, but it may not claim a gap or interface that the projected geometry cannot support.
13. Drafting geometry must not contradict the source assembly bounds, part counts, motion limits, or datum references already declared in `engineering_plan.md` and `assembly_definition.yaml`.
14. Units, scale, and handedness are explicit and consistent across all views, dimensions, and drafting metadata.
15. The drawing is generated from a recorded source snapshot or revision hash so the 2D package can be tied back to the exact 3D model it projected.
16. If the handoff declares static randomization, runtime jitter, or motion envelopes, the source geometry is validated at the declared extremes rather than only at nominal pose.
17. If the handoff depends on assembly, insertion, or adjustment access, the drawing validates the necessary access clearance or marks that access as intentionally out of scope.
18. Callout IDs, datum labels, and feature references are unique and stable across the package.
19. The planner-authored evidence script and technical-drawing script preserve the same labels, quantities, and COTS identities as the assembly inventory; the planner must self-validate this before handoff.
20. Drafting-enabled revisions are invalid unless the `preview_drawing()` usage gate above has been satisfied on the current revision.
21. The technical-drawing scripts, `solution_plan_technical_drawing_script.py` and `benchmark_plan_technical_drawing_script.py`, must pass a structural build123d `TechnicalDrawing` import-and-call check. Validation must parse the Python source or resolve its symbols; substring matching is not accepted.
22. The planner-authored evidence script must not use exploded-layout presentation; that presentation belongs in the technical-drawing script instead.

Warning-only checks:

- view count sanity,
- dimension completeness for non-critical interfaces,
- drawing-to-plan keyword consistency,
- datum-reuse consistency so the same datum name does not drift between views,
- part and assembly bounding-box sanity,
- non-zero volume for solid parts that are meant to be solids,
- inter-part clearance sampling where the plan depends on a gap,
- projection consistency across standard views,
- vector-export generation sanity.

Warnings do not block handoff, but the reviewer should see them in the generated validation report.

## Geometric validity scope

The planner drawing gate is intentionally lighter than the benchmark CAD gate.

It should verify that the sketch package is truthful and internally consistent, not that it is simulation-ready or manufacturable end-to-end. In practice, that means:

- no invalid 2D sketch topology,
- no contradictory dimension or datum claims,
- no section/detail callouts to missing geometry,
- no hidden interface being documented only by a perspective view,
- no claimed clearances or fits that are impossible to measure from the authored view set.

## Backing 3D Geometry

Because the drawings are projections of a 3D source model, the validator must check the 3D backing geometry as well as the 2D views.

Minimum 3D checks:

1. Every view target resolves to a real 3D part, subassembly, face, edge, datum, or interface in the authored model.
2. Every solid target is a valid 3D body or compound: finite bounds, non-zero extent, and no obvious self-intersections, non-manifold shells, or open-solid artifacts unless the object is intentionally a reference surface.
3. Unintended inter-part overlap is rejected. Contact is only acceptable when the handoff explicitly describes a mate, joint, fused body, or other intentional interface.
4. Section and detail views must be anchored to real 3D cut planes or feature regions and must produce non-empty geometry in the intended area of the source model.
5. Binding dimensions that reference a feature in the drawing must map back to a measurable 3D relationship in the source model, such as face-to-face distance, hole-center spacing, axis alignment, or feature extent.
6. Claimed clearances, fits, envelopes, and motion envelopes are validated against the 3D source geometry, not only against the projected 2D outline.
7. The drawing must be generated from the same 3D source snapshot that the planner claims in `engineering_plan.md` and `assembly_definition.yaml`; a cleaner surrogate model with different topology is not acceptable.

## Constraint Parity

The drawing gate should mirror the spatial rules that the downstream coder in the same graph will be required to satisfy.

For the benchmark graph, that means validating the same benchmark-owned keep-out contract that `benchmark_coder` later inherits.
For the engineering graph, that means validating the benchmark keep-outs plus any planner-owned build-zone, interface, or motion-envelope constraints that the engineer coder must preserve.

Minimum parity checks:

1. The 3D source geometry must not intersect any declared forbid zone or keep-out volume, except where the plan explicitly says the geometry is an intentional mate or capture surface.
2. The 3D source geometry must remain inside the applicable build zone or workspace envelope for the handoff.
3. Any moving part, swept feature, or motion envelope must clear all forbid zones across its declared operating range.
4. Any interface that is meant to be preserved by the downstream coder must be identifiable in the drawing and must correspond to a stable 3D feature, datum, edge, face, or axis.
5. Goal-zone or target-zone overlap is only acceptable when the plan explicitly states that the geometry is intended to capture, occupy, or reference that zone.
6. Benchmark-owned read-only fixtures and objective markers may be referenced, but the drafting layer may not redefine them or move them into a different spatial contract.

Recommended parity checks:

- minimum clearance to forbid zones,
- swept-envelope sampling for moving geometry,
- proximity sampling for interfaces that sit near keep-outs,
- explicit validation of preserved datums against the 3D source snapshot.

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

- automatic feature-to-view coverage for critical interfaces,
- section-plane hit confirmation for section views,
- projection clipping and annotation crowding checks,
- explicit tagging for reference-only geometry so surfaces, axes, and centerlines are not mistaken for solids,
- tool-access clearance volumes for drawings that imply fasteners or serviceability,
- duplicate-dimension contradiction detection on the same feature,
- full GD&T,
- deeper cross-checking between callouts and YAML notes,
- richer section/detail automation,
- future benchmark-specific drafting semantics beyond the mirrored artifact split,
- limited GD&T for hole position and fit control,
- automatic dimension extraction from model features,
- detail-view templates for common mechanisms,
- section-view auto-selection,
- vector-only review where it is clearly better than raster inspection,
- benchmark-side adoption if the mirrored artifact split proves useful.

The first version should prove that technical drawings reduce ambiguity.
Only after that should the architecture grow additional drafting semantics.
