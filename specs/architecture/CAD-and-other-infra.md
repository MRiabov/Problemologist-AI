# CAD and other infrastructure; dependencies.

## Scope summary

- Primary focus: CAD metadata requirements and supporting infrastructure assumptions outside the main agent workflow docs.
- Defines part metadata, rendering direction, schema strictness, and logging/tooling expectations that support CAD execution.
- Use this file when changing CAD model contracts or shared infra assumptions that are not specific to distributed execution.

## CAD

### Part metadata

Parts and Assemblies have metadata, e.g. `cots_id` for COTS parts, `material_id` for material parts, and others; e.g. `fixed: bool` for usage during benchmark generation.

Define a classes `PartMetadata` and `CompoundMetadata` that can store all properties related to them. Without these mandatory fields, validation will fail.

Metadata validation is ownership-sensitive:

1. Engineer-created manufactured parts and planner-declared manufactured parts must carry the manufacturing/workbench metadata required for manufacturability validation and pricing.
2. Benchmark-owned environment geometry, benchmark input objects, and benchmark objective markers are not treated as manufactured outputs.
3. Benchmark-owned COTS fixtures such as motors, bearings, or electronics remain benchmark fixtures even when they carry `cots_id`; they are excluded from engineer manufacturability validation and engineer pricing.
4. Those benchmark-owned read-only fixtures may carry physics/render metadata, but they are excluded from manufacturability validation and pricing.
5. Missing `manufacturing_method` / `material_id` is therefore a hard validation failure only for engineer-owned manufactured parts (and planner-owned manufactured-part definitions), not for benchmark fixtures.

Benchmark definitions also carry a declarative benchmark-side fixture metadata layer in `benchmark_definition.yaml`.

The rule is:

1. `benchmark_definition.yaml` may declare benchmark-owned fixture metadata such as `fixed`, `material_id`, `cots_id`, and attachment policy under `benchmark_parts`.
   - Attachment policy is an allowlist contract for benchmark fixtures, expressed as typed `attachment_methods` (`fastener` or `none`) plus optional reviewer-facing `notes`.
   - When a benchmark fixture may be drilled for fastener mounting, that same policy carries a typed `drill_policy` with numeric limits such as hole count, diameter range, and depth.
   - The Benchmark Planner defines drillable versus non-drillable benchmark parts. This is a permission contract for engineering, not a requirement that engineering must use those attachment paths.
   - The contract is whole-part in MVP. The Benchmark Planner and Benchmark Coder do not define benchmark-side drilling zones or exact allowed coordinates on the part.
   - If a benchmark-owned part is declared in `benchmark_definition.yaml` and its attachment policy allows fastener attachment, engineer-owned parts declared in `assembly_definition.yaml` may attach to it.
   - It is benchmark-owned permission metadata, not engineer implementation metadata.
2. That YAML metadata is the benchmark contract for planning/handover, not the runtime CAD instance metadata used by simulation/export.
3. The actual built geometry still needs runtime `.metadata` on CAD parts/assemblies for exporter, validation, and rendering behavior.
4. Engineer-owned solution metadata remains outside `benchmark_definition.yaml`; it belongs in `assembly_definition.yaml` and the authored CAD result.
   - Planner-declared intended drilling into benchmark fixtures is captured in `assembly_definition.yaml.environment_drill_operations` and must validate against the benchmark-side `drill_policy`.
   - The engineer decides where to place those drilled holes on the allowed benchmark part; benchmark-side metadata constrains whether drilling is allowed and the numeric limits only.

### Assigning a part to workbenches

The agent must assign a part manufacturing method to every part it expects to price or send for manufacturing. If not, the parts can not be priced or sent for manufacturing.

This rule does not apply to the benchmark-owned environment or other benchmark input fixtures handed to the engineer. Those objects are validated as geometry/physics context only, not as manufacturable outputs.

The verification is done by a method.

So suppose the agent's code is as follows:

```py
from build123d import *
from utils.models.schemas import PartMetadata
from utils.enums import ManufacturingMethod
from utils import validate_and_price 
# utils is a __init__.py gathering all utils importable by the agent, ideally. We don't want to force the agent to look through the codebase; in fact it can't because of read permissions. However, it should be doable through utils.

with BuildPart() as part_builder:
    Box(10,10,10)

part_builder.part.label="custom_label"
part_builder.part.metadata = PartMetadata(
    manufacturing_method=ManufacturingMethod.CNC,
    material_id="aluminum-6061"
)

validate_and_price(part_builder.part) # prints ("Part {label} is valid, the unit price at XYZ pcs is ...)
```

### Rendering

The project needs to render the models in images (for preview) and for rendering.

#### Rendering CAD

All CAD preview render jobs are executed by the dedicated headless `worker-renderer` service.

The renderer worker does not inherit a host X server as its normal execution path. The active physics backend and the renderer each select their own OpenGL backend through explicit env vars, with the renderer defaulting to an OSMesa-backed VTK window class for headless reliability.

The rendering backend is not a single global choice. We split rendering by purpose:

1. Explicit preview renders use build123d/VTK by default inside `worker-renderer`.
2. Simulation-video rendering is a switchable contract; the current implementation keeps it on `worker-heavy`/MuJoCo because that was the lowest-overhead route.
3. Genesis-native visual outputs follow the selected simulation backend when the artifact depends on Genesis-only behavior such as FEM, fluids, or backend-native stress/state output.

This split is intentional. Preview evidence does not require Genesis runtime features and therefore stays on the build123d/VTK geometry path, but it is produced only on demand through the dedicated render worker boundary.

Preview renders are context artifacts, not backend-authoritative proof of simulation behavior. Genesis-specific runtime behavior is still established through actual Genesis simulation runs where Genesis behavior is required.

On-demand preview uses the worker-light-facing `preview(...)` helper instead of any validation-time render path. Benchmark callers compose `build()` output with objective overlays reconstructed through the public `utils.objectives_geometry()` helper from the `objectives` section of `benchmark_definition.yaml` before previewing benchmark context, while engineer callers preview their solution geometry directly. The helper is part of the exposed `utils` package, alongside `preview()` and `validate()`, so callers import it instead of defining benchmark-specific geometry logic in agent code. It accepts modality booleans and multi-view camera inputs, normalizes scalar values into view bundles, returns a job ack, and causes the renderer worker to persist workflow-specific preview artifacts under the existing render buckets without implying Genesis parity. The canonical RGB preview filename rule is defined in [Simulation and Rendering](./simulation-and-rendering.md#render-profile-ownership): the part label prefixes the basename as `{part_name}_render_{angle_1}_{angle_2}.png`, so the default 45/45 orbit for `Part(Box(), label="test_part")` is `test_part_render_e45_a45.png`.

#### Rendering views

I presume the model will need to render a view or a set of views to get an understanding of what's happening during the simulation. Allow an extra `view_angles` parameter on `simulate` to trigger simulation from different sides, which would essentially reposition a camera (or a multiple) to render.

Preview evidence is generated explicitly, not as a validation side effect. The default policy is:

1. `/benchmark/validate` performs validation only and does not generate preview artifacts by default.
2. `preview(...)` generates benchmark, engineer, or final preview evidence through the renderer worker when requested.
3. `/benchmark/simulate` may generate backend-native dynamic renders or videos using the selected simulation backend in the renderer worker.

Preview bundles preserve the workflow that produced them instead of flattening every image into a single directory:

1. benchmark preview evidence is persisted under `renders/benchmark_renders/`,
2. single-view engineer inspection previews are persisted under `renders/engineer_renders/`,
3. final engineer preview evidence is persisted under `renders/final_preview_renders/`.

For build123d/VTK-backed preview renders, the requested modality set is persisted under the selected bundle directory and the manifest records the modality-specific artifact path and request-scoped view index for each requested view. RGB previews preserve material colors. Depth and segmentation previews remain PNG-based, and segmentation renders carry a legend mapping colors to object identity.

Those files are context artifacts for downstream agents and reviewers. They follow the same persistence/discovery flow as the existing preview images rather than introducing a second artifact channel, and the render path itself now encodes whether the evidence belongs to benchmark input, engineer inspection, or final preview review.

Dynamic simulation renders and videos are resolved at runtime from the selected simulation backend and are recorded in `simulation_result.json`; they do not take their camera/view contract from `agents_config.yaml` or from benchmark task metadata. The heavy worker executes those jobs and persists the outputs. See [Simulation and Rendering](./simulation-and-rendering.md#render-profile-ownership) for the ownership split.

The simulation backend still exposes a typed render-capability record so the runtime can distinguish supported artifact modes from unsupported ones. Static preview emission is separately governed by the YAML render policy in `config/agents_config.yaml`.

For the RGB preview image, manufactured-part material colors come from the manufacturing material configuration associated with each part's `material_id`. The preview is therefore expected to preserve meaningful color differences between materials, not flatten everything to the same neutral shade.

Render outputs are published as immutable bundle directories under `renders/**`. Each bundle carries a bundle-local render manifest plus any sidecars needed for later lookup. The append-only discovery surface is `renders/render_index.jsonl`, which records `bundle_id`, `created_at`, `revision`, `scene_hash`, bundle path, and primary media paths for each published bundle. `renders/render_manifest.json` may remain as a latest-bundle compatibility alias for current-revision tooling, but the bundle-local manifest and history index are the source of truth for historical lookup.

Bundle-local sidecars are allowed when they help agent tooling resolve the exact render state:

1. `preview_scene.json` stores the exact build123d scene snapshot used for preview rendering.
2. `frames.jsonl` stores sparse frame metadata for video evidence.
3. `objects.parquet` stores dense object pose tables for query helpers. The active `PhysicsBackend` export path produces this file, so both MuJoCo and Genesis can emit it.

The worker-light render-query helper family resolves against these bundle-local artifacts when it needs a point coordinate from a render. It does not infer coordinates from the video bytes alone.

For segmentation renders, the bundle-local manifest must contain a legend mapping rendered colors to object identity. The legend is instance-aware:

1. `semantic_label` is the model-facing semantic name,
2. `instance_id` / `instance_name` distinguishes repeated instances of the same semantic part,
3. repeated parts therefore appear as multiple legend rows that may share `semantic_label` but must not share `instance_id`.

When `inspect_media(...)` reads a render artifact, it resolves metadata from the bundle-local manifest for that bundle. The global `renders/render_manifest.json` path remains compatibility plumbing for current-revision tooling.

Render-modality emission is config-driven through `config/agents_config.yaml`:

```yaml
render:
  split_video_renders_to_images: true
  video_frame_attachment_stride: 6
  video_frame_jpeg_quality_percent: 85
  rgb:
    enabled: true
    axes: true
    edges: true
  depth:
    enabled: true
    axes: true
    edges: true
  segmentation:
    enabled: true
    axes: true
    edges: true
```

If one of the `enabled` flags is set to `false`, the corresponding preview artifact type is not emitted into `renders/**`. This switch controls static preview artifact persistence, not the higher-level worker routing policy.
`split_video_renders_to_images` is separate from the static preview modality flags. When enabled, agent-facing media inspection may decode persisted `.mp4` artifacts into representative image frames for multimodal review. The sampling cadence is controlled by `video_frame_attachment_stride`, which means the tool attaches every Nth frame rather than imposing a fixed cap. `video_frame_jpeg_quality_percent` controls the JPEG encoding quality as a percent value. These settings do not change which artifacts are stored, only how `inspect_media(...)` attaches them to the model.

Each modality can independently enable world-coordinate axes and edge emphasis. Those overlays are controlled by `render.<modality>.axes` and `render.<modality>.edges`.

The RGB static preview can overlay adaptive world-coordinate axes with tick labels and a subtle CAD-style edge emphasis. Depth and segmentation previews can use the same axes overlay, and their edge highlights use a visible non-black accent so they do not disappear into the background or read as void.

### Workbench technical details

Technical details of manufacturability constraints are discussed in spec 004 (not to be discussed here; however manufacturability is determined by deterministic algorithms.)

The workbench validation (as well as other util infrastructure are read-only in the container)

### Supported workbenches

3D printing, CNC and injection molding are supported.

<!-- In the future, it's very interesting to support topology optimization, but that's a separate project. -->

### Off-the-shelf parts (COTS)

COTS search remains the job of the COTS Search subagent. Class resolution and proxy-provider contracts are defined in [COTS geometry import](./cots-geometry-import.md).

The only CAD rule that stays here is metadata ownership: imported COTS geometry must still carry `PartMetadata.cots_id` and must remain benchmark-owned or engineer-owned according to the assembly context, not according to the provider implementation.

<!-- Future work: commented class-instantiation hints may later be generated into starter benchmark and solution scripts, but that is a template concern, not a runtime concern. -->

### Drillability and fasteners

## Other infra

### Benchmark drilling pricing

Drilling into benchmark-owned fixtures has non-zero cost.

For MVP, benchmark drilling cost is static and comes from `manufacturing_config.yaml`.

That cost is part of planner/execution costing whenever `assembly_definition.yaml.environment_drill_operations` is non-empty. It is not optional reviewer commentary.

### Strict schema

We will run `schemathesis` checks against the OpenAPI. Strictly type all schema to avoid ANY issues.

We use Pydantic and Beartype for this.

#### Beartype

To further avoid any issues, we will use Beartype for type checking.

### Schema autogeneration

We autogenerate python schemas, keeping in sync to the workers. We keep schemas defined in the Controller app; worker-light, worker-heavy, and frontend inherit them (for now). We have git hooks that implement the model.
Trace schema contract includes optional reasoning metadata fields (`reasoning_step_index`, `reasoning_source`) and generated clients must preserve those fields.

### Logging

We choose Structlog because it looks better and easier to trace; also theoretically works with OpenTelemetry out of the box.
For utils used internally in an agent, a simple `logging` is acceptable too.

<!--
## CAD and and design validation

As said, "agents will live inside of a filesystem". The agents will generate and execute design validations of files in the filesystem.-->
