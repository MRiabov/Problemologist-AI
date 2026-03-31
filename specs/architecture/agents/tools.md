# Agent tools (ReAct and python importable functions)

## Scope summary

- Primary focus: the tool surfaces available to agent runtimes and Python-side execution helpers.
- Defines DSPy-native tool calling, filesystem permission expectations, and planner/reviewer submission utilities.
- Use this file when changing agent-accessible tools, validation hooks, or handoff gate functions.

## "Agent-native" tools (callable by DSPy.ReAct runtime)

We use standard DSPy/LiteLLM *native* tool calls, as defined in the following snippet:

```py
import dspy

# Define your tools as functions
def get_weather(city: str) -> str:
    """Get the current weather for a city."""
    # In a real implementation, this would call a weather API
    return f"The weather in {city} is sunny and 75°F"

def search_web(query: str) -> str:
    """Search the web for information."""
    # In a real implementation, this would call a search API
    return f"Search results for '{query}': [relevant information...]"

# Create a ReAct agent
react_agent = dspy.ReAct(
    signature="question -> answer",
    tools=[get_weather, search_web],
    max_iters=5
)

# Use the agent
result = react_agent(question="What's the weather like in Tokyo?")
print(result.answer)
print("Tool calls made:", result.trajectory)
```

(from: https://dspy.ai/learn/programming/tools/)

We allow DSPy and LiteLLM do all manipulations over said files.
In the end, the models will write `write_file` and not `next_tool_write_file` or whatever.

### DSPy adapter preference for tool-calling nodes

For DSPy ReAct nodes in this architecture:

1. Use `dspy.ChatAdapter` by default.
2. Prefer `ChatAdapter` over `JSONAdapter` as the standard runtime setting.
3. Use `JSONAdapter` only for explicitly scoped cases where strict whole-response JSON is required and cannot be enforced by downstream validators/parsers.
4. Adapter choice is part of node runtime configuration and should be observable (logged in trace metadata).
5. Do not add permissive fallback logic from `ChatAdapter` failures to `JSONAdapter` (or vice versa) inside a run; treat adapter/runtime mismatch as a deterministic error and route fail-closed.

## Filesystem permissions

Actual read/write/edit access is enforced per-role via `config/agents_config.yaml` path rules. A tool call targeting a forbidden path is rejected by policy (deterministic permission error).

The canonical workspace contract uses workspace-relative paths. `/workspace` aliasing may still exist in runtime plumbing as a compatibility fallback, but it is not the preferred API contract; see `architecture/agents/agent-harness.md` for the rationale and cleanup intent.

There is a mechanism for privileged filesystem-policy bypass, which is explicitly system-only and requires both:

1. Request payload field `bypass_agent_permissions=true`.
2. HTTP header `X-System-FS-Bypass: 1`.

Header-only or payload-only requests remain policy-enforced (no bypass).
This is mostly for integration tests where such bypass is convenient (for system access, why not just access directly?).

## Tool definitions

- `execute_command` Execute a terminal shell command in the session workspace and return stdout/stderr/exit status.
- `list_files` Structured listing with file metadata.
- `read_file` Read UTF-8 text file content.
- `write_file` Create or overwrite a file.
  Reviewer roles use this same tool, but filesystem permissions narrow it to the stage-owned review paths.
- `edit_file` Edit a file by replacing string occurrences.
- `grep` Structured text search.
- `inspect_media` Read visual evidence (`.png`, `.jpg`, `.jpeg`, and supported video frame bundles) and attach it to the model as media input for the current tool step.
- `inspect_topology` Inspect assembly/topology metadata.
- `search_cots_catalog` Search COTS parts catalog.
- `invoke_cots_search_subagent(query: str)` Hand off one request string to the shared COTS Search node.
- `submit_plan` Validate and submit planner handoff artifacts.
- `save_suggested_skill` Persist skill-agent suggested skill output.

`invoke_cots_search_subagent(query: str)` is a prompt-only handoff. It passes exactly one request string to the shared `COTS Search` node and does not inherit planner/coder `task`, `plan`, or `journal` state or use graph-specific signature variants.

Importantly, we have all these methods as async functions, their names with `aread`, `awrite`, `aedit`, etc. This is likely the preferred way to call all these functions.

`execute_command` is a shell-command tool. The agent contract is terminal execution from the session workspace root, not a raw-Python special case.

Rules:

1. The command string is interpreted as a shell command, exactly as an agent would type it in a terminal.
2. Workspace-relative paths such as `solution_script.py`, `benchmark_script.py` once created by `Benchmark Coder`, `plan.md`, and `renders/` remain the canonical path contract inside that shell session.
3. Shell usage such as `python -c ...`, `python solution_script.py`, `python benchmark_script.py` once created by `Benchmark Coder`, `bash -lc ...`, `uv run ...`, `ls`, `cat`, and similar terminal invocations is valid if it stays within filesystem policy.
4. Any implementation that accepts only raw Python source for `execute_command(...)` is a runtime bug or temporary drift, not the intended architecture contract.
5. If we want a raw-Python helper in the future, it should be a separate tool with a separate name and spec, not an overload of `execute_command(...)`.

### Shell-script bridge for missing tools

When the architecture needs a command-like capability but does not expose it as a native tool call, the repo exposes that capability as a checked-in `.sh` script instead. That shell-script bridge is the canonical surface for custom command-like behavior, not ad hoc pseudo-tools in prompts and not newly invented ReAct tool endpoints.

The shell script is the canonical command surface for that capability.
It preserves the same runtime intent as a direct tool call, but it keeps the contract visible in the repository and callable from the workspace shell.
Reviewer submission and planner/coder submission both use this shell-script pattern when a role needs an explicit command-like completion gate.
For reviewer roles, the local bridge is `scripts/submit_review.sh`.
Prompts may also reference the underlying Python utility (`utils.submission.submit_for_review(...)`) as an alternative completion path when a supporting script is clearer for the role; the shell bridge remains the default command-like surface.

Input handling follows a simple split:

1. Short inputs use handles, IDs, or other compact scalar tokens.
2. Larger inputs use file references, typically a YAML or JSON file already materialized in the workspace.
3. YAML submission payloads are passed by path, not inlined as long command arguments.
4. The script validates the referenced file content itself and returns deterministic exit status plus machine-readable stdout/stderr when needed.
5. This bridge is the intended replacement for inventing pseudo-tools in prompt text when the runtime does not have a direct callable tool for the capability.

`read_file` is a text-only contract. It must not be overloaded to return image bytes, base64 blobs, or multimodal payloads for binary assets. Agents must use `inspect_media(...)` for visual evidence. If a binary/media path is passed to `read_file`, runtime should fail closed with a deterministic error telling the agent to use `inspect_media(...)` instead.

The rest (submitting the work, testing for design validity, etc) is called via and calling python functions in the code. (as described below)

Note: runtime is DSPy.ReAct-first with LangGraph-managed architecture. We do not use LangChain `create_agent`; orchestration is implemented in our LangGraph workflows and Python runtime wrappers. However, we *do* use native tool calls. We do *not* have tool calls as python objects, they are simply functions with a signature passed into DSPy as in the code snippet above.

ReAct is the first-line correction loop:

1. The model produces output and calls tools.
2. If a tool/validation response is not acceptable, ReAct continues the same node loop and retries.
3. The agent converges on valid handoff output or exits due to hard limits (timeout/turn budget/token budget).

Native completion-call naming contract:

1. Native tool-loop reviewer nodes terminate by writing the stage-owned review artifacts with `write_file` and then running `bash scripts/submit_review.sh` exactly once.
2. Native tool-loop non-reviewer nodes terminate by calling `finish(...)` exactly once with required signature output fields.
3. Planner nodes still require `submit_plan()` success before their final completion call.

### Config-driven anti-stall reminder policy

For native tool-loop nodes, if the model produces more than 5 consecutive no-tool turns, runtime injects a configured nudge telling it to stop narrating and actually act.

### The "tools" as Python functions - Utils

For submission scripts, the canonical import surface is a top-level `utils` package exposed in the runtime environment. Agent code should import helper functions and metadata types from that package, not from `shared.*` implementation paths.

Canonical submission-script imports:

```py
from utils.submission import validate, simulate, submit_for_review
from utils.metadata import PartMetadata, CompoundMetadata
```

`shared.utils.agent` and `shared.models.schemas` remain implementation modules inside the repo, but they are no longer the agent-facing contract. If runtime internals still route through those paths, that is compatibility plumbing rather than the intended submission script API.

I propose the following set of tools (their usage is below). Notably, the tools are python imports and functions, called right in one of the edited files!

#### Engineer tools

- `validate_and_price(component: Part|Compound) -> float|dict[str, float]`: validates a part by for manufacturability, then prices it if valid using its workbench's cost calculation interface, or returns an error with a description and a location
  - If validating a compound, it will also check for unusual DOFs, e.g. a part has >=4 DOFs, which is unusual in engineering. It won't raise immediately, but it will throw a "warning". The reviewer will also get notified that DOFs are excessive in this part in particular, and will be more strict in review.
- `simulate(Compound) -> SimulationResult` - Submits a model for a simulation. Robustness checking uses runtime randomization by executing one heavy-worker job with one backend scene build/load and `num_scenes` parallel jittered scene instances inside that one backend run. `num_scenes` is batch width, not permission for serialized whole-scene reruns or multiple heavy jobs on one worker; nor multithreaded implementation - only batch width.

<!-- dev note: assert against submitting a BuildPart builders, or other types. -->

<!-- should it contain its environment model or only the generated model?  -->

- `submit_for_review(Compound)` - submits the whole assembly for a review to `Reviewer` agent node, which can later approve it and submit return the final design to the user.

<!-- Same: what's in the compound? -->

- `preview(component: Part|Compound, pitch: float = 45, yaw: float = 45, rendering_type: RenderingType | Literal["rgb", "depth", "segmentation"])` - a way to render the CAD files on demand. Used for the engineer to get a visual inspection of its work. The single-view preview bundle is persisted under `renders/engineer_renders/` rather than being flattened into the root render directory. Benchmark callers compose benchmark geometry first and then preview the composed result.
- `objectives_geometry()` - a zero-argument utility imported from `utils` that materializes the benchmark objective geometry from the canonical benchmark definition path for the current workspace. Preview callers combine its output with benchmark `build()` output before rendering benchmark context.
- `get_docs_for(type)` - a util invoking a documentation subagent that parses skill and then b123d documentation (local copy, built into container) in search of documentation <!--note: it's probably ideal to have some service which does it fo us-->

#### Reviewer / media-inspection tool

- `inspect_media(path: str) -> dict` is the explicit visual-evidence tool.
  - It is the only agent-facing tool allowed to feed images/video-derived frames back into the model.
  - Supported first-line inputs are image files (`.png`, `.jpg`, `.jpeg`).
  - For videos such as `simulation.mp4`, runtime may expose representative extracted frames through the same tool contract when `config/agents_config.yaml render.split_video_renders_to_images=true`. Frame cadence is controlled by `render.video_frame_attachment_stride`, so the tool attaches every Nth frame rather than a fixed cap. That remains a media-inspection action, not a text-file read.
  - The tool returns structured metadata (for example path, media kind, frame/image count, attach success), while the actual image/frame content is attached to the LLM call through the runtime's multimodal message path.
  - When the inspected file is a render artifact under `renders/benchmark_renders/`, `renders/engineer_renders/`, or `renders/final_preview_renders/`, the tool should also return persisted render metadata from `renders/render_manifest.json` when available.
  - For segmentation renders, that metadata must include a color legend mapping rendered colors to object identity.
  - Segmentation legend entries must expose both:
    1. a semantic label the model can reason about (`semantic_label`),
    2. a unique instance identifier (`instance_id` / `instance_name`) so repeated parts in an assembly remain distinguishable.
  - Repeated parts are therefore represented as repeated semantic labels with distinct instance identifiers, not collapsed into one legend row.
  - Merely listing `renders/` or reading `simulation_result.json` does not count as visual inspection.

#### Config-driven visual-inspection policy

- Visual-inspection requirements are configured in `config/agents_config.yaml`, not hardcoded in prompts alone.
- Policy shape:
  - `visual_inspection.required`: whether the role must inspect render images when available.
  - `visual_inspection.min_images`: minimum number of distinct render images the role must inspect.
  - `visual_inspection.reminder_interval`: how often runtime injects reminder messages while the role keeps operating without satisfying the image requirement.
- Current required roles are:
  - `benchmark_plan_reviewer`
  - `benchmark_reviewer`
  - `engineer_planner`
  - `engineer_coder`
  - `engineer_plan_reviewer`
  - `engineer_execution_reviewer`
- `min_images` is policy-driven. Current implementation uses `1` for the roles above.
- The gate is conditional on actual image availability in `renders/**`. If no render images exist for the current node/revision, the media requirement does not trigger.
- Runtime behavior is fail-closed:
  - for required non-reviewer roles in native tool-loop mode, `finish` is blocked until the configured image minimum is satisfied
  - for reviewer roles, approval is invalid unless required media inspection occurred during the current review attempt (enforced before accepting `submit_review.sh`)
- `read_file(...)` never satisfies visual-inspection policy, even if it points at a render path.

#### Benchmark generator (CAD editor) tools

- `validate(Compound) -> bool` the benchmark is validated to not be out of bounds, and not have intersecting:

  - Input object with environment
  - Goal objective with forbid objective
  - Input objective with goal or forbid objectives.
  - Top-level authored part labels must be non-empty, unique, and must not be `environment` or start with `zone_`, because the runtime reserves those names for the scene root and generated objective bodies.
  - `validate()` fails closed on missing or blank authored labels and does not invent fallback names for unlabeled parts.
  - Benchmark-owned moving fixtures must declare their motion contract explicitly; validate rejects missing, contradictory, or unsupported motion metadata, but it does not apply the engineering minimum-DOF rule to benchmark fixtures.

  Validated under all environment randomization.

  - `objectives_geometry()` returns benchmark objective geometry for the current workspace. It takes no arguments because the benchmark objectives are defined in the canonical `benchmark_definition.yaml` path for the current workspace, and preview callers compose the returned geometry with the benchmark assembly output before rendering.

  - The validation tool also generates the standard 24-view static preview package, persisted under `renders/benchmark_renders/` for benchmark-side validation and `renders/final_preview_renders/` for engineer-side validation.

  - That static preview package uses the validation-preview renderer, which is build123d/VTK by default.

  - `validate()` is therefore a fast geometry + preview gate, not a Genesis-runtime parity gate.

  - Current implementation bug to eliminate: `validate()` must fail closed if the compound being validated can only be reproduced from transient shell state and not from the persisted script/workspace snapshot. A minimal fix is to derive a canonical semantic signature from the compound's child/component history, using primitive/component types, authored parameters, label, bounding box, rounded volume, face count, and wire length with tolerance-aware normalization, and compare it against the persisted authored script state; any mismatch is a fail-fast validation error. Raw mesh equality and direct volume equality are too brittle for this gate.

- `simulate(Compound) -> SimulationResult` - a simulation that, unlike the engineering simulation, can not fail, except if not valid as per `validate()`.

- `submit_for_review(Compound)` - submits the whole benchmark compound for a review to `Reviewer` agent node, which can later approve it and thus putting it to the "to solve" pipeline. This call is valid only after current-revision validation/simulation succeed.

- `get_docs_for(type)` - a util invoking a documentation subagent that parses skill and then b123d documentation (local copy, built into container) in search of documentation <!--note: it's probably ideal to have some service like Context7 which does it for us-->

<!-- Note 2: LangGraph subgraphs/subagents composed with DSPy modules are what we'll use here.-->

For benchmark and engineering submission scripts, direct Python calls inside the submission script are the canonical usage. The intended pattern is to run `validate` and `simulate` as intermediate checks before `submit_for_review`:

```py
from utils.submission import validate, simulate, submit_for_review
from utils.metadata import PartMetadata, CompoundMetadata

# construct geometry / assembly

validate(result)
simulate(result)
submit_for_review(result)
```

The preferred execution path for the agent is to run the checked-in shell helper (`bash scripts/submit_for_review.sh`). The underlying Python utility `utils.submission.submit_for_review(...)` is also available for direct invocation in a supporting script when a prompt explicitly chooses that route; `validate` and `simulate` remain intermediate checks.

`build()` may still exist as a compatibility helper, but it is no longer a required submission-script entrypoint. The architecture contract is the submission script itself, not an import-only `build()` API.

### Planner tools

`validate_costing_and_price`. Will:

1. Validate the pricing YAML file.
2. Deterministically compute and normalize cost/weight totals to cent precision from the declared parts and manufacturing inputs.
3. Overwrite the workspace file with the normalized numeric totals so the file on disk is the exit-time source of truth, not the LLM's draft.
4. Reject handoff if a later node-entry equality check would not reproduce the same totals exactly.

<!-- Future: will also add some basic planning suggestions. e.g.: i"t appears you are trying to CNC away over 80% of the stock. Consider picking a planning to use a smaller stock if possible."-->

`submit_plan()`. Will:

01. Validate planner-required files for the planner role (Engineering Planner/Electronics Planner/Benchmark Planner).
02. Return structured submission status (`ok`, `status`, `errors`) to the ReAct loop.
03. Be mandatory before planner completion/handoff.
04. Be the only valid planner completion gate: planner transitions are `PLANNED` only when `ok=true`.
05. Persist the stage-specific plan-review manifest for the next reviewer gate:
    - Benchmark Plan Reviewer: `.manifests/benchmark_plan_review_manifest.json`
    - Engineering Plan Reviewer: `.manifests/engineering_plan_review_manifest.json`
06. For `Benchmark Planner`, canonicalize benchmark constraints before final validation:
    - require planner-authored `constraints.estimated_solution_cost_usd` and `constraints.estimated_solution_weight_g`,
    - derive `constraints.max_unit_cost` and `constraints.max_weight_g` as `1.5x` those estimate fields,
    - reject planner handoff if those estimate fields are missing, non-numeric, or non-positive.
07. For `Benchmark Planner`, also reject handoff when `moved_object.material_id` is missing, empty, or not a known material in `manufacturing_config.yaml`, or when `benchmark_assembly_definition.yaml` is not a schema-valid full `AssemblyDefinition` artifact.
08. For `Benchmark Planner`, reject handoff when moving benchmark fixtures lack explicit motion metadata, when the motion contract is contradictory or unsupported, or when the fixture motion cannot be reconstructed from the benchmark handoff artifacts and evidence.
09. For planner handoffs with machine-readable pricing totals, the persisted file must exactly match the script-normalized values at node entry; any cent-level drift is a hard failure even when the YAML remains schema-valid.
10. If planner handoff validation still fails when the planner node exits, orchestration routes back to planner with `REJECTED` state plus validation logs (fail-closed loopback).

Structured-output validation contract for planner/reviewer YAML artifacts:

1. Schema validation is strict by default: unknown/extra fields are rejected.
2. The strictness applies to nested models as well as top-level fields.
3. Validation failures from unknown/extra fields are hard gate failures (`ok=false` for planner submission, invalid reviewer output for review persistence/routing).
4. Any intentionally open-ended structure must be explicitly typed as such (for example, a specific metadata map). Runtime must not rely on parser permissiveness to accept unknown fields.

### Exact tools logic

I will define exact tool (function) logic to avoid confusion.

#### `validate_and_price(component: Part|Compound)`

Run the workbench interface to validate the part for manufacturability; if passes - also run the pricing, if not, return a validation error with fix suggestions. Detects both assemblies and individual parts.

1. Check cache for if we need to reverify the solution, early exit if not,
2. If there is the environment in the assembly (as required by `simulate` command), assert that it is in the correct position,
3. Split the assembly into benchmark-owned read-only fixtures versus engineer-owned manufactured parts / COTS parts.
4. Validate manufacturability as per the Workbench interface only for engineer-owned manufactured parts. Do not reject because benchmark environment/input-objective fixtures lack manufacturing metadata.
5. Validate full-assembly placement and build-zone bounds, including interactions with the benchmark environment/objectives.
6. Determine cost for engineer-owned manufactured parts and selected COTS parts only. Benchmark-owned COTS fixtures and benchmark-owned electronics never flow into engineer solution pricing,
7. Validate for cost,
8. Validate for weight.

##### `simulate(Compound)`

Simulate the compound. Must have the environment at the exact point where we it was defined in the task. Must have price and manufacturability checked
So:

1. Validate the solution assembly as in `validate_and_price(...)`
   - if valid, pass, if not valid, fail.
2. Git commit all files.
3. Start simulation, locally.
4. If simulation passes, notify the engineer via logs. (don't ask the agent to improve for now, though it could be well cost-efficient and useful). The agent will then run a `submit_for_review(final_compound)`.
   - Don't render the video yet! If the simulation didn't pass, maybe we don't need to render the video. We can instead print positions (probably just final positions) of all parts in the simulation and let the agent introspect them.
     The simulation will produce video. The issue is, it's expensive to render.
5. If doesn't, retry the simulation.

#### submit_for_review(compound: Compound)

The Engineering Coder calls `submit_for_review(compound)` after validation and simulation pass for the latest code revision. This utility persists handover artifacts and marks the submission candidate as ready for review.

Submission-stage contract:

1. The submission call is reviewer-stage explicit; runtime must not guess a default reviewer stage when review submission is requested.
2. Benchmark submission targets `Benchmark Reviewer` and resolves `benchmark_assembly_definition.yaml` as the stage-correct assembly artifact.
3. Engineering and electronics submission targets resolve `assembly_definition.yaml` as the stage-correct assembly artifact.

## Prompt vs skill guidance

Detailed role workflow recipes should live in runtime-loaded skills, not be continually expanded into long monolithic prompts.

Rules:

1. Prompts should carry only the minimum contract, safety, and role-goal information needed for the node.
2. Reusable authoring/debugging workflows belong in skills that the runtime mounts into the agent environment.
3. If a rule needs long examples, troubleshooting branches, or repeated operational reminders, prefer a skill update over appending more prompt text.

Manifest persistence contract:

1. No canonical/central reviewer manifest is persisted.
2. Exactly one reviewer-stage manifest is persisted per submission:
   - Benchmark Reviewer submission: `.manifests/benchmark_review_manifest.json`
   - Engineering Execution Reviewer submission: `.manifests/engineering_execution_review_manifest.json`
   - Electronics Reviewer submission: `.manifests/electronics_review_manifest.json`
3. Planner `submit_plan` persists the plan-review manifest:
   - Benchmark Plan Reviewer: `.manifests/benchmark_plan_review_manifest.json`
   - Engineering Plan Reviewer: `.manifests/engineering_plan_review_manifest.json`

Reviewer entry preconditions are explicit and fail-closed:

1. `submit_for_review(compound)` must be called in the latest code revision (latest candidate script state), not in an earlier failed revision.
2. The latest validation artifacts must indicate success (`validate()` in benchmark / `validate_and_price()` in pass).
3. The latest simulation artifact must indicate success and objective completion (target object reached the green/goal zone).
4. The reviewer-stage manifest must parse into a typed model (`ReviewManifest`) with `status=ready_for_review`, matching session/revision metadata, and correct `reviewer_stage`.
5. Reviewer model access to `.manifests/**` is denied by policy; reviewer eligibility is evaluated by deterministic system checks, not model-side reads of manifest files.

If any precondition is missing/invalid, it is a handoff invariant violation (not a reviewer decision). The system must:

1. Treat the handoff as invalid and route back to the producing agent with explicit validation feedback.
2. Keep this as fail-closed control flow (`REJECTED`/retry path), not as a successful transition into reviewer acceptance semantics.
3. Escalate to terminal `FAILED` when hard limits are reached (timeout/turn/token budget), rather than circling forever.

Reviewer output contract is strict:

1. Reviewer completion must produce a structured `review_decision`.
2. Missing structured reviewer output is invalid reviewer output and stays in fail-closed routing (with explicit logs), never auto-promoted to acceptance.
3. If renders/media are available for the current latest revision, reviewer approval is invalid unless the reviewer has called `inspect_media(...)` on that evidence during the current review attempt.
4. `list_files("/renders")` or equivalent text-only inspection does not satisfy the media-review requirement.
5. The exact minimum number of required inspected render images comes from the reviewer's `visual_inspection.min_images` policy in `config/agents_config.yaml`.

### Dealing with latency

All Python tools require all files to be uploaded. While this is a very rare edge case that an agent would run code tool before the file was edited, we should ensure that this doesn't happen.
