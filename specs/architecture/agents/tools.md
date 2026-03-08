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

There is a mechanism for privileged filesystem-policy bypass, which is explicitly system-only and requires both:

1. Request payload field `bypass_agent_permissions=true`.
2. HTTP header `X-System-FS-Bypass: 1`.

Header-only or payload-only requests remain policy-enforced (no bypass). 
This is mostly for integration tests where such bypass is convenient (for system access, why not just access directly?).

## Tool definitions

- `execute_command` Execute a command in the sandbox and return command output.
- `list_files` Structured listing with file metadata.
- `read_file` Read file content.
- `write_file` Create or overwrite a file.
- `edit_file` Edit a file by replacing string occurrences.
- `grep` Structured text search.
- `inspect_topology` Inspect assembly/topology metadata.
- `search_cots_catalog` Search COTS parts catalog.
- `submit_plan` Validate and submit planner handoff artifacts.
- `write_review_file` Persist reviewer decision output to the stage-specific file.
- `save_suggested_skill` Persist skill-agent suggested skill output.

Importantly, we have all these methods as async functions, their names with `aread`, `awrite`, `aedit`, etc. This is likely the preferred way to call all these functions.

The rest (submitting the work, testing for design validity, etc) is called via and calling python functions in the code. (as described below)

Note: runtime is DSPy.ReAct-first with LangGraph-managed architecture. We do not use LangChain `create_agent`; orchestration is implemented in our LangGraph workflows and Python runtime wrappers. However, we *do* use native tool calls. We do *not* have tool calls as python objects, they are simply functions with a signature passed into DSPy as in the code snippet above.

ReAct is the first-line correction loop:

1. The model produces output and calls tools.
2. If a tool/validation response is not acceptable, ReAct continues the same node loop and retries.
3. The agent converges on valid handoff output or exits due to hard limits (timeout/turn budget/token budget).

### The "tools" as Python functions - Utils

I propose the following set of tools (their usage is below). Notably, the tools are python imports and functions, called right in one of the edited files!

#### Engineer tools

- `validate_and_price(component: Part|Compound) -> float|dict[str, float]`: validates a part by for manufacturability, then prices it if valid using its workbench's cost calculation interface, or returns an error with a description and a location
  - If validating a compound, it will also check for unusual DOFs, e.g. a part has >=4 DOFs, which is unusual in engineering. It won't raise immediately, but it will throw a "warning". The reviewer will also get notified that DOFs are excessive in this part in particular, and will be more strict in review.
- `simulate(Compound) -> SimulationResult` - Submits a model for a simulation. Should run multiple simulations with slightly perturbed object spawn position; to make sure the engineer agents generate robust solutions.
<!-- dev note: assert against submitting a BuildPart builders, or other types. -->
<!-- should it contain its environment model or only the generated model?  -->
- `submit_for_review(Compound)` - submits the whole assembly for a review to `Reviewer` agent node, which can later approve it and submit return the final design to the user.
<!-- Same: what's in the compound? -->
- `preview_design` - a way to render the CAD files. Used for the engineer to get a visual inspection of its work. Probably doesn't need to render all 24 pictures (maybe, allow a `pitch=180, yaw = 45` parameter to look from a specific side.)
Note - used by default by
- `get_docs_for(type)` - a util invoking a documentation subagent that parses skill and then b123d documentation (local copy, built into container) in search of documentation <!--note: it's probably ideal to have some service which does it fo us-->

#### Benchmark generator (CAD editor) tools

- `validate(Compound) -> bool` the benchmark is validated to not be out of bounds, and not have intersecting:
    - Input object with environment
    - Goal objective with forbid objective
    - Input objective with goal or forbid objectives.
    
    Validated under all environment randomization.

- `simulate(Compound) -> SimulationResult` - a simulation that, unlike the engineering simulation, can not fail, except if not valid as per `validate()`.
- `submit_for_review(Compound)` - submits the whole benchmark compound for a review to `Reviewer` agent node, which can later approve it and thus putting it to the "to solve" pipeline. This call is valid only after current-revision validation/simulation succeed.
- `get_docs_for(type)` - a util invoking a documentation subagent that parses skill and then b123d documentation (local copy, built into container) in search of documentation <!--note: it's probably ideal to have some service like Context7 which does it for us-->
<!-- Note 2: LangGraph subgraphs/subagents composed with DSPy modules are what we'll use here.-->

### Planner tools

`validate_costing_and_price`. Will:

1. Validate pricing YAML file,
2. Output a price in the terminal and warn if the file,
3. Autopopulate some fields to prevent forcing LLMs into calculating or inserting them (e.g., will populate max_unit_cost fields in the engineering planner.)
<!-- Future: will also add some basic planning suggestions. e.g.: i"t appears you are trying to CNC away over 80% of the stock. Consider picking a planning to use a smaller stock if possible."-->

`submit_plan()`. Will:

1. Validate planner-required files for the planner role (engineering planner/electronics planner/benchmark planner).
2. Return structured submission status (`ok`, `status`, `errors`) to the ReAct loop.
3. Be mandatory before planner completion/handoff.
4. Be the only valid planner completion gate: planner transitions are `PLANNED` only when `ok=true`.
5. If planner handoff validation still fails when the planner node exits, orchestration routes back to planner with `REJECTED` state plus validation logs (fail-closed loopback).

### Exact tools logic

I will define exact tool (function) logic to avoid confusion.

#### `validate_and_price(component: Part|Compound)`

Run the workbench interface to validate the part for manufacturability; if passes - also run the pricing, if not, return a validation error with fix suggestions. Detects both assemblies and individual parts.

1. Check cache for if we need to reverify the solution, early exit if not,
2. If there is the environment in the assembly (as required by `simulate` command), assert that it is in the correct position,
3. Validate for the manufacturability as per the Workbench interface,
4. Validate for being in build zone bounds,
5. Determine cost,
6. Validate for cost,
7. Validate for weight.

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

The CAD engineer/coder calls `submit_for_review(compound)` after validation and simulation pass for the latest code revision. This utility persists handover artifacts and marks the submission candidate as ready for review.

Manifest persistence contract:

1. Canonical internal manifest: `.manifests/review_manifest.json`.
2. Reviewer-scoped aliases (mirrors of the canonical manifest):
   - `.manifests/benchmark_review_manifest.json`
   - `.manifests/engineering_plan_review_manifest.json`
   - `.manifests/engineering_execution_review_manifest.json`
   - `.manifests/electronics_review_manifest.json`

Reviewer entry preconditions are explicit and fail-closed:
1. `submit_for_review(compound)` must be called in the latest code revision (latest candidate script state), not in an earlier failed revision.
2. The latest validation artifacts must indicate success (`validate()` in benchmark / `validate_and_price()` in  pass).
3. The latest simulation artifact must indicate success and objective completion (target object reached the green/goal zone).
4. The canonical manifest and the reviewer-stage alias must parse into a typed model (`ReviewManifest`) with `status=ready_for_review` and matching session/revision metadata.
5. Reviewer model access to `.manifests/**` is denied by policy; reviewer eligibility is evaluated by deterministic system checks, not model-side reads of manifest files.

If any precondition is missing/invalid, it is a handoff invariant violation (not a reviewer decision). The system must:
1. Treat the handoff as invalid and route back to the producing agent with explicit validation feedback.
2. Keep this as fail-closed control flow (`REJECTED`/retry path), not as a successful transition into reviewer acceptance semantics.
3. Escalate to terminal `FAILED` when hard limits are reached (timeout/turn/token budget), rather than circling forever.

Reviewer output contract is strict:
1. Reviewer completion must produce a structured `review_decision`.
2. Missing structured reviewer output is invalid reviewer output and stays in fail-closed routing (with explicit logs), never auto-promoted to acceptance.

### Dealing with latency

All Python tools require all files to be uploaded. While this is a very rare edge case that an agent would run code tool before the file was edited, we should ensure that this doesn't happen.
