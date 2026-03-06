# Desired architecture (human written)

## Table of Contents

- Objective of the system (228)
  - Outputs and end project goals (232)
- Agents (245)
  - Benchmark generator agent (or graph) (249)
    - Agent purpose (251)
    - Agent subagents (255)
    - Output requirements (265)
    - Benchmarks (280)
    - Subagents output requirements (290)
    - Sample output (292)
  - Engineer (problem solver) (371)
    - Purpose (373)
    - Description (377)
    - Engineer agent details (391)
      - Planner workflow (401)
    - Verification (490)
  - COTS search subagent (502)
    - Model and runtime (508)
    - Inputs (from planner/engineer/benchmark planner/benchmark CAD engineer) (514)
    - Tools (519)
    - Invocation (529)
      - Reasons for invocation (536)
    - COTS catalog database (spec 006) (542)
  - Agentic framework (554)
    - Implementation (558)
  - Filesystem (583)
    - Templates (587)
      - Initial files for each agent (591)
      - Template auto-validation (610)
    - Immutability validation (614)
    - File updates (618)
    - Utils files (626)
    - Skills files (630)
      - Managing skills in git repo (642)
    - Automatic validation of files (650)
      - Validating markdown files (654)
      - Validating python files (660)
    - Starting folder structure for various agents (664)
      - Benchmark generator (685)
        - Benchmark Generator - planner (687)
        - Benchmark Generator - CAD agent (693)
      - Engineer (706)
      - Planner - benchmark generator (719)
    - Paralel environments for agents (721)
  - Agent artifacts (727)
    - Prompts and environment (731)
    - Journal (738)
      - Journal lookup for learner agents (756)
    - TODOs (762)
    - Reviews by reviewers (766)
  - Token compression (773)
  - Skills (777)
    - `build123d` skill and access to documentation (787)
    - Different agents have different skills (791)
    - Explicit skill agent (795)
      - Skill agent is run async (803)
      - Skill agent has a journal too (808)
        - Structure of the journal (817)
        - Using a sidecar agent for Journalling (833)
        - When to write in a journal? (839)
        - Example (848)
    - Worker skills are persisted and are separate from the repository-level skills (855)
    - Skill safety toggle (859)
    - Skill versioning (863)
    - Linting (870)
  - Execution process (876)
  - Feedback to the agent (880)
    - Feedback from the simulation (884)
      - Compressing the video (890)
    - Feedback from cost and manufacturability constraints (894)
  - Agent handovers (898)
    - All handovers that happen (900)
    - Benchmark generator Planner and Benchmark CAD designer (916)
    - Benchmark Generator with Engineer handover (945)
      - A benchmark is reused multiple times (960)
      - What if the benchmark agent set the price too low? (964)
    - Engineer Planner and "Coder" interaction (968)
      - `plan.md` structure for the engineering plan (977)
      - `objectives.yaml` (1012)
      - `assembly_definition.yaml` (1088)
    - Coder and Reviewer interaction (1203)
  - Clarification - definition of constraints in planning (1231)
  - Agents cap on execution (1237)
  - Steerability (1241)
    - Steerability with exact pointing (1250)
    - Steering code directly (1289)
    - Pointing in the chat (1306)
    - Steerability - Other (1310)
    - Hard constraints on Agent output (1317)
- Agent Evaluations (1324)
  - Evaluations architecture (1333)
  - Multi-level evaluations architecture (1340)
    - Fast (1344)
    - Medium evals (1355)
      - Medium evals - Engineer evaluations (1357)
        - Medium evals - Engineer Planner evaluations (1359)
        - Medium evals - CAD Engineer (1371)
        - Medium evals - Engineer Reviewer (1379)
      - Medium evals -  Benchmark Generator (1403)
        - Medium evals - Benchmark Generator Planner (1405)
        - Medium evals - Benchmark Generator CAD engineer (1418)
      - Medium evals - Skill Learning Agent (Async) (1426)
      - Journaling (1434)
    - Slow (essentially, production tasks) (1439)
      - Slow evals - Engineer (1441)
        - End-to-end eval (1443)
        - Engineering Planner (1451)
        - CAD Engineer (1453)
        - Reviewer (1465)
      - Slow evals - Benchmark generator (1469)
        - Slow evals - CAD Engineer (1471)
    - Integration/post-processing evals (more than one episode) (1476)
  - (1485)
- Distributed execution (1487)
  - Worker API (1498)
    - Asset Serving (1502)
    - Concurrency and Parallelism (1515)
  - Persistent state and durable execution (1525)
  - Hyperscaler of choice (1529)
    - Deployment specifics (1533)
    - Podman containers (1537)
      - `uv` base containers (1541)
  - Persisting files (1545)
    - Workers' filesystem (1553)
    - Videos (1562)
  - Database (1574)
  - Agent and Worker boundary (1583)
    - Separation (1585)
    - Temporal (1589)
- Simulation and "Definitions of Done" (1597)
  - Simulation constants and assumptions (1606)
    - Physically-realistic constraints (1614)
      - Creating realistic constraints (1625)
        - Fixed parts for the simulation definition (1629)
        - Fasteners (1633)
        - Agent workflow for fasteners (1646)
        - Edge case: multiple holes (1699)
        - Mechanisms and Moving Parts (1711)
        - Implementation Logic for constraints (1741)
      - Constraining to the environment (1749)
        - Specifics (1758)
      - Allowed components in simulation (1762)
    - Constants (1784)
  - Convex decomposition (1797)
  - Motors (1805)
    - Controller functions (1809)
      - Time-based functions (take in `t` as time) (1813)
      - Implementation for time-based controller functions (1824)
      - Position-based functions (1828)
        - Position-based controllers implementation (1840)
    - Actuator force limits (forcerange) (1877)
    - Motor overload failure (1894)
  - Definition of "success" and failure in the simulation (1906)
    - Moving an object from one screen to another (1912)
      - Checking for objective interaction (1925)
    - Randomization (1929)
      - Static randomization (1936)
        - Material static randomization (1943)
        - Visual static randomization (1949)
      - Runtime randomization (1953)
        - Runtime randomization verification (1959)
    - Failure (1963)
  - Conversion of CAD to mesh and to MuJoCo/Genesis (1977)
  - Preview/visualization (1987)
    - Mesh limits and Simplification (1991)
  - Materials (2019)
- Observability (2049)
  - LangFuse for observability, DB for deeper logs (2063)
    - All collected events (2071)
      - Not just events, but numeric events (2118)
      - Tracking seeds (2122)
  - Metrics (2126)
    - Bulk uploading events (2155)
  - Best practice: Give LLMs a way to complain (2160)
  - Backups (2171)
  - User Reviews (2178)
    - Training on user reviews (2189)
- Strict schema (2197)
  - Beartype (2203)
  - Schema autogeneration (2207)
- "Workbenches" - manufacturability verification (2211)
- CAD and and design validation (2218)
  - Tools (2222)
    - "Agent-native" tools (callable by DSPy.ReAct runtime) (2224)
    - The "tools" as Python functions - Utils (2261)
      - Engineer tools (2265)
      - Benchmark generator (CAD editor) tools (2278)
    - Planner tools (2292)
    - Exact tools logic (2301)
      - `validate_and_price(component: Part|Compound)` (2305)
      - `simulate(Compound)` (2317)
      - submit_for_review(compound: Compound) (2331)
    - Dealing with latency (2335)
  - Part metadata (2339)
  - Assigning a part to workbenches (2345)
  - Rendering (2371)
    - Rendering CAD (2375)
    - Rendering views (2380)
  - Workbench technical details (2384)
  - Supported workbenches (2390)
  - Off-the-shelf parts (COTS) (2396)
- Code and responsibility separation (2404)
  - Database(s) (2415)
    - Assets (2429)
  - Repo code structure (2435)
  - Logging (2452)
  - Testing (2458)
  - Networking (2467)
    - Communication between containers (2471)
    - Multiple agents per machine CPU (2475)
  - On container termination (2482)
  - Strict API requirement (2494)
  - Batch support as a first-class citizen (2500)
    - Async execution (2505)
  - Frontend (2509)
- Inputs to the system (2513)
  - Inputs to the Benchmark generator (2515)
  - Engineering agent(s) (2529)
- Security (2533)
- Outputs of the system and postprocessing (2537)
- Other notes (2543)
- Complexity tracking worksheet (what is more complex but necessary) (2555)

## Objective of the system

We are to create a benchmark and a training dataset for evaluating LLM models on creating and solving dynamics problems.

### Outputs and end project goals

1. Benchmarks and problems to solve
2. Reasoning traces trying to create benchmarks,
3. Reasoning traces trying to solve the problems with manufacturable and verified solutions
4. Solutions to the problems, with end goals of CAD models and manufacturable, verified solutions,
5. Notable optimization to the problems (found but not applied, by mistake or to save compute),
6. Skills acquired during execution of the model (SKILL.md files and their assets - references and scripts)
7. Journals and "scannable" summaries of the execution.
8. An open-source framework for benchmarking, optimization of visual-language models to solve dynamic mechanical engineering problems
9. **An open-source framework to solve mechanical engineering problems.**
10. (additionally) A large library of build123d code.

## Strict schema

We will run `schemathesis` checks against the OpenAPI. Strictly type all schema to avoid ANY issues.

We use Pydantic and Beartype for this.

### Beartype

To further avoid any issues, we will use Beartype for type checking.

### Schema autogeneration

We autogenerate python schemas, keeping in sync to the workers. We keep schemas defined in the Controller app; worker-light, worker-heavy, and frontend inherit them (for now). We have git hooks that implement the model.
Trace schema contract includes optional reasoning metadata fields (`reasoning_step_index`, `reasoning_source`) and generated clients must preserve those fields.

## "Workbenches" - manufacturability verification

The agents will have *Workbenches* - a set of tools they can use to:

1. Verify their manufacturability.
2. Calculate costs of their parts (and verify against the user-inputted goals)

## CAD and and design validation

As said, "agents will live inside of a filesystem". The agents will generate and execute design validations of files in the filesystem.

### Agent tools (ReAct and python importable functions)

#### "Agent-native" tools (callable by DSPy.ReAct runtime)

The following agents can expose the same tool API surface: `execute`, `ls_info`, `read`, `write`, `edit`, `grep_raw`, `glob_info`, `upload_files`, `download_files`.

Actual read/write/edit access is enforced per-role via `config/agents_config.yaml` path rules. A tool call targeting a forbidden path is rejected by policy (deterministic permission error).

Submission/review handoff contract:

- `submit_plan()` is ReAct-callable for planner roles.
- `submit_for_review(compound: Compound)` is a Python util called from CAD scripts (not a ReAct-native tool call).
- Reviewer entry is gated by persisted handover artifacts and typed payload validation, not by a ReAct tool trace.

1. Benchmark Planner: `execute`, `ls_info`, `read`, `write`, `edit`, `grep_raw`, `glob_info`, `upload_files`, `download_files`
2. Benchmark Generator: `execute`, `ls_info`, `read`, `write`, `edit`, `grep_raw`, `glob_info`, `upload_files`, `download_files`
3. Benchmark Reviewer: `execute`, `ls_info`, `read`, `write`, `edit`, `grep_raw`, `glob_info`, `upload_files`, `download_files`
4. Engineering Planner: `execute`, `ls_info`, `read`, `write`, `edit`, `grep_raw`, `glob_info`, `upload_files`, `download_files`
5. Mechanical Engineer: `execute`, `ls_info`, `read`, `write`, `edit`, `grep_raw`, `glob_info`, `upload_files`, `download_files`
6. Electrical Engineer: `execute`, `ls_info`, `read`, `write`, `edit`, `grep_raw`, `glob_info`, `upload_files`, `download_files`
7. Engineering Reviewer: `execute`, `ls_info`, `read`, `write`, `edit`, `grep_raw`, `glob_info`, `upload_files`, `download_files`
8. Documentation: `execute`, `ls_info`, `read`, `write`, `edit`, `grep_raw`, `glob_info`, `upload_files`, `download_files`
9. Orchestrator: `execute`, `ls_info`, `read`, `write`, `edit`, `grep_raw`, `glob_info`, `upload_files`, `download_files`

The following agents have specialized tools:

1. Skill Creator
2. COTS Search

 Tool definitions:

- `execute` Execute a command in the sandbox and return ExecuteResponse.
- `ls_info` Structured listing with file metadata using os.scandir.
- `read` Read file content with line numbers using a single shell command.
- `write` Create a new file. Returns WriteResult; error populated on failure.
- `edit` Edit a file by replacing string occurrences. Returns EditResult.
- `grep_raw` Structured search results or error string for invalid input.
- `glob_info` Structured glob matching returning FileInfo dicts.
- `upload_files` Upload multiple files to the sandbox.
- `download_files` Download multiple files from the sandbox.

Importantly, we have all these methods as async functions, their names with `aread`, `awrite`, `aedit`, etc. This is likely the preferred way to call all these functions.

The rest (submitting the work, testing for design validity, etc) is called via and calling python functions in the code. (as described below)

Note: runtime is DSPy.ReAct-first with LangGraph-managed architecture. We do not use LangChain `create_agent`; orchestration is implemented in our LangGraph workflows and Python runtime wrappers.

ReAct is the first-line correction loop:
1. The model produces output and calls tools.
2. If a tool/validation response is not acceptable, ReAct continues the same node loop and retries.
3. The node converges on valid handoff output or exits due to hard limits (timeout/turn budget/token budget).
4. 
#### The "tools" as Python functions - Utils

I propose the following set of tools (their usage is below). Notably, the tools are python imports and functions, called right in one of the edited files!

##### Engineer tools

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

##### Benchmark generator (CAD editor) tools

- `validate(Compound) -> bool` the benchmark is validated to not be out of bounds, and not have intersecting:
    - Input object with environment
    - Goal objective with forbid objective
    - Input objective with goal or forbid objectives.
    
    Validated under all environment randomization.

- `simulate(Compound) -> SimulationResult` - a simulation that, unlike the engineering simulation, can not fail, except if not valid as per `validate()`.
- `submit_for_review(Compound)` - submits the whole benchmark compound for a review to `Reviewer` agent node, which can later approve it and thus putting it to the "to solve" pipeline. This call is valid only after current-revision validation/simulation succeed.
- `get_docs_for(type)` - a util invoking a documentation subagent that parses skill and then b123d documentation (local copy, built into container) in search of documentation <!--note: it's probably ideal to have some service like Context7 which does it for us-->
<!-- Note 2: LangGraph subgraphs/subagents composed with DSPy modules are what we'll use here.-->

#### Planner tools

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

#### Exact tools logic

I will define exact tool (function) logic to avoid confusion.

##### `validate_and_price(component: Part|Compound)`

Run the workbench interface to validate the part for manufacturability; if passes - also run the pricing, if not, return a validation error with fix suggestions. Detects both assemblies and individual parts.

1. Check cache for if we need to reverify the solution, early exit if not.
2. If there is the environment in the assembly (as required by `simulate` command), assert that it is in the correct position.
3. Validate for the manufacturability as per the Workbench interface
4. Validate for being in build zone bounds.
5. Determine cost
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

##### submit_for_review(compound: Compound)

The CAD engineer/coder calls `submit_for_review(compound)` after validation and simulation pass for the latest code revision. This utility persists handover artifacts (including `.manifests/review_manifest.json`) and marks the submission candidate as ready for review.

Reviewer entry preconditions are explicit and fail-closed:
1. `submit_for_review(compound)` must be called in the latest code revision (latest candidate script state), not in an earlier failed revision.
2. The latest validation artifacts must indicate success (`validate()` in benchmark / `validate_and_price()` in  pass).
3. The latest simulation artifact must indicate success and objective completion (target object reached the green/goal zone).
4. `.manifests/review_manifest.json` must exist and parse into a typed model (`ReviewManifest`) with `status=ready_for_review` and matching session/revision metadata.
5. Reviewer model access to `.manifests/**` is denied by policy; reviewer eligibility is evaluated by deterministic system checks, not model-side reads of manifest files.

If any precondition is missing/invalid, it is a handoff invariant violation (not a reviewer decision). The system must:
1. Treat the handoff as invalid and route back to the producing agent with explicit validation feedback.
2. Keep this as fail-closed control flow (`REJECTED`/retry path), not as a successful transition into reviewer acceptance semantics.
3. Escalate to terminal `FAILED` when hard limits are reached (timeout/turn/token budget), rather than circling forever.

Reviewer output contract is strict:
1. Reviewer completion must produce a structured `review_decision`.
2. Missing structured reviewer output is invalid reviewer output and stays in fail-closed routing (with explicit logs), never auto-promoted to acceptance.

#### Dealing with latency

All Python tools require all files to be uploaded. While this is a very rare edge case that an agent would run code tool before the file was edited, we should ensure that this doesn't happen.

### Part metadata

Parts and Assemblies have metadata, e.g. `cots_id` for COTS parts, `material_id` for material parts, and others; e.g. `fixed: bool` for usage during benchmark generation.

Define a classes `PartMetadata` and `CompoundMetadata` that can store all properties related to them. Without these mandatory fields, validation will fail.

### Assigning a part to workbenches

The agent must assign a part manufacturing method to the part. If not, the parts can not be priced or sent for manufacturing.
I suppose we could use

The verification is done by a method.

So suppose the agent's code is as follows:

```py
from build123d import *
from shared.models.schemas import PartMetadata
from shared.enums import ManufacturingMethod

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

To simplify matters *(actually, I couldn't debug pyvista, so I'm doing this)*, the CAD will be rendered by porting to MuJoCo/Genesis and rendering in there. It'll also provide a unified view for the model.
Alternatively, we could use some simple GLB renderer, but MuJoCo/Gensis already do it for us!

#### Rendering views

I presume the model will need to render a view or a set of views to get an understanding of what's happening during the simulation. Allow an extra `view_angles` parameter on `simulate` to trigger simulation from different sides, which would essentially reposition a camera (or a multiple) to render.

### Workbench technical details

Technical details of manufacturability constraints are discussed in spec 004 (not to be discussed here; however manufacturability is determined by deterministic algorithms.)

The workbench validation (as well as other util infrastructure are read-only in the container)

### Supported workbenches

3D printing, CNC and injection molding are supported.

<!-- In the future, it's very interesting to support topology optimization, but that's a separate project. -->

### Off-the-shelf parts (COTS)

It is self-understanding that engineers will use off-the-shelf parts - motors, fasteners, gears, etc. The catalog is well-defined in spec 007, but the model should have an access to a CLI tool or a set of python scripts to find something in a codebase. Again, we use DSPy.ReAct.

I suggest using a subagent for this. Give a prompt of what's necessary and let the subagent execute a series of read-only SQL prompts over a catalog DB. The agent will return a series of catalogs to use.

Both planner agent and engineer can only prompt the searching agent for searching.

## Code and responsibility separation

We are building open-source, and we trust our compute nodes, so no complex "zero-trust" architecture is necessary.

All the code that runs in the controller app is in `controller/`. Worker code is split into `worker_light/` and `worker_heavy/`, with shared worker contracts in `shared/workers/`.

Controller, worker-light, and worker-heavy each have their own container files.

Controller is responsible for scheduling jobs and using LLMs (and containing secrets, except optional skill-sync credentials). Worker-light and worker-heavy perform execution workloads, with ephemeral session storage and explicit API boundaries.

### Database(s)

We have multiple databases in the project: Postgres for Temporal and for Langfuse (because Langfuse requires a Postgres database) - they are running on a single machine but have different DATABASE partitions.

For ease of deployment, I decided to use Postgres for the controller app too.

As the goal of the project is to store solutions and intermediary outputs, the Assets of the projects - the final code outputs - will be sent to S3 (railway) buckets.

Any non-file persistence done by worker node ephemerally for which it does not need to report to the controller (for whichever internal processes, e.g. scripts; I doubt this will ever be necessary) is done on a local SQLite database.

We use SQLAlchemy and alembic for management of controller and worker databases.

We assume that worker databases are ephemeral.

#### Assets

Instead of duplicating traces (which are probably verbose and take space), I will link results to traces to each other.

The assets will be stored on S3 as discussed above. The Postgres database will track them.

### Repo code structure
<!--failed -->
The repository should be in a monorepo structure and as follows:

```text
config/
dataset/evals/
frontend/
worker_light/
worker_heavy/
controller/
shared/
  workers/
<!-- optionally, workspace, to store agent output during testing to not have them appear in the file. It is basically a tempdir.(perhaps, should be moved to `/tmp/`.)-->
/workspace/ 
```

and OpenAPI schemas for controller and worker services (`worker_light` + `worker_heavy`).
<!-- Ideally, The worker and controller have separate `pyproject.toml` and `uv` locks. But it's fine to ignore for now as they have shared tests.-->

### Logging

Structlog.
<!-- Looks nicer. -->
For utils used in agent, a simple `logging` is acceptable too.

### Testing

Mandatory testing of happy and expected fail paths. Integration tests in docker-compose to not fail in production.

Github CI/CD too.

We have a list of tests in `kitty-specs/integration-tests.md` - over 50 tests which forbid usage of mocking. This is the only real way to test the application properly.
(Agents: when debugging integration tests, please follow the quality checks in the document!)

### Networking

We primarily use the internal networking for Railway for inter-node communication.

#### Communication between containers

The controller controls the execution of LLMs, and frontend communicates only to controller, and worker also communicates only to controller. The frontend does not communicate to worker nodes.

#### Multiple agents per machine CPU

A single agent would consume a single cpu only during simulation. Thus, it's ideal to allow multiple agents to run on a single node; but don't allow multiple simulations at the same time to avoid OOM issues or CPU overload.
By default, there should be 4 agents per a machine or two, but only one can simulate (`max_workers=1` in the simulation executor). This is especially critical on CPU-only hardware where Genesis kernel compilation is extremely resource-intensive.
There will also likely be multiple workers. At least, the system should be implemented such that it can (because it makes for clear and easy devops).
This requires agents writing in different directories (presumably /tmp/) and cleaning up after they are finished.

### On container termination

What do we do if the container terminates mid-execution? We have a Temporal workflow for that. For every operation expected to run for more than 30 seconds, store that to Temporal.

Do nothing with it because it's cheaper to resume the extraction than to write the code. (compute is cheap but engineering time is expensive.)

And because we only store repository locally and don't persist files anywhere, we have to retry the entire episode starting from planning.
Store in the database that the episode "Terminated due to container preemption" and retry via temporal (right? or wrong?)
The temporal

<!-- But what do we do with *resuming* the work? should we bother? -->

### Strict API requirement

The APIs are to be strict. OpenAPI schemas will be autogenerated on git commit hooks of controller and `schemathesis` will fuzz API endpoints. We have strict schemas for: worker-light <-> controller, worker-heavy <-> controller, worker-light <-> worker heavy and frontend <-> controller.

We use Pydantic, and we use Beartype for hard type checking.
Node runtime execution requires non-empty `session_id`; missing runtime identity is a hard error.
Unsupported agent role/type is rejected explicitly (no fallback role mapping).
`agent_name` is a strict `AgentName` enum across API request models, orchestration nodes, filesystem policy, and eval runtime contracts.

### Batch support as a first-class citizen

Both batch generation (batch solution, or batch simulation) support and frontend (as below) are first-class citizens.
Batch generation will be ran from a script or a container, offloading to multiple workers.

#### Async execution

For both frontend and for backend, the workers will run async, with a parallel execution requirement.

### Frontend

Frontend specs now live in the sibling `frontend-specs.md` file. It serves as a central point of reference for frotend.

Frontend observability requirements (mandatory):
- Reasoning visibility in chat (`View reasoning`) must be backed by persisted backend traces that are queryable via episode APIs (for example, dedicated reasoning traces per node); frontend must not rely on placeholder-only phase labels.
- Reasoning traces must appear incrementally while the agent is running (streaming), not only after episode/node completion.
- If a run is configured to require reasoning telemetry for debugging/eval, absence of persisted reasoning traces is a validation failure (fail-closed), not a UI-only warning.
- Tool activity rows shown in chat must map directly to backend tool-call traces for that run (no synthetic-only rows).

## Inputs to the system

### Inputs to the Benchmark generator

The inputs to the system are a set of prompts of what the user wants generated. A brief note about what kind of problem is presented (e.g. the simplest is - "Put a ball to the funnel and assert that it will fall to the bottom".)

The Planner will generate a longer plan with rough dimensions and will guide the creation of the agent.

The CAD drafting agent will implement, the reviewer will send back reviews or accept the benchmarks. The reviewer will also review all randomness in the script.

Upon accepting, the environment and its randomized versions will be saved to Assets.

Notably, the environment will also pre-render a set of pictures (e.g., 24 pictures—each from different side) so that the engineering agent does not have to request their generation.

We may also make a script to generate a number of short input prompts via a LLM.

### Engineering agent(s)

The engineering agent(s) get benchmarks as inputs and a standard prompt.

## Security

Minimal security is at least desired. Secure all endpoints with an API key env var that needs to pass in headers. "Admin" endpoints - e.g. reset a database (if ever necessary) will use the

## Outputs of the system and postprocessing

The goal of the application (as stated in the very beginning of the document) is to create dataset. There will also be some postprocessing of the database. There will likely be prompt tuning (Generic Pareto - self-reflective way to optimize prompts via a eval dataset.) All the code for optimization will be in a separate folder.

<!-- This will be specified and executed on later - after the system is running. -->

## Other notes

1. There is no need to reinvent the wheel here. The codebase is to use the best practices. I don't want "innovative" code that is hard to work with and demands 2x of my time.
2. "Fallbacks" lead to bad code. Early termination is preferred. When we are making 3-4 fallbacks which lead to more logic and outdated codebases, it leads to issues for everybody. Need to refactor something? confirm it with me and stay lean. Fail fast if the application fails, because the "happy path" isn't met.
3. Because the application is open-source and asks for reproducibility, use open-source frameworks.
4. Because the application is for a scientific use-case (and should be presented in such a light), detailed statistics should be gathered.

<!-- ### Production workflow

Notably, the production workflow is not an important part *right now* (February 4). We should prioritize the development workflows. -->
<!-- Production workflow became a priority -->

## Complexity tracking worksheet (what is more complex but necessary)

```yaml
Updating skills via git:
  what: 
  We will pull from git to update the skills before every session (e.g. benchmark generation start to finish). The skills will be updated on the next agent execution.
  reason: >
  We need to update to 

tool calls are just python imports: 
  what:
    To make things workable on our side but also easy for the agent, tool calls are always just python functions that are imported into the final call.
  reason: >
    I don't see how agents can reliably run a `simulate.py` script in a function. E.g., the agent would run a command like `python3 simulate.py --source_file solution.py` The issue with this is that - how would we get the exact build123d "result" assembly from it? I don't know. Not without some subprocesses. And with subprocesses, how would we get the part by label? I don't know.

    Why not create a separate tool? because the agents are proven to work better with calling scripts through code and not through tool calls - tool calls confuse them. (reference DSPy.ReAct framework for it, if you wish.)
  additional issues: 
    the real issue is that literally all the submitted scripts would have a `submit()` or similar commands in the end. It's not really a blocked, but it kind of is a code smell. On the brighter side, it will lead to deterministic execution where LLMs can't break business logic - they, for example, could, if the LLM would override an abstract class and would write it's logic in the base class. 
    Not exactly a blocker, but just complexity. 

  git logic and uploading git results to s3: 
    what:
      We want to track code evolution. Hence, we will use `git add . && git commit` during every successful `simulate` call. Presumably, the commit names are static.
    Upon the episode end, we will push the compressed git archive to the s3.
      The repo will always start with "initial commit" message before the agent even starts there (perhaps, it's cloned or similar.)
    reason: 
      We want to track code evolution.
    
```

## Appendix / Deep-Dives
- Keep using existing `specs/desired_architecture_WP*.md` docs as deep-dive references.
