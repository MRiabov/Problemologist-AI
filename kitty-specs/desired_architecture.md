# Desired architecture (human written)

## Objective of the system

We are to create a benchmark and a training dataset for evaluating LLM models on creating and solving dynamics problems.

### Outputs and end goals

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

## Agents

We have two agents (or agent graphs) - the benchmark generator and the engineer solving these issues.

### Benchmark generator agent (or graph)

The benchmark generator writes code for the and gets internal reviews by the reviewer inside of the generator.

The benchmarks are randomized to ensure data distribution.

The benchmarks are consisting of CAD models which are converted into XML.

- The execution runs in isolated containers to prevent accidental harmful code execution in the main system.
- The benchmarks are verified for being creatable in MuJoCo. They are not solved by an engineer yet, just benchmarks are created and verified for validity.
  - Validity means no intersections and other problems; It also means that the code will compile in the first place.
  - MJCF is verified for the correctness by XML schema. And also by running a few frames of a simulation
- MJCF is created programmatically, not by a LLM.
<!-- I will need to experiment, but I don't think the LLM should be able to edit it.->

#### Benchmarks

The environments are made with static objects, dynamic objects, and motors. <!-- note - not sure if I handle dynamic objects atm. If I do, how are they specified in CAD? -->

Motors require power to run, however, we don't bother with "wiring" yet.

Problems with motors and moving parts are verified more consistently because they are more prone to error.

<!-- note: it would be useful to persist an "expected solution" from the planner during task generator. It'll help guide exploration and maybe improve LLM optimization (prompt reflection/RL) with more data. -->

### Engineer (problem solver)

Writes CAD code to solve the problems (the benchmarks as above).

Is constrained by cost and manufacturability. Also constraints such as weight.

Has access to all realistic CAD tools.

The agent can preview their tools and check for whether the designs are manufacturable without being "punished".

It is OK if agents are running for 10 or more minutes (in production).

Again, the execution runs in isolated containers to prevent accidental harmful code execution in the main system.

#### Engineer agent details

Engineer has a planner, which is responsible for architecting the solution, the engineer which actually implements the solution, and the critic.

The architect will create and persist a TODO list. The engineer must implement. The agent will have easy access to the TODO list.

- The engineer, after *proving* the TODO list is impossible, can refuse the plan.

<!-- Standard best practices will be used. No point in making anything weird in this application. -->

#### Verification

The Engineer agent will verify its work by:

1. Checking the manufacturability of its solution, on
  a. Individual part scale (this part can be machined)
  b. Assembly scale - this assembly has no interference of parts and has valid part constraints.
2. Checking the cost of its solution; against the part count and unit cost as specified by the user.
3. Checking the weight of its solution.
4. Simulating - did the model achieve the goal as per the benchmark?
5. The critic will assess whether the simulation is stable or not - will it be allowed to work in a real scenario? Is it flaky?

### Agentic framework

We use LangChain and LangGraph for the agentic infrastructure.

### Filesystem

Both of the agents "live" directly in the filesystem of the container that they have been assigned to and thus runs their workflow. This serves the purpose of reducing complexity in tooling, and giving the agents the familiarity with editing tools. There are skills, a script to be written, and verification tools in the script.

#### File updates

The file writes don't persist in the Controller except into the observability database. Their edits are sent directly into the container mounted (ephemeral) storage.

To update the files, or execute other functions: the controller parses the tool call locally; then sends the request over to the worker to execute whatever is related to the tool - read file, write file, or edit file (perhaps, `deepagents` handles the network part too? I think it does, via the Filesystem...).

The network latency is perfectly acceptable as LLM reasoning latency far outweights the ping time.

#### Utils files

The agent has a set of utils - python scripts (files) that the agent can import from. These are explicitly unwritable by the `deepagents` `FilesystemMiddleware` (which can force certain files to be unwritable by the in agents) the filesystem.

#### Skills files

Unlike utils files which are static (or the container restart is acceptable), skills files will be updated by a sidecar learner agent rather frequently (possibly after every 10-20 conversations?).

The container will download the pull skills from a skill git repo before every run (justified? else how do we handle the updates? The file size there is minimal.)

Maybe use a dedicated public git repo? I think it's sound. However this is complexity, and probaly isn't that sound. But it'll enable human readability, which is a bonus.

*Note: the skills are to be versioned either way.*

*Note on observability: for observability and reproducibility, the skills usage is to be tracked in the database.*

#### Starting folder structure for various agents

We define the file structure as follows, individual agents adapt to individual needs:

```text
.
├── skills/                     # [Read-Only] Learned skills and documentation
├── utils/                      # [Read-Only] Fixed utilities and shared code
│   └── ...
├── renders/                      # [Read-Only/tool-generated] (Renders of images and  from the enviornment) 
│   ├── images/                 # Images from 24 angles (8 images clockwise on 3 levels).
│   └── videos/                 # Video of the simulation. (latest simulation video.)
├── reviews/                     # [Read-Only] Reviews of the reviewer agent.
├── journal.md                  # [Read-Write] Decisions, reasoning, and execution log
├── todo.md                     # [Read-Write] Execution plan
├── plan.md                     # A plan
└── script.py                   # [Read-Write] Main execution script (from template)

<!-- The agent can create more than one .py file. -->
```
<!-- Important note: some of these are always fetched from the s3 and are only "fake" for the agent. The "fake" is done by FilesystemMiddleware. -->
##### Benchmark generator (CAD agent)

1. Skills (read-only)
2. A template for creating benchmark generator files (read-write)
3. A TODO list (read-write, validated (validated how?))
4. A plan from the planner (read-only)
5. A journal (read-write)

Utils (read-only):

1. Refuse plan (a script that sends a request to the endpoint) (shouldn't happen, realistically.)
2. Utils necessary for functioning (as described in other parts of the document)

##### Engineer

1. Skills (read-only)
2. A template for an engineer files solutions (read-write)
3. A TODO list from the planner (read-write)
4. A plan from the planner (read-only)
5. A Journal (read-write)

Utils (read-only):

1. Refuse plan (a script that sends a request to the endpoint)
2. Utils necessary for functioning (as described in other parts of the document)

##### Planner (with different templates for engineer planner and benchmark planner)

1. Planning skills
2. A markdown plan template (auto-validated, refuses pass if doesn't match template.)
3. A TODO list from the planner

### Agent artifacts

The important part of managing agents are their artifacts.

#### Journal

The Journal is the agent's **Episodic Memory**. It is a structured log (a constrained Markdown) where the agent records a high-level narrative of its execution.

**Purpose:**

1. To help the agent remember what it did: what it tried and what didn't work:
  a. Intent,
  b. Result,
  c. Reflection
  d. Next step.
2. To help maintain context amongst task solving retries, and with user conversations.
3. (implicitly, a non-functional requirement) to help debug the agent.
4. (implicitly, a non-functional requirement) to help train the agent
5. (implicitly, a non-functional requirement) to help create skills for the agent.

(note: we should not optimize the journal architecture for the non-functional requirements.)

##### Journal lookup for learner agents

The skill learner agent should be able to dig and scrutinize into why something has happened without overflowing their context. The entries in Journal should contain the "start and finish" on the token timeline.

The agents will delimit their reasoning with markdown headings, which would allow easier disclosure on top of the "start and finish".

#### TODOs

The Planner will build a list of TODOs for the agent to do. The critic will verify against the plan and the list of TODOs.

#### Reviews by reviewers

Reviews will be written by a markdown document and stored in `/reviews/` folder, to be able to access it later.
Notably, to keep the agents accountable and enforce stricter and more consistent typing, agents would write a yaml frontmatter on reviews - e.g. `planned`, `has_feedback`, `done`
<!-- Note: the entire schema was not defined yet. However, I like how spec-kitty that I use in the project does it. -->

### Token compression

As the agent will near its token generation limits, they will compress their old memory by a summarizing agent.

### Skills

The agents will be able to create and update their skills to improve the future runs. This is vital for continous improvement, and will be used alongside prompt improvement. *Skills* are actually the information that the agent doesn't know and needs to learn - for example how to use `build123d`.

- If the agent fails too much, after resolving a solution, they will persist it to the skill.

We use the `SKILL.md` format as by Anthropic. I refer to ".agent/skill-creator/SKILL.md` for more information about skills.

We know that agents can not use `build123d` very well despite it being one of the central part of their work in this repo. This is part of a mitigation.

#### `build123d` skill and access to documentation

The agents will have an access to build123d documentation throught the skill (as per the SKILL.md standard, in `/references/ folder.)

#### Different agents have different skills

The "Engineer" agent does not need a "benchmark creation skill". It could be necessary the other way around. Nevertheless, agents access to skills should be configurable by default. If an agent has created a new skill, let it stay just to that agent.

#### Explicit skill agent

The agents can not be trusted with updating the skill well, and they may be out of context. Quite likely, implement a separate "learner" agent node that runs after success, probably async of the agent.

It will work over smaller chunks of uncompressed info (how exactly? Maybe progressive disclosure? I think implement a progressive disclosure from a journal)

The skill agent will read a `skill-creator/` skill as from Anthropic.

##### Skill agent is run async

The skill agent is run asyncronous to the execution, modifying the skill folder and pushing it to a git repo. It's filesystem is managed by `deepagents` (as an exception), is stored on (the same) Railway bucket under the folder (because it's easier from the deployment perspective).
The containers will likely have an endpoint to update the skills without restarting. However, for agents, skills are read-only.

##### Skill agent has a journal too

Skill agent has a journal of:

1. Issues faced and what they resolved to (or weren't resolved)
2. Commonly faced problems (happened more than twice) and solutions to them

If the patterns emerge in the journal; or the solution is obvious enough, the learner writes it into the skill.

###### Structure of the journal

1. Observed struggles (tool calls failed over 4 times)
2. Found solutions to struggles (happened once)
    - The skill builder agent may make mistakes, so, observe the patterns at least twice.
3. Skills to add in the end (happened twice).

Importantly! the agent **must** write an ID of observation and link it to a journal entry. This way, they can:

- Link the found "breakthrough" to an exact problem
- Read the actual reasoning and outcomes from journals instead of putting it to memory.

When writing the skill in the end, they will write a skill from the actual reasoning of the agent, and not be confused.

<!-- Perhaps, make a python script that would add them as entries and make the journal read-only except for the modification of the script? e.g. spec-kitty does this with YAML frontmatter. -->

###### Using a sidecar agent for Journalling

It was proven to be more cost-effective to use a "sidecar" agent that runs in paralel to the primary model. We will use an inexpensive model, such as DeepSeek 3.2 or small models.

<!-- Note: it was found that YAML/Markdown are the most effective for storing model outputs structured information output. YAML, later converted to markdown programmatically is likely preferable? -->

###### When to write in a journal?

As above - when the agent has spent >4 tool calls in a single conversation trying to figure out an issue or syntaxis, or
This also applies to refactors. If the agent has taken an approach, spent 5 tool calls doing a roundtrip and decided against it in the end, note the architectural issue.
In both cases, if the agent found the solution

The agent will go through all notes in the session and read through ones that are relevant. They will then link the found and solved issues.
<!-- Note: maybe add a "hypothesis" - an agent would be perform better if we X? -->

###### Example

1. Observation: Mulitple agents have an issue with struggling to group parts together into a `Compound` - they had more than four unsuccessful tool calls trying to do it(struggle) .
2. The learner agent records the issue (async)
3. The main model finally finds that `Compound` syntaxis requires `Compound(children=[Part|Compound, ...])` syntaxis (with `children` upfront)
4. The learner agent records the found solution and the skill.

#### Worker skills are persisted and are separate from the repository-level skills

Skills in the `.agent/skills/` in the repo root are different from the agent skills we are learning in the database! The repo root skills are for the coding agent to write this codebase. The learned skills should be, e.g. in workspace/ folder.

### Skill versioning

Because the application's performance is quite dependant on SKILL.md files which detail how to use `build123d` and other issues, however those are frequently updated by a "learner" agent, the skill git hashes need to be versioned and persisted to the observability database.

### `deepagents` framework

LangChain and LangGraph developers have introduced an abstraction over LangChain and LangGraph called `deepagents`. This is a system in particular suitable for creating long-running, complex agents, for example coding agents. It has access to spawning subagents, long-term memory, skills, filesystem capabilities which essentially encapsulate all functionality that we need.

- We use `FilesystemMiddleware` to support `ls`, `write`, `read`, `edit`, and `execute` (notably, their async versions too, which are called `awrite`, `aexecute`, etc.). Notably, we are natively supporting a "sandbox" storage in workers - `deepagents` has a `SandboxFilesystemBackend` integration which allows for safe, disposable environment.
- We use `TodoListMiddleware` which provides a `todo_list` to support TODO lists.

<!-- Note to LLMs! `deepagents` was introduced in late 2025 and you don't know much about it, but it is a framework for managing agents.

Overview: https://docs.langchain.com/oss/python/deepagents/overview.md

 -->

#### Linting

The agents will receive the linting from the tools. The linting is done on the worker nodes for safety and performance. The agent will have `ruff` and/or `pyright` on the device (I don't think pyrefly is necessary here; standard, non-based pyright will suffice).

The linting will happen at any `write` command; can be triggered by `run_command` command using `ruff`/`pyright`..

### Execution process

The agents (both the engineer and benchmark generator) will create a set of files to solve a given problem. If a benchmark requires 10 different objects, they could create multiple files to solve them, or reuse an existing one.

### Feedback to the agent

It was proven that agents work better with markdown (in general) than JSON (probably due to familiarity with petabytes of text); thus, pass all textual, structured (e.g. text and JSON) feedback in the `markdown` format.

#### Feedback from the simulation

The simulation returns the data from *video*. There will be a simple text summary prepended to the video, e.g. "the agent failed to hit the objective"

The agent (the engineer, critic or another "summarizer") will write the video summary to the Journal.

##### Compressing the video

Future work will need to address the issue of the video being too expensive. Instead, a "T*" agent could be implemented - a small model that picks the most important shortcuts from the video and feeds it to the agent. This will significantly reduce the pollution of the engineer's agent's attention.

#### Feedback from cost and manufacturability constraints

The agent will receive feedback from cost and manufacturability constraints (basically, workbenches) in markdown.

### Agent handovers

#### Benchmark generator with engineer handover

The Engineer agent(s) have can access to meshes and a exact reconstruction of the environment as a starting point to their build123d scene, however they can not modify/move it from their build123d scene. In fact, we validate for the fact that the engineer wouldn't move it or changed it (validating for changing it via hashing) - in both MJCF and build123d.

Additionally, the engineering agent will be supplied with renders for preview automatically rendered from 24 views. (Clockwise, 8 pictures, on 30 degrees up or down (configurable)).

The engineer will receive exact positions of objectives in a YAML file, and information on randomization of starting object positions, so as to prevent guessing.

#### Coder and Reviewer interaction

1. The reviewer will have access to all files of agents in read-only mode (note: questionable decision - why would they need code files?). Primarily, they will focus on reviewing the video and image files for a more realistic review (presumably directly from the Railway bucket, if filesystem allows it). Thus the Reviewer will only have readonly on all agent files permissions.
The reviewer will also have `write` and `edit` tool with permissions of editing a single "reviews/review-round-[round number]" folder.

The goal is to persist the reviews into a persistent file which the agent can reference at any time (alongside previous reviews), and see it only once; and to avoid plumbing to route "reviews" text when required.

The reviews will also come with the following YAML frontmatter:

```yaml
decision: approved # [approved, rejected]
comments: [
   # short commentary on issues, as a array.
   # Up to 100 chars per issue.
]
```

As usual, the reviews will be strictly typed.

## Distributed execution

There is a controller node which runs the LLM and tool calls, and there worker node which:

1. Executes the simulation
2. Executes the python scripts.

For both safety and performance reasons, it desirable that the LLM-generated scripts are never executed on the controller machine.

In the future we may well refactor to run on distributed notes, perhaps even IPv6.

### Persistent state and durable execution

To simplify app logic and avoid writing retry and other "safety" logic, we will deploy a small `temporal` instance running on a separate small machine next to the main app.

### Hyperscaler of choice

We are deploying to Railway. (In production, we may deploy workers to inexpensive, *batch* IPv6 nodes - SaladCloud, or use a on-prem infra that somebody will give to me...)

#### Deployment specifics

Railway supports docker-compose import and we will start with the docker-compose. This is the easiest choise and I have familiarity with it (k8s too bloated for this purpose)

#### Podman containers

We decided to run in Podman containers because they are leaner than Docker and serve all same purposes. Worker containers have an attached volume.

##### `uv` base containers

We will use astral-sh/uv as base containers and not standard python ones (for quality).

### Persisting files

The files are written directly to the worker container. We don't store it on controller. However, we upload final results ("Assets") to the Railway bucket S3.

The worker's filesystem is implemented as a "disposable sandbox" via `SandboxFilesystemBackend` in ``deepagents`.

The "main app" essentially serves as a business logic layer that also forwards requests to observability layer, but the actual execution - from linting to simulation - happens in the worker container.

#### Workers' filesystem

`deepagents` `FilesystemMiddleware` supports a "sandbox" filesystem backend. This is handy, and we will expose the workers' filesystem as sandbox - something that can easily be removed..

Notably, the files can be created locally (e.g. video, image, MJCF outputs), and something should be done about it.

The filesystem will be reset (and pull skills) (how? presumably just by deleting and cloning a copy - perhaps from local) on every run from a github repo.
For videos and large files, we will also use a `CompositeBackend`. It will route the `/render/` folder to the s3; we will need internal plumbing to make this happen (presumably, any python function that will render, will upload to s3).

#### Videos

Videos are the largest artifact that we will need to generate and store.
To ensure consistency, we will upload them to Railway buckets.
This is OK as we already work on Railway and internal networking both fast, easy to set up and inexpensive.
Storing videos on buckets is also cheaper than in volumes.

I suppose the videos will be automatically deleted after a short period, e.g. a day to avoid storage costs. <!--(why not store them in ephemeral storage then?...)-->

Videos are rendered on-demand and only if the model requests it (I'll need to double-check if is preferable or not)
Why: models don't strictly always need to view videos; they might want to view only coordinates in the end (videos can be rendered by a variable in `simulate(component, render=True)`) this is in part to make video.

### Database

We do persistence via SQLAlchemy and Alembic migrations to avoid issues with hand-written SQL.

<!-- If something is a long-running process, prefer to persist it to a database and receive a callback. -->
<!-- Solved via Temporal.  -->

All important updates must be persisted into the DB (for observability, as below.)

### Agent and Worker boundary

#### Separation

The Agent (managed by LangGraph) never "knows" about distributed workers. It only calls an async Python function (a tool). It is the job of the tool to dispatch a job and handle the retry. In this case, retries and persistence are handled by Temporal. LangGraph handles the retries in case the LLM call fails.

#### Temporal

Temporal is used to orchestrate the workers. It is not used to run or retry the agent.

Temporal needs a database and we will use the Postgres database used by temporal, except under the different `DATABASE` partition.

Because tasks like simulation (with involve both simulation and uploading to the database) could be long-running we are using webhooks and callbacks to report their completion.

## Simulation and "Defintions of Done"

While this platform has notable downsides for future use, we pick MuJoCo, because it's battle-tested, and requires almost no compile time.

<!-- Downsides of MuJoCo?

- we won't support deformation (finite element analysis)
- we won't support fluids

But for an MVP this is fine. -->
<!-- The corollary of not being able to run FEM is that the model can produce physically inadequate parts and still succeed. But I can't do much about it yet. -->

### Definition of "success" and failure in the simulation

We want to support one primary use-case: moving an object from one position to another, using motors and gravity; avoiding forbidden zones and staying in simulation bounds.

<!-- another use-case could be: given a severe constraint in positioning, design a system which would support a given load. However, the issue is that it's not  -->

#### Moving an object from one screen to another

We define the "simulation objective" from four components:

1. A "build zone" - where the agent can actually create parts (note: the agent is forbidden to construct outside of this zone),
2. A "goal zone" - the area to which the goal object needs to move to,
3. The moved object - the object which is spawned to be moved into the goal
4. A "forbid" zone - an area none of the simulation objects agent may not go into.

The objectives are always axis-aligned bounding boxes (AABB) for simplicity. The forbid or goal zone is triggered if the agent touches it even slightly.

Additionally, the simulation is constrained by the bounds of the simulation, i.e. the space which the simulation can not leave.

#### Randomization

<!-- LLM-generated from my other spec. -->
The benchmarks are randomized to enable a wider data distribution with less generation effort.

- The benchmark volume size can vary 2x in all sides, and will be rescaled to random values, e.g. 1.68\*0.8\*1.3; the benchmark generator agent can narrow the scaling down if somehing is expected to break; however it is undesirable as we want to keep randomization higher.
  - The environment - ojectives (goals, forbids, build zones) are rescaled respectively.
- Goal, and obstacle positions are randomized by up to 40% of their size inwards (meaning they are becoming smaller and repositioned anywhere in the region where they are becoming smaller; smaller by their own size. They will always stay within original (maximum) bounds for safety).
- The spawned "moved" object will also include some position jitter to ensure the CAD model's robustness against variable input.
- The models that make up the scene can and should be different. Up to the point where it can be solved; but the benchmark generation agent must ensure randomization of the environment too; which can be also made quite extreme (which is preferred - we are not to make it easy.)

#### Failure

Failure is achieved via either of:

1. timeout of the simulation
2. Any of components going out of bounds of the workspace.OR goign out of bounds OR instability in simulation

## Observability

To track all agent movements and to persist data, we encode the following:

1. The agent pass/fail reasons
2. Error messages from script execution, linting,
3. All agent thoughts.
4. A mechanism to reconstruct those - e.g. we record the entire conversation and tool-calling structure, so how can we read it? How can we show it to users that use this prompt? How can we use it for debugging? basically, some order matters. Or, maybe just dump the conversation in/out to schema, that could also work.
5. Renders (visuals) of what the agent sees (optionally; if it doesn't take too much space; it may very well be more economical to reconstruct at runtime using a scripts. So record what comes into the agent, all parameters, code, setup, etc, and rebuild at runtime.)

These will be later used for querying, preproc and model training.

### Langfuse

We use LangFuse for LLM observability. We will use a Railway template / deploy a separate container to the langfuse. This will require a Postgres DB.

### Backups

In prod we will backup the database(s) daily.
<!-- Notably, the file could be quite big, as we persist sqlite text. Max compression it before backing up. -->

One way to do it is by sending a `cron` job daily. Thus, implement an endpoint which will accept a cron call, and will back up the SQLite folder to the s3. Again, this is in production.

## Strict schema

We will run `schemathesis` checks against the OpenAPI. Strictly type all schema to avoid ANY issues.

We use Pydantic and Beartype for this.

### Beartype

To further avoid any issues, we will use Beartype for type checking.

### Schema autogeneration

We autogenerate python schemas, keeping in sync to the workers. We keep schemas defined in the Controller app, the worker and frontend inherit it. (for now). We have git hooks that implement the model.

## "Workbenches" - manufacturability verification

The agents will have *Workbenches* - a set of tools they can use to:

1. Verify their manufacturability.
2. Calculate costs of their parts (and verify against the user-inputted goals)

## CAD and and design validation

As said, "agents will live inside of a filesystem". The agents will generate and execute design validations of files in the filesystem.

### Tools

#### "Agent-native" tools (callable by LangChain)

(Experiment:) The agent only has a minimal set of tools appropriate for a coding agent: `ls`, `view_file`, `edit_file` (edit some lines in a file), `write file` (write/overwrite the entire file), and works in the filesystem (the filesystem is described as above), `execute` (runs a shell command, e.g. python -m ...) and `wait` for waiting for the agent. The (engineering) agent will validate, cost-estimate, verify, submit; the benchmark generator agent will create, test, render (visually view) its environment *only via script calls*.

We also have other notable, only possibly useful commands (as from [deepagents documentation](https://reference.langchain.com/python/deepagents/backends/sandbox/)):

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

The rest (submitting the work, testing for design validity, etc) is called via and calling python functions in the code. (as desribed below)

#### The "tools" as Python functions - Utils

I propose the following set of tools (their usage is below). Notably, the tools are python imports and functions, called right in one of the edited files!

##### Engineer tools

- `validate_and_price(component: Part|Compound) -> float|dict[str, float]`: validates a part by for manufacturability, then prices it if valid using its workbench's cost calculation interface, or returns an error with a description and a location
- `simulate(Compound) -> SimulationResult` - Submits a model for a simulation. Should run multiple simulations with slightly perturbed object spawn position; to make sure the engineer agents generate robust solutions.
<!-- dev note: assert against submitting a BuildPart builders, or other types. -->
<!-- should it contain its environment model or only the generated model?  -->
- `submit_for_review(Compound)` - submits the whole assembly for a review to `Reviewer` agent node, which can later approve it and submit return the final design to the user.
<!-- Same: what's in the compound? -->
- `get_docs_for(type)` - a util invoking a documentation subagent that parses skill and then b123d documentation (local copy, built into container) in search of documentation <!--note: it's probably ideal to have some service which does it fo us-->

##### Benchmark generator (CAD editor) tools

- `validate(Compound) -> bool` the benchmark is validated to not be out of bounds, and not have intersecting:
- Input object with environment
- Goal objective with forbid objective
- Input objective with goal or forbid objectives.

Validated under all environment randomizations.

- `simulate(Compound) -> SimulationResult` - a simulation that, unlike the engineering simulation, can not fail, except if not valid as per `validate()`.
- `submit_for_review(Compound)` - submits the whole benchmark compound for a review to `Reviewer` agent node, which can later approve it and thus putting it to the "to solve" pipeline.
- `get_docs_for(type)` - a util invoking a documentation subagent that parses skill and then b123d documentation (local copy, built into container) in search of documentation <!--note: it's probably ideal to have some service like Context7 which does it for us-->
<!-- Note 2: `deepagent` framework supports subagents this is what we'll use here.-->

#### Exact tools logic

I will define exact tool (function) logic to avoid confusion.

##### `validate_and_price(component: Part|Compound)`

Run the workbench interface to validate the part for manufacturability; if passes - also run the pricing, if not, return a validation error with fix suggestions. Detects both assemblies and individual parts.

1. Check cache for if we need to reverify the solution, early exit if not.
2. If there is the environment in the assembly (as required by `simulate` command), assert that it is in the correct position.
3. Validate for the manufacturability as per the Workbench interface
4. Validate for being in bounds
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
4. If simulation passes, notify the engineer via logs. (don't ask the agent to improve for now, though it could be well cost-efficient and useful). The agent will then run a "submit for review.
    - Don't render the video yet! If the simulation didn't pass, maybe we don't need to render the video. We can instead print positions (probably just final positions) of all parts in the simulation and let the agent introspect them.
    the simulation will produce video. The issue is, it's expensive to
5. If doesn't, retry the simulation.

##### submit_for_review(compound: Compound)

The CAD engineer agent run `simulate(),` will ask a reviewer agent to review. If already the environment was already/recently simulated, the cache will hit and will skip directly to the review.

#### Dealing with latency

All Python tools require all files to be uploaded. While this is a very rare edge case that an agent would run code tool before the file was edited, we should ensure that this doesn't happen.

### Assigning a part to workbenches

The agent must assign a part manufacturing method to the part. If not, the parts can not be priced or sent for manufacturing.
I suppose we could use

The verification is done by a method.

So suppose the agent's code is as follows:

```py
from build123d import *
from utils import ManufacturingMethod, submit # mock
from src.workbenches import validate_and_price
from 

with BuildPart() as part_builder:
    Box(10,10,10)

part_builder.part.label="custom_label".
part_builder.part.metadata = {
  "manufacturing_method": ManufacturingMethod.CNC 
  "material-id": "aluminum-6061"
}

validate_and_price(part_builder.part) # prints ("Part {label} is valid, the unit price at XYZ pcs is ...)
```

### Workbench technical details

Technical details of manufacturability constraints are discussed in spec 004 (not to be discussed here; however manufacturability is determined by deterministic algorithms.)

The workbench validation (as well as other util infrastructure are read-only in the container)

### Supported workbenches

3D printing, CNC and injection molding are supported.

<!-- In the future, it's very interesting to support topology optimization, but that's a separte project. -->

### Off-the-shelf parts (COTS)

It is self-understanding that engineers will use off-the-shelf parts - motors, fasteners, gears, etc. The catalog is well-defined in spec 007, but the model should have an access to a CLI tool or a set of python scripts to find something in a codebase. Again, we use CodeAct.

I suggest using a subagent for this. Give a prompt of what's necessary and let the subagent execute a series of read-only SQL prompts over a catalog DB. The agent will return a series of catalogs to use.

Both planner agent and engineer can only prompt the searching agent for searching.

## Code and responsibility separation

We are building open-source, and we trust our compute nodes, so no complex "zero-trust" architecture is necessary.

All the code that runs in the controller/main app will be in the controller node directory, and all the worker node will be packaged into the controller node.
<!-- note: the above was failed. -->

Both controller and worker will have their own container files.

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

```
frontend/
worker/
controller/
```

and a schema with a OpenAPI schema. The worker and controller have separate `pyproject.toml` and `uv` locks.

### Logging

Structlog.
<!-- Looks nicer. -->

### Testing

Mandatory testing of happy and expected fail paths. Integration tests in docker-compose to not fail in production.

Github CI/CD too.

### Networking

We primarily use the internal networking for Railway for inter-node communication.

#### Communication between containers

The controller controls the execution of LLMs, and frontend communicates only to controller, and worker also communicates only to controller. The frontend does not communicate to worker nodes.

### On container termination

What do we do if the container terminates mid-execution? We have a Temporal workflow for that. For every operation expected to run for more than 30 seconds, store that to Temporal.

Do nothing with it because it's cheaper to resume the extraction than to write the code. (compute is cheap but engineering time is expensive.)

And because we only store repository locally and don't persist files anywhere, we have to retry the entire episode starting from planning.
Store in the database that the episode "Terminated due to container preemption" and retry via temporal (right? or wrong?)
The temporal

<!-- But what do we do with *resuming* the work? should we bother? -->

### Strict API requirement

The APIs are to be strict. OpenAPI schemas will be autogenerated on git commit hooks of controller and `schemathesis` will fuzz API endpoints. We have two schemas - one between worker and controller, another between the frontend and the controller.

We use Pydantic, and we use Beartype for hard type checking.

### Batch support as a first-class citizen

Both batch generation (batch solution, or batch simulation) support and frontend (as below) are first-class citizens.
Batch generation will be ran from a script or a container, offloading to multiple workers.

#### Async execution

For both frontend and for backend, the workers will run async, with a paralel execution requirement.

### Frontend and debugging infrastructure

I will need some frontend. I suggest designing a custom UI. This is relatively easy now because we can handle off to Google Stitch (Google's AI website designer; however it only designs websites, it doesn't integrate them). Plus it's convenient to use in backend. The website will be deployed on Vercel for simplicity (or a railway bucket maybe? it doesn't really matter.)
A detailed specification of what needs to be in frontend to create a good UI will be required.

This means that we will have a user-facing API.

<!-- I used streamlit in the past and it works but is limiting due to inability to stream data, as far as I've seen it.-->

#### Debugging requirements

As the agents are long-running (and cost money!) it is desirable to be able to:

1. Submit requests one-by-one (as opposed to working in batch over a dataset of prompts)
2. View their reasoning prompts
3. Interrupt them before they finish.
4. (dev only) using an environmental variable on all all nodes(dev_mode=True), fetch logs from either controller or worker (and maybe view in UI) (this may be easier as we use `structlog`)

##### Interrupting the worker and progress bars

If we want to stop the generation in the controller, it will also halt the job(s) in the workers.

Notably `deepagents` has a [support for this](https://docs.langchain.com/oss/python/deepagents/human-in-the-loop.md) - for reviewing and interruption.

#### Frontend architecture

Vite, React. Autogenerated types on git hooks from Controller.

### Config

Config - benchmark generation config, linting config, manufacturability and pricing config, and prompts will be stored in YAML files.

## Inputs to the system

### Benchmark generator

The inputs to the system are a set of prompts of what the user wants generated. A brief note about what kind of problem is presented (e.g. the simplest is - "Put a ball to the funnel and assert that it will fall to the bottom".)

The Planner will generate a longer plan with rough dimensions and will guide the creation of the agent.

The CAD drafting agent will implement, the reviewer will send back reviews or accept the benchmarks. The reviewer will also review all randomness in the script.

Upon accepting, the environment and its randomized versions will be saved to Assets.

Notably, the environment will also prerender a set of pictures (e.g. 24 pictures - each from different side) so that the engineering agent does not have to request their generation.

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
2. "Fallbacks" lead to bad code. Early termination is preferred. When we are making 3-4 fallbacks which lead to more logic and outdated codebases, it leads to issues to everybody. Need to refactor something? confirm it with me and stay lean. Fail fast if the application fails, because the "happy path" isn't met.
3. Because the application is open-source and asks for reproducibility, use open-source frameworks.
4. Because the application is for a scientific use-case (and should be presented in such a light), detailed statistics should be gathered.

<!-- ### Production workflow

Norably, the production workflow is not an important part *right now* (February 4). We should prioritize the development workflows. -->
<!-- Production workflow became a priority -->

## Complexity tracking worksheet (what is more complex but necessary)

```yaml
Updating skills via git:
  what: 
  We will pull from git to update the skills before every session (e.g. benchmark generation start to finish). The skills will be updated on the next agent execution.
  reason: >
  We need to update to 

deepagents framework: 
  what: >
    We will use a deepagents framework.
  reason: >
    deepagents provides abstractions over filesystem, memory, TODO lists and Subagents. (so-called Middleware.)

tool calls are just python imports: 
  what:
    To make things workable on our side but also easy for the agent, tool calls are always just python functions that are imported into the final call.
  reason: >
    I don't see how agents can reliably run a `simulate.py` script in a function. E.g., the agent would run a command like `python3 simulate.py --source_file solution.py` The issue with this is that - how would we get the exact build123d "result" assembly from it? I don't know. Not without some subprocesses. And with subprocesses, how would we get the part by label? I don't know.

    Why not create a separate tool? because the agents are proven to work better with calling scripts through code and not through tool calls - tool calls confuse them. (reference CodeAct framework for it, if you wish.)
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
