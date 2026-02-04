# Desired architecture (human written)

## Agents

We have two agents (or agent graphs) - the benchmark generator and the engineer solving these issues.

### Benchmark generator agent (or graph)

The benchmark generator writes code for the and gets internal reviews by the reviewer inside of the generator.

The benchmarks are randomized to ensure data distribution.

The benchmarks are consisting of CAD models which are converted into XML.

- The execution runs in isolated containers to prevent accidental harmful code execution in the main system.
- The benchmarks are verified for being creatable in MuJoCo. They are not solved by an engineer yet, just benchmarks are created and verified for validity.
  - Validity means no intersections and other problems; It also means that the code will compile in the first place.
  -
  - MJCF is verified for the correctness by XML schema. And also by running a few frames of a simulation

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

The Engineer agent will verify it's work by:

1. Checking the manufacturability of its solution, on
  a. Individual part scale (this part can be machined)
  b. Assembly scale - this assembly has no interference of parts and has valid part constraints.
2. Checking the cost of it's solution; against the part count and unit cost as specified by the user.
3. Simulating - did the model achieve the goal as per the benchmark?
4. The critic will assess whether the simulation is stable or not - will it be allowed to work in a real scenario? Is it flaky?

### Agentic framework

We use LangChain and LangGraph for the agentic infrastructure.

### Filesystem

Both of the agents "live" directly in the filesystem of the container that they have been assigned to and thus runs their workflow. This serves the purpose of reducing complexity in tooling, and giving the agents the familiarity with editing tools. There are skills, a script to be written, and verification tools in the script.

Clarification: The file writes don't persist locally except into the observability database. They are forwarded (copied) directly into the container.

#### Utils

The agent has a set of utils - python scripts (files) that the agent can import from. These are explicitly read-only in the filesystem, and are "baked-in" to the container. The agent can't enable write constraints on them.

#### Starting folder structure for various agents

We define the file structure as follows, individual agents adapt to individual needs:

```text
.
├── skills/                     # [Read-Only] Learned skills and documentation
├── utils/                      # [Read-Only] Fixed utilities and shared code
│   └── ...
├── journal.md                  # [Read-Write] Decisions, reasoning, and execution log
├── todo.md                     # [Read-Write] Execution plan
├── plan.md                     # A plan
└── script.py                   # [Read-Write] Main execution script (from template)

<!-- The agent can create more than one .py file. -->
```

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

### Token compression

As the agent will near it's token generation limits, they will compress their old memory by a summarizing agent.

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

The skill agent is run asyncronous to the execution, modifying the skill folder and pushing it to the containers.
The containers will likely have an endpoint to update the skills without restarting. However, for agents, skills are read-only.

#### Worker skills are persisted and are separate from the repository-level skills

Skills in the `.agent/skills/` in the repo root are different from the agent skills we are learning in the database! The repo root skills are for the coding agent to write this codebase. The learned skills should be, e.g. in workspace/ folder.

### Tools

(Experiment:) The agent only has a minimal set of tools appropriate for a coding agent: `view_file`, `edit_file` (edit some lines in a file), `write file` (write/overwrite the entire file), and works in the filesystem (the filesystem is described as above). The (engineering) agent will submit, validate, verify, cost-estimate; the benchmark generator agent will create, test, render (visually view) it's environment *only via script calls*.

#### Linting

The agents will receive the linting from the tools. The linting is done on the worker nodes for safety and performance. The agent will have `ruff` and/or `pyright` (I don't think pyrefly is necessary here; standard, non-based pyright) on the device.

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

## Distributed execution

There is a controller node which runs the LLM and tool calls, and there worker node which:

1. Executes the simulation
2. Executes the python scripts.

For both safety and performance reasons, it desirable that the LLM-generated scripts are never executed on the controller machine.

<!-- ## Containerization and paralel execution

We will run benchmark generation and CAD generation/sim in paralel. The system should scale to approx 4 container on a 4-code CPU, at least.-->
<!-- this became implicit as we introduce the container architecture. -->

In the future we may well refactor to run on distributed notes, perhaps even IPv6.

### Persistent state and durable execution

To simplify app logic and avoid writing retry and other logic, we will deploy a small `temporal` instance running on a separate small machine next to the main app.

### Hyperscaler of choice

We are deploying to Railway. (In production, we may deploy workers to inexpensive, *batch* IPv6 nodes - SaladCloud, or use a on-prem infra that somebody will give to me...)

### Persisting files

The files are generated locally by the agent. They are then sent to the container to the prod.

We need to maintain a copy of files locally, and only send requests later, such that files complete their transfer.

### Database

We do persistence via SQLAlchemy and Alembic migrations to avoid issues with hand-written SQL.

<!-- If something is a long-running process, prefer to persist it to a database and receive a callback. -->
<!-- Solved via Temporal.  -->

All important updates must be persisted into the DB (for observability, as below.)

<!-- #### Debugging processes

We need to debug processes and that means we need a folder to store the files locally. During dev, we can use a local podmanfile that mounts/links its subfolders to the local `/workspace/[run-id]` dir. However, in production, we will containerize the main app it would store these in a volume.
And of course, we persist all of the files to the SQLite (with SQLAlchemy and Alembic) for observability. -->
<!-- Forget it. -->

### Agent and Worker boundary

#### Separation

The Agent (managed by LangGraph) never "knows" about distributed workers. It only calls an async Python function (a tool). It is the job of the tool to dispatch a job and handle the retry. In this case, retries and persistence are handled by Temporal. LangGraph handles the retries in case the LLM call fails.

#### Temporal

Temporal is used to orchestrate the workers. It is not used to run or retry the agent.

## Simulation and "Defintions of Done"

While this platform has notable downsides for future use, we pick MuJoCo, because it's battle-tested, and requires almost no compile time.

<!-- Downsides of MuJoCo?

- we won't support deformation (finite element analysis)
- we won't support fluids

But for an MVP this is fine. -->
<!-- The corollary of not being able to run FEM is that the model can produce physically inadequate parts and still succeed. But I can't do much about it yet. -->

### Definition of "success" in the simulation

We want to support one primary use-case: moving an object from one position to another, using motors and gravity; avoiding forbidden zones.

<!-- another use-case could be: given a severe constraint in positioning, design a system which would support a given load. However, the issue is that it's not  -->

#### Moving an object from one screen to another

We define the objective from four components:

1. "a build zone" - where the agent can actually create parts (note: the agent is forbidden to construct outside of this zone), "a goal zone" - the area to which the object needs to move to, a "forbid" zone - an area which the agent may not go into.
The objectives are always axis-aligned bounding boxes (AABB) for simplicity. The forbid or goal zone is triggered if the agent touches it even slightly.

#### Randomization

<!-- LLM-generated from my other spec. -->
The benchmarks are randomized to enable a wider data distribution with less generation effort.

- The benchmark volume size can vary 2x in all sides, and will be rescaled to random values, e.g. 1.68\*0.8\*1.3; the benchmark generator agent can narrow the scaling down if somehing is expected to break; however it is undesirable as we want to keep randomization higher.
  - The environment - ojectives (goals, forbids, build zones) are rescaled respectively.
- Goal, and obstacle positions are randomized by up to 40% of their size inwards (meaning they are becoming smaller and repositioned anywhere in the region where they are becoming smaller; smaller by their own size. They will always stay within original (maximum) bounds for safety).

## Observability

To track all agent movements and to persist data, we encode the following:

1. The agent pass/fail reasons
2. Error messages from script execution, linting,
3. All agent thoughts.
4. A mechanism to reconstruct those - e.g. we record the entire conversation and tool-calling structure, so how can we read it? How can we show it to users that use this prompt? How can we use it for debugging? basically, some order matters. Or, maybe just dump the conversation in/out to schema, that could also work.
5. Renders (visuals) of what the agent sees (optionally; if it doesn't take too much space; it may very well be more economical to reconstruct at runtime using a scripts. So record what comes into the agent, all parameters, code, setup, etc, and rebuild at runtime.)

These will be later used for querying, preproc and model training.

### Backups

In prod we will backup the schema daily in s3.
Notably, the file could be quite big, as we persist sqlite text. Max compression it before backing up.

One way to do it is by sending a `cron` job daily. Thus, implement an endpoint which will accept a cron call, and will back up the SQLite folder to the s3. Again, this is in production.

## Strict schema

We will run `schemathesis` checks against the OpenAPI. Strictly type all schema to avoid ANY issues.

### Schema autogeneration

We autogenerate python schemas, keeping in sync to the workers. We keep schemas defined in the main app, the worker inherits (for now). We have git hooks that implement the model.

## "Workbenches" - manufacturability verification

The agents will have *Workbenches* - a set of tools they can use to:

1. Verify their manufacturability.
2. Calculate costs of their parts (and verify against the user-inputted goals)

## CAD and and design validation

As said, "agents will live inside of a filesystem". The agents will generate and execute design validations of files in the filesystem.

### Benchmark generator and engineer handover

The Engineer agent(s) have can access to meshes and a exact reconstruction of the environment as a starting point to their build123d scene, however they can not modify/move it from their build123d scene. In fact, we validate for the fact that the engineer wouldn't move it or changed it (validating for changing it via hashing).

### Set of tools and utils

I propose the following set of tools (their usage is below):

- `validate_and_price(component: Part|Compound) -> float|dict[str, float]`: validates a part by for manufacturability, then prices it if valid using its workbench's cost calculation interface, or returns an error with a description and a location
- `simulate(Compound) -> SimulationResult` - Submits a model for a simulation.
<!-- dev note: assert against submitting a BuildPart builders, or other types. -->
<!-- should it contain it's environment model or only the generated model?  -->
- `submit_for_review(Compound)` - submits the whole assembly for a review to `Reviewer` agent node, which can later approve it and submit return the final design to the user
<!-- Same: what's in the compound? -->

*Note:* terminology: I use "component" is a shorthand for part OR assembly.

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

### Technical details

Technical details of manufacturability constraints are discussed in spec 004 (not to be discussed here.)

The workbench validation (as well as other util infrastructure are read-only in the container.

### Supported workbenches

3D printing, CNC and injection molding are supported.

<!-- In the future, it's very interesting to support topology optimization, but that's a separte project. -->

### Off-the-shelf parts (COTS)

It is self-understanding that engineers will use off-the-shelf parts - motors, fasteners, gears, etc. The catalog is well-defined in spec 007, but the model should have an access to a CLI tool or a set of python scripts to find something in a codebase. Again, we use CodeAct.

I suggest using a subagent for this. Give a prompt of what's necessary and let the subagent execute a series of read-only SQL prompts over a catalog DB. The agent will return a series of catalogs to use.

Both planner agent and engineer can only prompt the searching agent for searching.

## Other notes

1. There is no need to reinvent the wheel here. The codebase is to use the best practices. I don't want "innovative" code that is hard to work with and demands 2x of my time.
2. "Fallbacks" lead to bad code. Early termination is preferred. When we are making 3-4 fallbacks which lead to more logic and outdated codebases, it leads to issues to everybody. Need to refactor something? confirm it and stay lean. Fail fast if the application fails, because the "happy path" isn't met.

<!-- ### Production workflow

Norably, the production workflow is not an important part *right now* (February 4). We should prioritize the development workflows. -->
<!-- Production workflow became a priority -->
