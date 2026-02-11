# Desired architecture (human written)

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

## Agents

We have two agents (or agent graphs) - the benchmark generator and the engineer solving these issues.

### Benchmark generator agent (or graph)

#### Agent purpose

The agent is to generate problems for an engineer to solve. This is important, as to train or fine-tune a model with reinforcement learning or prompt optimization we need a challenges to scale, data to improve against.

#### Agent subagents

1. Planner - compose a description of how the benchmark behaves, what the learning goal is, and design such a challenge (such a puzzle) that would teach the agent something; e.g., how gravity works, friction, dynamic objects, motors, etc.
2. A CAD engineer/coder that implements the benchmark from the plan
3. A reviewer that reviews the environment for
    - Feasibility of solution
    - Lack of violation of environment constraints (no significant, etc.)
    - Proper randomization.
    - No excessive degrees of freedom; all parts are fixed as they should be.

#### Output requirements

The benchmark generator writes code for the and gets internal reviews by the reviewer inside of the generator.

The benchmarks are randomized to ensure data distribution.

The benchmarks are consisting of CAD models which are converted into XML.

- The execution runs in isolated containers to prevent accidental harmful code execution in the main system.
- The benchmarks are verified for being creatable in MuJoCo. They are not solved by an engineer yet, just benchmarks are created and verified for validity.
  - Validity means no intersections and other problems; It also means that the code will compile in the first place.
  - MJCF is verified for the correctness by XML schema. And also by running a few frames of a simulation
- MJCF is created programmatically, not by a LLM.
<!-- I will need to experiment, but I don't think the LLM should be able to edit it.  ->

#### Benchmarks

The environments are made with static objects, dynamic objects, and motors. <!-- note - not sure if I handle dynamic objects atm. If I do, how are they specified in CAD? -->

Motors require power to run, however, we don't bother with "wiring" yet.

Problems with motors and moving parts are verified more consistently because they are more prone to error.

<!-- note: it would be useful to persist an "expected solution" from the planner during task generator. It'll help guide exploration and maybe improve LLM optimization (prompt reflection/RL) with more data. -->

#### Subagents output requirements

#### Sample output

(input: Test benchmark: drop a ball into a funnel)

```markdown
1. **Learning Objective**: This benchmark tests the agent's ability to perform **spatial redirection** and **slope calculation**. The agent must design a geometry that intercepts a dynamic object (falling ball) and uses gravity/inclination to guide it to a target while avoiding a forbidden area directly beneath the release point; and also avoiding the bounds of the simulation.

2. **Static Geometry**:
    - **Support Wall**: A vertical plate at the back (Y=-20) to provide a mounting surface for the agent's part.
    - **Goal Bin**: A small open-topped box or designated area where the ball must land.

3. **Input object**:
    - **The Projectile**: A high-density sphere (ball) spawned at a high Z-coordinate. It is subject to gravity and will fall immediately upon simulation start.

3. **Objective entities**:
    - **Forbid zones**:
        - A block located directly under the ball's release point to penalize a simple vertical drop.
    - **Goal zone**:
        - An objective located inside the funnel.

4. **Success Criteria**: The ball's center of mass must enter the `zone_goal` (an axis-aligned bounding box defining the interior of the Goal Bin).

5. **Rescale Limits**: [0.8, 1.2] for X, Y, and Z to maintain the functional relationship between the drop point and the target.

6. **Randomization**:
    - `ball_start_x`: Small horizontal offset to the release point.
    - `goal_offset_x`: The distance the ball needs to be redirected horizontally.
    - `obstacle_height`: The height of the forbidden zone, forcing the agent to adjust the steepness of its ramp.

7. **Python Script Structure**:
    - Define the environment (Wall, Obstacle, Goal Zone).
    - Define the dynamic ball as part of the environment (but with mass/physics).
    - Define the agent's part (the ramp/bridge).
    - Use `to_mjcf` to export the scene.
```

### Engineer (problem solver)

#### Purpose

There are real-life engineering problems that LLM AI agents can help overcome. This agent is an engineer that should be capable of solving problems, given their visuals/CAD designs, and produce solutions as to how to solve a given problem; constrained on physics. The agent should operate in the most closely real-life environment. The agent shouldn't even understand it works in a "fake" environment.

#### Description

Writes CAD code to solve the problems (the benchmarks as above).

Is constrained by cost and manufacturability. Also constraints such as weight and where the agent can actually build their solution.

Has access to all realistic CAD tools.

The agent can preview their tools and check for whether the designs are manufacturable without being "punished".

It is OK if agents are running for 10 or more minutes (in production).

Again, the execution runs in isolated containers to prevent accidental harmful code execution in the main system.

#### Engineer agent details

Engineer has a Planner, which is responsible for architecting the solution, the engineer which actually implements the solution, and the critic.

The architect will create and persist a TODO list. The engineer must implement. The agent will have easy access to the TODO list.

- The engineer, after *proving* the TODO list is impossible, can refuse the plan.

<!-- Standard best practices will be used. No point in making anything weird in this application. -->

##### Planner workflow

The Engineering Planner workflow is:

1. **Intake and mandatory context read**
   - Read `objectives.yaml` as present from the benchmark generator (goal/forbid/build zones, moving parts + DOFs, runtime jitter, benchmark-level `max_unit_cost`/`max_weight`).
   - Read benchmark visuals (`renders/images`, 24-view context) and environment geometry metadata.
   - Read required skills/config inputs (CAD drafting skill, manufacturing knowledge when cost/quantity matters, manufacturing config + catalog).

2. **Plan the mechanism and budgets**
   - Propose a physically feasible mechanism that fits build-zone constraints and runtime jitter; and fit
   - Set planner-owned `max_unit_cost` and `max_weight` **under** benchmark/customer caps.
   - Select candidate COTS parts (motors/fasteners/bearings/gears) via the COTS search subagent and carry part IDs + catalog prices into the plan.

3. **Calculate the costs per part**:

  We want to estimate a rough, but detailed prices and architecture of the solution.

  Create a file like `preliminary_cost_estimation.yaml` containing:

  For each part:
      1. A name of the part
      2. a description of the part,
      3. costing information each part in the simulation, e.g. for CNC, blank size, and final volume of the part (and calculate how much to be removed). The inputs are auto-validated, per manufacturing method

  Then: create an assembly structure, like:

  ```yaml
  final_assembly:
    - subassembly_1: 
        parts:
          - part_1 
          - part_2
          - part_3
        joints: 
        - joint_1: 
            parts:
              - part_1
              - part_2
            type: fastener_joint 
    - subassembly_2:
        parts:
        - part_4
        - part_1 # part 1 is inserted twice. Hence it'll be estimated as necessary to manufacture it twice, hence unit costs will drop (as per manufacturing method config)
        joints:
          parts:  #note: maybe we want to inline it.
            - part_4
            - part_1
          type: fastener_joint
    - part_5 
  ```

  Then: Run a script like `validate_and_price.py` that would automatically validate the YAML file for consistency and output pricing. The model can thus use a stricter constraint.

  Notably, if the plan is higher than the max_unit_cost, it can't proceed and needs to adapt the plan.

1. **Write required planner artifacts**

- Create `plan.md` using the strict engineering structure:
     `## 1. Solution Overview`, `## 2. Parts List`, `## 3. Assembly Strategy`, `## 4. Cost & Weight Budget`, `## 5. Risk Assessment`.
- In `plan.md`, include manufacturing method/material choices, assembly strategy (including rigid-connection fastener strategy), and risk mitigations.
- Create `todo.md` as an implementation checklist for the CAD engineer (initially `- [ ]` items).
- Create `preliminary_cost_estimation.yaml` with per-part costing fields (method-specific) and a `final_assembly` structure for reuse/quantity accounting.

At this point, the planner can handoff the documents to the CAD engineering agent. Before handoff, the planner runs a standalone script from `skills/manufacturing-knowledge/scripts/validate_and_price.py` to validate `preliminary_cost_estimation.yaml` and compute preliminary totals (including geometry-driven fields such as part volume, blank/stock size, stock volume, and removed volume for CNC). If the estimated cost is above `max_unit_cost`, the planner cannot proceed and must adapt the plan. The planner's documents are autovalidated; if validation fails, handoff (submission) is refused until fixed. (the validation is currently implemented as Pydantic validation.)
<!-- 
4. **Pre-handover validation gate**
   - Ensure markdown/YAML structure is valid (plan sections + list/table requirements, TODO checkbox format).
   - Verify constraints/logic consistency: units, build-zone fit, cost/weight bounds, and no invented catalog pricing.
   - Planner submission is treated as invalid if required files are missing or malformed.

5. **Handover and iteration loop**
   - Handover `plan.md` + planner-constrained objectives + `todo.md` to the CAD engineer.
   - CAD engineer implements and may request refusal only with proof that planner constraints/approach are infeasible.
   - Reviewer either confirms refusal (`confirm_plan_refusal`) and routes back to Planner for re-plan, or rejects refusal (`reject_plan_refusal`) and routes back to CAD implementation.

6. **Observability**
   - Emit structured events for plan submission, COTS search usage, markdown/YAML failures, logic/constraint failures, and plan-refusal decisions for downstream evaluation. -->

#### Verification

The Engineer agent will verify its work by:

1. Checking the manufacturability of its solution, on
    1. Individual part scale (this part can be machined)
    2. Assembly scale - this assembly has no interference of parts and has valid part constraints.
2. Checking the cost of its solution; against the part count and unit cost as specified by the user.
3. Checking the weight of its solution.
4. Simulating - did the model achieve the goal as per the benchmark?
5. The critic will assess whether the simulation is stable or not - will it be functional to work in a real scenario? Is it flaky?

### COTS search subagent

An engineering agent(s) can invoke a COTS (Commercial-Off-The-Shelf) search agent to delegate the search for off-the-shelf components. E.g.: search for motors.

Purpose: a lightweight subagent that performs catalog lookups and returns verified part candidates.

#### Model and runtime

- Uses a smaller/cheaper model than the primary planner/engineer.
- Read-only access to the COTS catalog DB and/or CLI; no writes.
- No file edits except optional `journal.md` logging of queries + results.

#### Inputs (from planner/engineer/benchmark planner/benchmark CAD engineer)

- Part intent (e.g., "M3 fasteners", "servo motor 3-5 kg*cm", "bearing 608").
- Constraints: quantity tier, max_unit_cost, size/torque/voltage limits, material, mounting/shaft constraints.

#### Tools

- Read-only SQL queries against the COTS catalog database.
- Read-only CLI helpers/scripts (if provided) for catalog search.

Outputs (structured, concise):

- 3-10 candidate parts with `part_id`, `manufacturer`, `key specs`, `unit_cost`, `quantity_tier`, `source`, `why it fits`.
- If no match, explicitly say "no match" and list which constraints were too strict.

#### Invocation

- Engineering Planner, implementer, reviewers and benchmark Planner, implementer, reviewers calls the subagent whenever a COTS part is needed (motors, bearings, fasteners, gears, etc.).
- The returned part IDs and prices must be used in the plan and cost estimates.

Notably the benchmark planner will need it too since they are also responsible for (hard cap) price estimation

##### Reasons for invocation

1. Planners in benchmark and engineering invoke to check exact prices for subcomponents (both make price decisions)
2. Engineers invoke them because they need to use them and check prices for components (they are constrained by the price)
3. Reviewers may search for a better component *(suggestion: reviewers may want to read the search queries of invoking agents to decide if the part found was sufficiently good or not. If the query is good enough - can just skip!)*

#### COTS catalog database (spec 006)

This system is backed by a SQL catalog built from `bd_warehouse`. The catalog is **read-only in workers** and queried only via the COTS search subagent.

Database:

- Artifact: `parts.db` (SQLite; local on worker, read-only).
- Build-time indexer extracts metadata and usage snippets from `bd_warehouse`.
- Store reproducibility metadata: `catalog_version`, `bd_warehouse_commit`, `generated_at`.

<!-- TODO move details away from this section into CAD section... -->

### Agentic framework

We use LangChain and LangGraph for the agentic infrastructure. The `deepagents` framework from LangChain developers helps stitching them together (with filesystem utils, TODO lists, etc.)

### Filesystem

Both of the agents "live" directly in the filesystem of the container that they have been assigned to and thus runs their workflow. This serves the purpose of reducing complexity in tooling, and giving the agents the familiarity with editing tools. There are skills, a script to be written, and verification tools in the script.

#### Templates

Each agent starts with a template, roughly defined in [Starting folder structure for various agents](#starting-folder-structure-for-various-agents).  It is predefined for each agent and we will test it.

##### Initial files for each agent

- Engineer Planner: `skills/`, `utils/`, `plan.md`, `todo.md`, `journal.md`, `objectives.yaml` (read), `preliminary_cost_estimation.yaml`
- Engineer CAD: `skills/`, `utils/`, `plan.md` (read), `todo.md`, `objectives.yaml` (read), `preliminary_cost_estimation.yaml` (read), `script.py`, `journal.md`, `renders/`
- Engineer Reviewer: read-only all agent files, plus write access to `reviews/` (uses `renders/`, `plan.md`, `objectives.yaml`, `preliminary_cost_estimation.yaml`)
- Benchmark Planner: `skills/`, `utils/`, `plan.md`, `todo.md`, `objectives.yaml` (draft), `journal.md`
- Benchmark CAD: `skills/`, `utils/`, `plan.md` (read), `todo.md`, `objectives.yaml`, `script.py`, `journal.md`, `renders/`
- Benchmark Reviewer: read-only all agent files, plus write access to `reviews/` (uses `renders/`, `plan.md`, `objectives.yaml`)
- COTS Search: read-only COTS catalog DB/CLI, `journal.md` (queries + results)
- Skill Creator: `skill-creator/SKILL.md` (read), `skills/` (read/write), `journal.md`, git metadata

Notably, I don't think that creating them as "templates" (outside of symlinks) is necessary as they are programmatically assembled. That said, if they are programmatically assembled, it should be tested; could be a centralized schema creation. Note that `skills/` are pulled from git repo (as specified in other parts of the doc).

Another important note: files in e.g. Engineer CAD agent or reviewer aren't created anew - they are reused from the previous agent.

<!-- Note: the filesystem is not in repo root, but in docker containers. -->

<!-- Note: each of these should be asserted.-->

##### Template auto-validation

Where possible, templates would have a validation schema. E.g. (in particular) for YAML files, we would define a validation schema

#### File updates

The file writes don't persist in the Controller except into the observability database. Their edits are sent directly into the container mounted (ephemeral) storage.

To update the files, or execute other functions: the controller parses the tool call locally; then sends the request over to the worker to execute whatever is related to the tool - read file, write file, or edit file (perhaps, `deepagents` handles the network part too? I think it does, via the Filesystem...).

The network latency is perfectly acceptable as LLM reasoning latency far outweighs the ping time.

#### Utils files

The agent has a set of utils - python scripts (files) that the agent can import from. These are explicitly unwritable by the `deepagents` `FilesystemMiddleware` (which can force certain files to be unwritable by the in agents) the filesystem.

#### Skills files

Unlike utils files which are static (or the container restart is acceptable), skills files will be updated by a sidecar learner agent rather frequently (possibly after every 10-20 conversations?).

The container will download the pull skills from a skill git repo before every run (justified? else how do we handle the updates? The file size there is minimal.)

Maybe use a dedicated public git repo? I think it's sound. However this is complexity, and probably isn't that sound. But it'll enable human readability, which is a bonus.

*Note: the skills are to be versioned either way.*

*Note on observability: for observability and reproducibility, the skills usage is to be tracked in the database.*

##### Managing skills in git repo

Yes, skills are versioned via a public git repo. Because of filesystem edits and more complex logic, I propose to send git commits and pushes directly from workers.

Default logic: git commit & git push. If push fails due to merge conflict, do git merge. If merge fails, have the skill creator LLM handle it and push it. I suggest using `GitPython` or `git2` (note: we stuck with git2 for the moment) as a library since it's more robust than using shell.

*note*: Ideally, we'd do this logic in controller since controller is the place where we manage all secrets, but... it's not that scary, and we trust our compute nodes, for now.

#### Automatic validation of files

Many files - TODO lists, plans, would be automatically verified, with descriptive, markdown-like errors submitted to the agent. (e.g. in plan files, all sections must be submitted).

##### Validating markdown files

Markdown is validated statically to ensure a structure and exact match of headings, sometimes content - e.g. bullet points, exact headings or others is met.

In TODO lists, we assert that either [x] or [-] is present in all checkboxes; and no checkboxes were deleted. It *is* pretty inflexible, but I guess that's what we'll do for now. This should keep the model on task longer.

##### Validating python files

Files are linted and don't pass execution/submission if they have red errors. Refer to "linting" section.

#### Starting folder structure for various agents

We define the file structure as follows, individual agents adapt to individual needs:

```text
.
├── skills/                     # [Read-Only] Learned skills and documentation
├── utils/                      # [Read-Only] Fixed utilities and shared code
│   └── ...
├── renders/                      # [Read-Only/tool-generated] (Renders of images and  from the environment) 
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
##### Benchmark generator

###### Benchmark Generator - planner

1. Planning skills
2. A markdown plan template detailing learning objective and, in particular, **geometry** containing (auto-validated, refuses submission if doesn't match template as above)
3. Sample objectives.yaml (validated)

###### Benchmark Generator - CAD agent

1. Build123d and implementation skills (read-only)
2. A template for creating benchmark generator files (read-write) (e.g. `output.py`)
3. A TODO list (read-write, validated automatically via a on-edit hook (watchdog, or maybe an implicit trigger via ))
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

##### Planner - benchmark generator

### Agent artifacts

The important part of managing agents are their artifacts.

#### Prompts and environment

An agents must know what it is capable to work with. Have a "capabilies.md" document that shows

<!-- Issue found: I tried giving the planner a standard prompt - to move a ball of a diameter 500mm sideways. It created a plan which used Neosprene drive belts, conveyors, aluminum frames - none of which we have. -->
<!-- I didn't have a good LLM to write the propmt. Currently relied on "tricking" the model with a "manufacturing-capability-like" document that shows what it can use.-->

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
Notably, to keep the agents accountable and enforce stricter and more consistent typing, agents would write a yaml frontmatter on reviews - with `accepted`, `rejected`, `confirm_plan_refusal`, `reject_plan_refusal` for `decision`.

If the CAD agent refuses the plan, either of `confirm_plan_refusal`, `reject_plan_refusal` can be selected, but only in this case (there must be validation for these fields).

### Token compression

As the agent will near its token generation limits, they will compress their old memory by a summarizing agent.

### Skills

The agents will be able to create and update their skills to improve the future runs. This is vital for continuous improvement, and will be used alongside prompt improvement. *Skills* are actually the information that the agent doesn't know and needs to learn - for example how to use `build123d`.

- If the agent fails too much, after resolving a solution, they will persist it to the skill.

We use the `SKILL.md` format as by Anthropic. I refer to ".agent/skill-creator/SKILL.md` for more information about skills.

We know that agents can not use `build123d` very well despite it being one of the central part of their work in this repo. This is part of a mitigation.

#### `build123d` skill and access to documentation

The agents will have an access to build123d documentation through the skill (as per the SKILL.md standard, in `/references/ folder.)

#### Different agents have different skills

The "Engineer" agent does not need a "benchmark creation skill". It could be necessary the other way around. Nevertheless, agents access to skills should be configurable by default. If an agent has created a new skill, let it stay just to that agent.

#### Explicit skill agent

The agents can not be trusted with updating the skill well, and they may be out of context. Quite likely, implement a separate "learner" agent node that runs after success, probably async of the agent.

It will work over smaller chunks of uncompressed info (how exactly? Maybe progressive disclosure? I think implement a progressive disclosure from a journal)

The skill agent will read a `skill-creator/` skill as from Anthropic.

##### Skill agent is run async

The skill agent is run asynchronous to the execution, modifying the skill folder and pushing it to a git repo. It's filesystem is managed by `deepagents` (as an exception), is stored on (the same) Railway bucket under the folder (because it's easier from the deployment perspective).
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

It was proven to be more cost-effective to use a "sidecar" agent that runs in parallel to the primary model. We will use an inexpensive model, such as DeepSeek 3.2 or small models.

<!-- Note: it was found that YAML/Markdown are the most effective for storing model outputs structured information output. YAML, later converted to markdown programmatically is likely preferable? -->

###### When to write in a journal?

As above - when the agent has spent >4 tool calls in a single conversation trying to figure out an issue or syntaxis, or
This also applies to refactors. If the agent has taken an approach, spent 5 tool calls doing a roundtrip and decided against it in the end, note the architectural issue.
In both cases, if the agent found the solution

The agent will go through all notes in the session and read through ones that are relevant. They will then link the found and solved issues.
<!-- Note: maybe add a "hypothesis" - an agent would be perform better if we X? -->

###### Example

1. Observation: Multiple agents have an issue with struggling to group parts together into a `Compound` - they had more than four unsuccessful tool calls trying to do it(struggle) .
2. The learner agent records the issue (async)
3. The main model finally finds that `Compound` syntaxis requires `Compound(children=[Part|Compound, ...])` syntaxis (with `children` upfront)
4. The learner agent records the found solution and the skill.

#### Worker skills are persisted and are separate from the repository-level skills

Skills in the `.agent/skills/` in the repo root are different from the agent skills we are learning in the database! The repo root skills are for the coding agent to write this codebase. The learned skills should be, e.g. in workspace/ folder.

#### Skill safety toggle

The skill writing agent can not delete or overwrite more than 15 lines of code per session (adding is unbound). This is to prohibit a skill being overwritten by any agent completely. Using git diff, if there are more than 15 lines detected, the agent is notified. If the session times out (too many tool calls/the agent finishes despite warnings), we revert the commits.

#### Skill versioning

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

#### All handovers that happen

User prompt ->
benchmark planner agent <-> Benchmark CAD agent <-> benchmark reviewer (If plan is not valid - e.g. it specifies a conflicting geometry, the CAD agent will refuse; and send back to benchmark planner agent. However, the benchmark CAD agent can not refuse because it fails to do CAD, it can only refuse if the model is in fact invalid.)
(Benchmark reviewer to CAD agent - if the environment CAD 3d model does not adhere to the plan OR the environment CAD model has invalid geometry e.g. intersections OR it is impossible to solve , the benchmark reviewer agent can refuse)

Benchmark reviewer "accepts" and passes the environment to the "lead engineer" - the Engineering Planner model. (indirect contact - no actual "communication")

Lead engineer <-> CAD modelling engineer <-> Engineering Reviewer

The lead engineer will try to think of the cheapest and most efficient, but *stable* approach of solving the environment given known constraints (objectives info, cost/weight info, geometry info, build zones).
CAD modelling engineer can refuse the plan if the plan was inappropriate, e.g. set too low price or the approach was inappropriate.
In this case, the Engineering Reviewer must agree and confirm that in fact, the price or weight was set too low, and will pass the plan back to the lead engineer for replanning.

The "reviews" are made more deterministic by passing YAML frontmatter to markdown review documents (which are later parsed deterministically). The reviews and plans must be appropriate.

#### Benchmark generator Planner and Benchmark CAD designer

The Benchmark Generator Planner will submit multiple files to the CAD implementing agent.

The plan will have the following bullet points. The plan will be validated for consistency, and will not be accepted until the markdown passes strict formatting criteria, and will ensure that the bullet points are there:

1. `plan.md`:
    - Learning objective (summary of what the agents needs to or will learn as a result of this challenge);
    - The geometry of the environment:
        - coordinates of all major shapes in the environment + randomization.
        - Geometry and coordinates of all moving parts:
            - motors
            - non-fixed parts.
    - Input objective:
        - Shape of the input (can be anything; the ball is the easiest, any more complex shape is more difficult to solve (however it's more interesting too))
            - how shape is randomized (shape of the input should be held more or less constant throughout of the run). This is a bullet point list.
        - Central position and position randomization margins (e.g. x:50, y:80,z:20, x_variation: 20, y_variation: 20, z_variation: 10)
    - Where the input objective is located (coordinates + randomization),
    - Objectives location:
        - A "forbid" objectives as a set of approximate AABB coordinates,
        - A "goal" objective as a single AABB coordinate
The agents' file must correspond to roughly the structure detailed above, with automatic checks in place.
2. A `todo.md` TODO list from the planner.
3. A draft of `objectives.yaml` with rough values filled in.

The agent must make sure that the geometric plan is valid, the input objective does not interfere with anything (and goal objectives are not obstruted), that there is proper randomization, etc., no object coincides with each other.

#### Benchmark Generator with Engineer handover

The Engineer agent(s) (for whom the first point of access is the Planner/lead engineer) have can access to meshes and a exact reconstruction of the environment as a starting point to their build123d scene, however they can not modify/move it from their build123d scene. In fact, we validate for the fact that the engineer wouldn't move it or changed it (validating for changing it via hashing) - in both MJCF and build123d.

Additionally, the engineering agent will be supplied with renders for preview automatically rendered from 24 views. (Clockwise, 8 pictures, on 30 degrees up or down (configurable)).

The engineer will also receive a YAML file with:
    1. Exact positions (boundaries) of objectives.
    2. All moving parts (it's impossible to guess from pictures what is moving and in which direction from pictures only), with their DOFs that are programmatically calculated, with a small text description of how they can move.
        - Including motors.
    3. "Runtime" randomization, i.e. the randomization of the starting positions this environment will use. Note that it's different from the "static" randomization which is stronger.
    4. Maximum prices and weight. Prices should be estimated by the planner to be relatively challenging but feasible by the planner (feasible but challenging related to *our* pricing sheet, as defined in planner.md). I suggest initially giving 50% safety margin in pricing and weight.
    Note that the maximum price and weight are also set by the planner later internally. However, the planner sets their own constraints *under* the maximum price. Here the "maximum prices and weight" are a "customer-specified price and weight" (the "customer" being the benchmark generator), and the planner price and weight are their own price and weight.
    <!-- (in future work) Later on, we will challenge the agent to optimize it's previous result. It would have to beat it's own solution, by, say, 15%.  -->

The positions of objectives (including a build zone) in a `objectives.yaml` file, and information on randomization of starting object positions (small "runtime" jitter), so as to prevent guessing in a 3d space.

##### A benchmark is reused multiple times

Notably, the benchmarks are randomized and reused multiple times under different variations. the Engineering agent only receieves a "runtime" randomization - (as in the list of the relevant heading) currently only a (relatively) small jitter in the environment.

##### What if the benchmark agent set the price too low?

For now, nothing. I'll filter it out via SQL or similar later and make a "price discussion" between agents - probably. Or human-in-the-loop.

#### Engineer Planner and "Coder" interaction

Engineer sends four files to the coder agent who has to implement the plan:

1. A `plan.md` file The plan.md is a structured document (much like the benchmark generator plan) outlining:
2. A stripped down `objectives.yaml` file, except the max price, weight are set by the planner now - and they are under the max weight set by the user/benchmark generator.
3. A `todo.md` TODO-list.
4. A `preliminary_cost_estimation.yaml` file with per-part pricing inputs, `final_assembly` structure, and preliminary totals produced by `validate_and_price.py`.

##### `plan.md` structure for the engineering plan

```markdown
# Engineering Plan
## 1. Solution Overview
A brief description of the approach to solve the benchmark objective (e.g., "A gravity-fed ramp with a deflector plate to redirect the ball into the goal bin").
## 2. Parts List
For each part:
- **Part name**: e.g., `ramp_main`
- **Function**: What role it plays
- **Geometry**: An overview of geometry
    - **Mating points** with other connectors (list)
- **Manufacturing method**: CNC / Injection Molding / 3D Print
- **Material**: e.g., `aluminum-6061`, `abs-plastic`
- **Estimated dimensions**: Rough sizing
## 3. Assembly Strategy
- How parts connect (fasteners, etc.) <!--e.g. bearings in the future -->
- Mounting points to environment (if any drilling/attachment is allowed)
<!-- - Order of assembly --> 
<!-- Order of assembly is partially unnecessary because we kind of work in CAD. However, it's a good thing to think of. -->
## 4. Cost & Weight Budget
- `max_unit_cost`: $X (from objectives.yaml, planner's allocation)
- `max_weight`: Y kg
- Preliminary breakdown per part
## 5. Risk Assessment
- Potential failure modes (e.g., "ball bounces off ramp edge")
- Mitigations for each risk
- Runtime randomization considerations (position jitter handling)
<!-- ## 6. Build123d Strategy
- CSG vs sketch-based approach
- Key operations (fillets, chamfers, patterns)
- Reference skills to consult -->
<!-- Note: the planner explicitly doesn't specify the CAD approach. It doesn't need to think about plans, it's about the geometry. -->
```

##### `objectives.yaml`

`objectives.yaml` is a central data exchange object to the system. To centralize it's structure:

```yaml
# =============================================================================
# OBJECTIVES.YAML - Your Task Definition
# =============================================================================
# This file defines WHAT you must achieve. Read it carefully before planning.
#
# YOUR MISSION: Guide the `moved_object` into the `goal_zone` while:
#   1. Staying WITHIN the `build_zone` (you cannot build outside it)
#   2. AVOIDING all `forbid_zones` (contact = failure)
#   3. Respecting `max_unit_cost` and `max_weight` constraints
#
# The environment and moving_parts are READ-ONLY. You design parts that
# interact with them, not modify them.
# =============================================================================

objectives:
  # SUCCESS: The moved_object's center enters this volume
  goal_zone:
    min: [x_min, y_min, z_min]
    max: [x_max, y_max, z_max]

  # FAILURE: Any contact with these volumes fails the simulation
  forbid_zones:
    - name: "obstacle_collision_zone"
      min: [x1, y1, z1]
      max: [x2, y2, z2]
    # Additional forbid zones may be listed here

  # CONSTRAINT: Your entire design MUST fit within these bounds
  # Parts placed outside will fail validation
  build_zone:
    min: [x, y, z]
    max: [x, y, z]

# Hard simulation boundaries - objects leaving this volume = failure
simulation_bounds:
  min: [-50, -50, 0]
  max: [50, 50, 100]

# -----------------------------------------------------------------------------
# THE OBJECT YOU MUST DELIVER
# -----------------------------------------------------------------------------
# This object spawns at `start_position` (with runtime jitter applied).
# Your design must reliably guide it to the goal_zone.
moved_object:
  label: "projectile_ball"
  shape: "sphere"
  # Static randomization: shape varies between benchmark runs
  static_randomization:
    radius: [5, 10]  # [min, max] - actual value chosen per benchmark variant
  start_position: [x, y, z]
  # Runtime jitter: small position variation per simulation run
  # Your solution must handle ALL positions within this range
  runtime_jitter: [2, 2, 1]  # [±x, ±y, ±z] mm

# -----------------------------------------------------------------------------
# ENVIRONMENT MOVING PARTS (READ-ONLY)
# -----------------------------------------------------------------------------
# These exist in the environment. You CANNOT modify them, but you CAN
# design parts that interact with them (e.g., attach to motor shafts,
# use passive sliders as triggers).
moving_parts:
  - name: "feeder_motor"
    type: "motor"
    position: [x, y, z]
    dof: "rotate_z"  # Degrees of freedom: rotate_x/y/z, slide_x/y/z
    control:
      mode: "sinusoidal"  # Options: constant, sinusoidal, on_off
      speed: 1.0          # rad/s (for rotate) or units/s (for slide)
      frequency: 0.5      # Hz - for sinusoidal mode
    description: "Rotates clockwise to push objects"

  - name: "passive_slider"
    type: "passive"  # Moves only when external force applied
    position: [x, y, z]
    dof: "slide_y"
    description: "Slides freely along Y when impacted"

# -----------------------------------------------------------------------------
# YOUR CONSTRAINTS
# -----------------------------------------------------------------------------
# These are challenging but achievable. Exceeding them = rejection.
constraints:
  max_unit_cost: 50.00  # USD - total cost of your manufactured parts
  max_weight: 1.2       # kg - total weight of your design

# Randomization metadata (for reproducibility)
randomization:
  static_variation_id: "v1.2"  # Which static variant this is
  runtime_jitter_enabled: true
```

<!-- Note: we are using metric units and degrees. -->

##### `preliminary_cost_estimation.yaml`

To reduce cost guessing, the Engineering Planner outputs a machine-readable estimate file that captures all pricing inputs per part and the assembly structure used to derive quantities and part reuse.

Expected flow:

1. Planner drafts entries for all planned manufactured parts and COTS components.
2. Planner defines `final_assembly` (subassemblies, part membership, and joints); under the hood we:
    - Calculate as much as possible to prevent the planner from needing to think (e.g.: cooling time in injection molding is autocalculated from wall thickness, 3d print time is autocalculated from volume, setup time is autocalculated etc.)
    - Estimate part reuse - if the part/subassembly is reused, unit costs go down as per manufacturing rules (making 2 equal parts is cheaper than making 1 due to economics of scale).
3. Planner runs `skills/manufacturing-knowledge/scripts/validate_and_price.py`.
    - The script validates schema consistency and computes preliminary totals.
    - The script auto-populates the unit cost and weight from the objectives.yaml file (unless the file is corrupted).
4. If totals exceed `max_unit_cost` (or other numeric constraints), planner must re-plan before handoff.
5. Planner writes planner-owned constraints in `objectives.yaml` using validated preliminary totals, under benchmark/customer caps.

Minimum per-manufactured-part fields:

- `part_name`, `part_id`, `description`, `manufacturing_method`, `material_id`
- `part_volume_mm3`
- method-specific costing inputs:
  - CNC: `stock_bbox_mm`, `stock_volume_mm3`, `removed_volume_mm3`
  - Injection molding / 3D print: required process-specific volume/thickness/time fields per validator contract
<!-- - `estimated_unit_cost_usd` - auto-calculated. The planner does not need to calculate each. However, it may be populated automatically by the script if it runs successfully? Again, the goal is to offload agent work and guessing (for performance reasons). -->
- `pricing_notes <!-- User review - maybe. Maybe for "confidence" scores or similar.>

Minimum per-COTS-part fields:

- `part_id`, `manufacturer`, `unit_cost_usd`, `source`

Required assembly fields:

- `final_assembly` containing subassemblies/parts/joints
- repeated part references are allowed and used by pricing logic to compute quantity effects

```yaml
version: "1.0"
units: #pre-populated in template.
  length: "mm"
  volume: "mm3"
  mass: "g"
  currency: "USD"
# constraints: # user review - no, unit constraints are written in objectives.yaml. 
#   benchmark_max_unit_cost_usd: 50.0
#   benchmark_max_weight_kg: 1.2
#   planner_target_max_unit_cost_usd: 34.0
#   planner_target_max_weight_kg: 0.9
manufactured_parts:
  - part_name: "ramp_main"
    part_id: "ramp_main_v1"
    description: "Primary sloped ramp for ball redirection"
    manufacturing_method: "CNC"
    material_id: "aluminum-6061"
    part_volume_mm3: 182340.0
    stock_bbox_mm: {x: 220.0, y: 120.0, z: 12.0}
    stock_volume_mm3: 316800.0
    # removed_volume_mm3: 134460.0 # no need to calculate, have script calculate it.
    estimated_unit_cost_usd: 18.70 # estimated automatically
    pricing_notes: "3-axis; no undercuts"
  - part_name: "guide_clip"
    part_id: "guide_clip_v1"
    description: "Guide clip holding edge trajectory"
    manufacturing_method: "INJECTION_MOLDING"
    material_id: "abs-plastic"
    part_volume_mm3: 24100.0
    wall_thickness_mm: 2.0
    cooling_time_s_estimate: 11.0
    estimated_unit_cost_usd: 0.82
    pricing_notes: "tooling amortized at target quantity"
cots_parts:
  - part_id: "M5x16-912-A2"
    manufacturer: "ISO"
    unit_cost_usd: 0.09 # auto-calculated.
    source: "parts.db"
  # user note: cots parts must be enforced to exist in the subassemblies, at least 1. Else why would it be here?
  # user note 2: reminder: search for COTS parts is performed by a subagent
final_assembly:
  - subassembly_id: "frame_and_ramp"
    parts: ["ramp_main_v1", "guide_clip_v1", "guide_clip_v1"]
    joints:
      - joint_id: "j1"
        parts: ["ramp_main_v1", "guide_clip_v1"]
        type: "fastener_joint"
totals:
  estimated_unit_cost_usd: 31.46
  estimated_weight_g: 742.0
  estimate_confidence: "medium"
```

Validation requirement:

- Submission is blocked if `preliminary_cost_estimation.yaml` is missing, malformed, still template-like, fails `validate_and_price.py`, or contains non-numeric values for required numeric fields (doesn't match schema in general)

#### Coder and Reviewer interaction

1. The reviewer will have access to all files of agents in read-only mode (note: questionable decision - why would they need code files?). Primarily, they will focus on reviewing the video and image files for a more realistic review (presumably directly from the Railway bucket, if filesystem allows it), and the `objectives.yaml` file (YAML files with objectives, as specified above). Thus the Reviewer will only have readonly on all agent files permissions.
The reviewer will also have `write` and `edit` tool with permissions of editing a single "reviews/review-round-[round number]" folder.

The goal is to persist the reviews into a persistent file which the agent can reference at any time (alongside previous reviews), and see it only once; and to avoid plumbing to route "reviews" text when required.

The reviewer will validate for the following:

1. The solution looks stable, i.e. #2
2. No excessive DOFs which will hijack the reliable work of the solution.
3. The solution is as optimal it should be.

Notably the Engineer will at this point already have passed the manufacturability constraint, cost constraint, and others.
<!-- and others - I need to ideate/remember what else it should review for.  -->

The reviews will also come with the following YAML frontmatter:

```yaml
decision: approved # [approved, rejected, confirm_plan_refusal, confirm_plan_refusal]
comments: [
   # short commentary on issues, as a array.
   # Up to 100 chars per issue.
]
```

As usual, the reviews will be strictly typed.

### Clarification - definition of constraints in planning

1. Constraints are set first at the application level; e.g. the timeout of simulation is always 30 seconds
2. Then the benchmark planner/implementer set a more realistic constraint for them (e.g., they set a max cost, max weight for the simulation, similarly to how a "customer" would do it for an engineering company)
3. The engineering planner can set an even lower constraint. to force the engineering implementer to think on how to achieve a certain goal cost-effectively. The implementer won't pass the cost metric until it is done.

### Agents cap on execution

The agents will have a max_turns=60 (for example) cap, which would control how many turns (tool call invocations/responses) the agent can do. The agent will stop on turn number max turns The user is able to have the agent continue for another number of turns.

## Agent Evaluations

Evaluations are treated as a first-class architecture in this application. In fact our work on manufacturability validation, code linting, simulation is just a tool for evaluation.

<!-- To build great agents, we need agent needs a great evaluation pipelines.
We need 10 at least test prompts for each agent (primary) agent subtype - 3 for benchmark (Planner, CAD_agent, Reviewer), 3 for engineering (Planner, CAD_agent, Reviewer).

We will also need evaluations for an agent. -->

### Evaluations architecture

We need evaluation criteria that would be not only functional, but tied to numbers and count of tool calls/reviews from LLM-as-judge.

Bad example: specific as "Markdown would be valid"
Good example: Testing of markdown for structural validity be successful in 95% of cases.

### Multi-level evaluations architecture

We should be able to test evaluations on multiple tiers, specifically:

#### Fast

1. Testing of markdown validity (for planners, TODO lists, reviewers) | 95% of cases on the first prompt.
2. Testing of code validity, ruff/pyright checks - 90% of cases on an average tool call
    - Including, specifically, build123d code. So we would generate a valid build123d code in 95% of the cases.
3. Testing that output YAML is valid in 95% of cases.
    - In `objectives.yaml`, the objectives do not intersect, specified objects do not intersect
    - In `objectives.yaml`, the objectives are in bounds of the model.
4. Given a prompt, agents read the necessary skill documents (as instructed).
    - For each agent.

#### Medium evals

##### Medium evals - Engineer evaluations

###### Medium evals - Engineer Planner evaluations

<!-- We probably want to split this into further three sections below. -->
1. Given a prompt, an engineering planner uses appropriate components to solve a problem, and creates plans that are physically achievable, does not have intersecting models, and actually passes.
    - Validation:
        1. YAML
        2. LLM-as-a-judge (plan reviewer).
2. Given a prompt, the engineering planner uses accurate prices from the catalog and doesn't "come up" with prices.
3. Given a prompt, an engineering planner does not generate plans for features outside of a build zone.
4. Given a prompt, an engineering planner plans for a solution that is equal or lower to the `max_unit_cost`, `max_weight`, as well as other numeric constraints in 95% of cases.
5. Given a prompt, the engineering planner produces plans with correct units (e.g. metric or US customary units in 95% of cases.)

###### Medium evals - CAD Engineer

1. Given a plan, the engineer will pass manufacturability checks in 70% during validation and pricing tool call. (note: lower than during submission because this is explicitly not submitting but validating the tool call).
2. Given a plan, the engineer will pass manufacturability checks in 90% of tool calls when expected during simulation or submission (when they *expect* that it will definitely pass). On second and third attempts this will improve to 95% and 97% respectively.
3. The engineer would adhere to requests of the user (passed indirectly to plan) about availability or forbid of drilling of the surrounding environment (note: it should be per-surface, not globally. E.g. one may drill into the floor but may not drill into another machine; but that will come later)
4. The engineer, after simulation, would interpret the simulation results correctly [...] (how to enforce it? they wouldn't always need to view the resuts, they can use final positions table that should be output or text too.)
5. The engineer will prefer CSG over sketches in 70% of the cases (soft requirement, but it makes it actually easier to build with code).

###### Medium evals - Engineer Reviewer

1. Given a viewed model, the review agent will correctly navigate images of the output (would check at least 3 images) before solution
2. Given a viewed model, the review agent would be able to correctly identify the issues in the plan in at least 70% of the cases.
    - Correctness: given a valid plan with an issue introduced by another LLM, a reviewer would be able to spot the issue in the plan (with the first LLM or other LLM validating that in fact, the described found issue matches).
3. Given a viewed model, the reviewer would force the CAD engineer to provide a cheaper solution in at least 15% of the cases.
4. Reviewer efficacy - reviews would be successful enough that they lead to a solution in at least 60% of cases after a failure. This means the reviewer should look deeper than one surface problem.
5. Price/weight escalation refusal - If the price or weight was set inadequately, the reviewer would scrutinize how the CAD model can be improved; and if they can find ways to thin/optimize a shape that would in fact be sufficient, then they will refuse the manufacturing. (specifics?)
6. Manufacturability awareness - the reviewer would be reporting only solutions appropriate to a given manufacturing method.
    - The reviewer would not be reporting solutions inappropriate to the manufacturing method in 97% of the cases.
7. Toolkit usage and diversity; the model will use:
    - fasteners in at least in 70% of builds
    - Motors in at least 20% of the builds
    - bearings in at least 10% of the builds
    - COTS search will be executed in at least 50% of builds (you need to look for specific version of fasteners, motors, etc.)
    - Other tools will be used at least reasonably often, or at least sometimes (3%?)
    (this requirement is more so for prompt debugging - that we don't miss including something into the prompt/skill sections.)
8. The model would be able to execute a search (or use a subagent) in COTS

<!-- Future: Given a prompt, the engineering planner doesn't use components that are out of stock -->

<!-- FIXME: Underspec: we don't define coordinate system starting point.I.e. is the center defined as (0,0,0)? Else how do we define it?
Proposal: normalize the simulation to the center bottom of the build zone. So the bottom center of the simulation would be 0,0,0; whatever is under it would be -z, and everything would be normalized to it. I expect it'll help agents adjust the build. -->

##### Medium evals -  Benchmark Generator

###### Medium evals - Benchmark Generator Planner

1. Given a prompt, a benchmark planner generates a plan that upon valid scrutinizing of a plan reviewer, passes in 80% of cases.
2. Given a propmt, a benchmark generator planner generates a plan that would have unimpeded objectives (objectives obstructed by obstacles by no more than 35%) in 97% of the cases (calculated by volume - shouldn't be obstructed.)
3. Given a propmt, the benchmark generator will produce plans for various manufacturing quantities (prototype <5, small volume <100, mass-manufacturing - 3000)
4. Given a propmt, we will include to the solution proposed by the propmt:
   - will include correct:
      - Quantity
      - Max weight,
      - Max cost
      - And other numerical parameters specified in objectives.yaml.
5. The benchmark generator would be able to predict the price and weight the engineer will solve the solution in the range of 80-120% (with 20% error) of the final price in 80% of the cases, within 50-150% in 97% of cases (this is the standard price, not the "safe" price)

###### Medium evals - Benchmark Generator CAD engineer

1. Given a plan, a benchmark CAD drafting agent generates a benchmark that is physically valid, uses features that actually exist in our pipeline (does correct constraints, etc) in 95% of cases after 30 turns, and 3 submissions to reviewer.

- First submission - 10 tool calls, 70% pass,
- Second submission - 20 tool calls, 85% pass,
- Third submission - 30 tool calls, 95% pass.

##### Medium evals - Skill Learning Agent (Async)

1. **Validity**: Generated skills (`SKILL.md`) are valid markdown/YAML and adhere to the skill schema in 100% of cases. (fast)
2. **Utility**: Generated skills are referenced/used by other agents in subsequent similar tasks (requires long-term tracking). (long-term)
3. **Non-duplication**: Generated skills do not duplicate existing skills (upon inspecting git changes after 30 turns- the skill rows aren't churned) (long-term) (not exactly an eval, but a tracking logic).
4. No overwrite: Skills aren't overwritten from scratch in 100% of cases
   - Skills can not be overwritten for more than 5 lines, to prevent catastrophic overwriting.

##### Journaling

1. Struggle detection: The agent detects and logs entries for 90% of "struggles" (failed tool calls > 4) detected in the logs.
2. **Linkage**: 97% journal entries are correctly linked to a unique problem/observation ID.

#### Slow (essentially, production tasks)

##### Slow evals - Engineer

###### End-to-end eval

1. Given a benchmark and a stadard prompt, the engineer builds a build123d model that is valid and reaches the goal within 30 tool calls and three simulation attempts.
    - First submission - 10 tool calls, 70% pass,
    - Second submission - 20 tool calls, 85% pass,
    - Third submission - 30 tool calls, 95% pass.
2. The Engineer (Planner+CAD Engineer) agents would not create examples that would break parts in 90% of the cases (can estimate if it (only motors for now, soft bodies in the future) will break using a formula).

###### Engineering Planner

###### CAD Engineer

1. Given a plan, an Engineer builds a build123d model that is valid and reaches the goal within 30 tool calls and three simulation attempts.
    - First submission - 10 tool calls, 70% pass,
    - Second submission - 20 tool calls, 85% pass,
    - Third submission - 30 tool calls, 95% pass.
2. Robustness test: If at least 1 simulation passes, at least 70% of "runtime jitter" variations pass too (i.e. the solution is not "flaky" that only works for one specific seed, but is mechanically robust).
3. Given a plan and `objectives.yaml`, the Engineer will not try to submit a solution that is more unit expensive or heavy than unit tests set by the planner and will:
    - Cheaper than the max price: 80% on first attempt, 90% after the first failure, 95% after the second failure.
    - Small error: 0-20% more expensive - In 10% on first attempt, 5% after the first error, 3% after the second failure (only small failyre, not including large error).
    - Large error: >20% more expensive - In 10% of first failure, 5% on second failure, 3% after after the second failure.

###### Reviewer

Plans not rejected by

##### Slow evals - Benchmark generator

###### Slow evals - CAD Engineer

1. Given a correct plan, a benchmark generator builds a benchmark that an engineer would be able to solve within 30 turns in 70% of cases.
2. Quality of Reasoning: The reasoning traces for benchmark generation are coherent and logically sound in >90% of cases (important for dataset quality).

#### Integration/post-processing evals (more than one episode)

There are some episodes which can be take multiple episodes to run.

1. Plan-to-CAD fidelity: after the CAD was successfully reconstructed into a successful solution, generate a plan, and reconstruct again. The result (volume) should be approximately the same (80%).
<!-- That said, I'm not sure if it's necessary. -->
1. Robustness to randomization - having solved 1 seed from a randomization batch, an engineer would be statistically 50% more likely to other episodes in the batch relative to average performance (would fail 50% less).
2. If a reviewer said "this is acceptably the cheapest we can get" and then the model got cheaper by 50% unit cost at the same quantity (during subsequent planning) - the reviewer in fact didn't find an optimal quantity, and that shouldn't happen... (in which cases out of which?)

###

## Distributed execution

There is a controller node which runs the LLM and tool calls, and there worker node which:

1. Executes the simulation,
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

The worker's filesystem is implemented as a "disposable sandbox" via `SandboxFilesystemBackend` in `deepagents`.

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

## Simulation and "Definitions of Done"

While this platform has notable downsides for future use, we pick MuJoCo, because it's battle-tested, and requires almost no compile time.

<!-- Downsides of MuJoCo?

- we won't support deformation (finite element analysis)
- we won't support fluids

But for an MVP this is fine. -->
<!-- The corollary of not being able to run FEM is that the model can produce physically inadequate parts and still succeed. But I can't do much about it yet. -->

### Simulation constants and assumptions

We operate in a real-world-like scenario, with rigid bodies, gravity,  real-world materials - with all standard properties like friction, restitution (bounciness), etc.

In environments, some objects are fixed, whereas others can be freely hanging or partially constrained at will to other objects - the input environment is not "bound" by physics too much. Whereas, the engineer-created objects can *never* be fixed unless they are properly constrained (in near future, we want to add "bolting" mechanism to the environment - i.e. the model would drill a hole in an environment and constrain it's object to the hole or multiple holes); and all constraints must be physically-realistic.

#### Physically-realistic constraints

In the end, our systems should be transferrable to the real world.

For engineers, constraints must be physically realistic. Meaning: if an engineer agent tries to constrain two parts together, they need to use fasteners or make a mechanism which would fit two parts together. However, the engineer can't constrain two parts by just assigning them a CAD constraint.
This is because it fits how physics works is in the real world, transferring to which is ultimately the goal of this project.

We can perhaps verify it by simply adding realistic fastener logic.

Similarly, while you can constrain a ball to a plane in CAD, you can't do so in real life. In real life, a ball needs to have a special holder. Two flat planes can't be constrained to each other, you need to either add them, make a real constraint that would hold them together.

##### Creating realistic constraints

Constraints done by the engineer should be enforced for validity. E.g.: two parts should be actually close together

###### Fixed parts for the simulation definition

Some parts will need to be "fixed" despite physics *during benchmark generation, not agents*, specifically for the implementation. We can pass `fixed=True` to the models as a custom parameter (or metadata).

###### Fasteners

We use **build123d's native `RigidJoint` system** for mating parts. This avoids custom positioning math — build123d handles transforms automatically via `connect_to()`. Fastener geometry (bolts, screws, nuts) comes from the [`bd-warehouse`](https://bd-warehouse.readthedocs.io/en/latest/fastener.html) package.

**Helper function**: `fastener_hole(part, pos, depth, diameter, hole_id: str, hole_type: HoleType = HoleType.FlatHeadHole, add_fastener=False)`

1. Cuts a hole at `pos` using build123d `Hole` (or `CounterBoreHole`)
2. Creates a `RigidJoint` at the hole location with a parameter `rigid_joint.hole_id=hole_id`
3. If `add_fastener=True`, inserts appropriate fastener from bd-warehouse catalog
4. Returns the modified part

The type of Hole is determined by an enum - HoleType: `FlatHeadHole`, `CounterBoreHole`, `CounterSinkHole` for according types of holes.

###### Agent workflow for fasteners

```python
from utils.fasteners import fastener_hole

# Create bracket (anchor part) - explicitly positioned
bracket = Box(100, 50, 10)
bracket = fastener_hole(bracket, pos=(20, 25), depth=10, diameter=5, hole_id="mount_1")
bracket = fastener_hole(bracket, pos=(80, 25), depth=10, diameter=5, hole_id="mount_2")
bracket.position = (0, 0, 100)  # world position

# Create arm - will be positioned via joint mating
arm = Box(200, 30, 8)
arm = fastener_hole(arm, pos=(10, 15), depth=8, diameter=5, hole_id="arm_1", add_fastener=True)
arm = fastener_hole(arm, pos=(50, 15), depth=8, diameter=5, hole_id="arm_2", add_fastener=True)

# Mate parts - build123d computes transform automatically
arm.joints["arm_1"].connect_to(bracket.joints["mount_1"])
arm.joints["arm_2"].connect_to(bracket.joints["mount_2"])
```

After `connect_to()`, the arm is automatically positioned so holes align. **No manual rotation/translation math needed.**

Note: hole names are given readable names, e.g. explicit names like "front_mount" or "pivot_hole" for easier identification. In fact, the hole name serves as a local label for the joint. So that we can reference the build123d joint with its hole name.

"""
Without hole_id:

```python
# How would you reference the joint?
arm.joints[???].connect_to(bracket.joints[???])
```

With hole_id:

```python
arm.joints["arm_1"].connect_to(bracket.joints["mount_1"])
```

"""
**Validation rules**:

- Single-fastener connections are **rejected** (underconstrained — allows rotation around bolt axis)
- Minimum 2 fasteners required for rigid connection between parts <!-- Note: this is not true, actually. You can design such inserts that only 1 will be sufficient. But, let it be.>
- Hole diameters must match between mated pairs
- Can't connect holes with both `add_fastener=True`. <!--Note: not a hard constraint - if it's difficult to do, skip.-->

**MJCF translation**:

1. Walk assembly, find all `RigidJoint` pairs that are connected
2. For each connected pair: emit `<weld body1="..." body2="..."/>` constraint
3. Fastener geometry is included in physics only as a visual (cosmetic in CAD renders only) <!-- (I don't care about making that collision with head. Actually, it's rather simple - just put the fastener at it's last position in CAD. But still.) -->

###### Edge case: multiple holes

The `hole_id` is **local to each Part**, not a global identifier. When one central part connects to multiple identical parts, each child part can have the same `hole_id` (e.g., `"attach"`) — the matching happens via explicit `connect_to()` calls:

```python
# 4 identical legs with same local hole_id
for i, leg in enumerate(legs):
    leg.joints["attach"].connect_to(bracket.joints[f"mount_{i+1}"])
```

This avoids the need for global ID management or dict-based hole matching.

###### Mechanisms and Moving Parts

MuJoCo constraints will only ever be spawned from predefined components. Meaning, a "revolute constraint" will only ever be spawned if there is either a:

1. Bearing (ideally),
2. Motor,
3. Through hole intentionally created for two parts.

For each, the internal/external diameters must match, and there will be special commands on how to define these. In addition, the critic will be prompted to specifically scrutinize if the constraint is valid. In addition, parts will have to be close to each other physically - distance between both must be nearing <1 mm or so.

This is to prevent the CAD designers from creating impossible constraints.

To support moving parts (hinges, sliders, motors), we force build123d Joints from *to be created from predefined CAD components* almost always - e.g., again, revolute joints from bearings, rigid joints/weld constraints via fasteners.

Notably this will also be affected when we will (later) transfer to deformable body simulation and we'll need to find ways how to make simulation stronger:

Map of joints to MuJoCo constraints and their uses:

1. RigidJoint to `<weld>` constraint:
    - Used for fasteners and fixed connections.
    - Connects two bodies rigidly at the joint location.
2. **RevoluteJoint** to `<joint type="hinge">`:
    - Used for axles, pivots, and motors.
    - The joint axis in build123d becomes the hinge axis in MuJoCo.
    - If the joint is motorized, we add an `<actuator>` targeting this joint.
3. **PrismaticJoint** -> `<joint type="slide">`:
    - Used for linear sliders and rails.
    - The joint axis defines the slide direction.
    - Can be motorized with a `<position>` or `<motor>` actuator.

###### Implementation Logic for constraints

- Walk the `build123d` assembly and inspect `part.joints`.
- If a joint is connected (via `connect_to`), identify the two parts involved.
- Assert the joint is valid programmatically (distance, not conflicting with other constraints, etc.)
- Generate the appropriate MuJoCo XML element connecting the two bodies.
- Assign stable names to identifying joints so controllers can reference them (e.g. "motor_joint").

##### Allowed components in simulation

The simulation would have only a set number of components that both benchmark designer and engineer can use. The following list is acceptable:

1. 3d CAD parts:
    - Environment (unmodifiable, or modifiable with minor changes, e.g. drilling);
        - Objectives (goal, forbid zones)
        - Parts (any obstacle/standard CAD object) <!-- probably needs for a better name-->
        - Input objectes (e.g. - a ball that needs to be delivered somewhere.)
    - Engineer parts:
       - 3d CAD parts representing real-life objects that engineers would normally create; bound by all physics.
2. Motors (and simple scripts/functions that run the motors, e.g. in sinusoidal wave, or start/stop every few seconds). Accessible by both engineer and benchmark generator.
3. Fasteners - Accessible by both engineer and benchmark generator, however likely environment doesn't really need them.

<!-- Future:
Bearings.
Gears,
PCBs
Wires
Fluid vessels, e.g. pipes, hoses, or tanks that supply each. 
Fluid pumps.-->

#### Constants

- Simulation timestep of the rigid-body simulation -  0.002s (default mujoco)
- Max simulation time - 30 seconds (configurable globally)
- Max speed - >1000m/s
- Default benchmark size - 1\*1\*1m
- Default stretch - 0.5\*0.5\*0.5 to 2*2*2, disproportionally
- Collision:
  - How often is the simulation checked for collision with goals - every 0.05s.
  - Number of vertices needed for collision - 1 (maybe more in the future)
- Units: Metric.
- Safety factor (for motors and parts breaking) 20%.

### Convex decomposition

We don't have convex decomposition logic in MuJoCo (we do in Genesis, but we'll approach it later). We'll need a V-HACD logic on worker.

<!-- Note: I have no clue about how V-HACD works. Assume good defaults. -->

### Motors

We use standard MuJoCo actuators. They need to be controller by the controller functions.

#### Controller functions

We need to define how motors will behave, abd we'll use a controller. For this, create a util package like `controllers`, which would have time and position-based controllers.

##### Time-based functions (take in `t` as time)

1. Constant - `constant(power:float) -> float` <!-- as far as I understand, a standard MuJoCo <motor> -->
2. Sinusoidal - `sinusoidal(t: float, power:float) -> float`
3. "full-on, full-off" - a.k.a. a "square" function in signals - `square(time_on_time_off: list[tuple[float,float]], power:float) -> float` - takes in lists of time when to start and stop; and how much power it would output.
4. "smooth on, smooth off"- a.k.a. a "trapezoidal function" in signals `trapezoidal(time_on_time_off: list[tuple[float,float]], power, ramp_up_time: float)`

Note: I'm not a pro in these functions - maybe they need renaming. but that's the idea.

Note: they will need to be importable utils, just as tools like `simulate` are.

##### Implementation for time-based controller functions

One easy way to implement it is to define a dict of control functions, then pass it to simulation logic, and it would control the motors by their control functions. On the other hand, the `objectives.yaml` will contain which functions the motors are referecing. the moving parts.

##### Position-based functions

Oftentimes of the time we'll want to control motors through positions, e.g. servos or stepper motors. Define a set of functions that would do inverse kinematics (rotate the motor to a given position, at least)

We want to allow to do something like "at 5 seconds, rotate to 45deg, then at 10 seconds, rotate to 0, and at 15 seconds rotate back to 45 deg." This will also involve Python functions (probably pre-determined). At least a basic set of these (time-based, constant).

<!-- In the future work, I presume, full inverse kinematics pipelines are desired. I know they are trivial in Genesis, it seems not so much in MuJoCo. -->

<!-- Notably, MuJoCo already has some... motor types: " MuJoCo has `position`, `velocity`, `motor` actuators". I don't know how they work -->

<!-- Warning to self: objectives.yaml gets bloated with moving parts definition, which doesn't explicitly belong in there. -->

###### Position-based controllers implementation

""" AI-generated, I'm not a in the MuJoCo motors.
For position-based control (servos, steppers), we use **MuJoCo's native `<position>` actuator**:

```xml
<actuator>
  <position name="servo1" joint="arm_hinge" 
            kp="{kp_from_COTS}" kv="{kv_from_COTS}"
            forcerange="-{max_torque_nm} {max_torque_nm}"/>
</actuator>
```

**Key differences from `<motor>`**:

- **`ctrl[i]` meaning**: Target position (radians for hinge, meters for slide) – *not* torque
- **Internal PD control**: MuJoCo applies `torque = kp * (target - pos) - kv * vel`
- **Physics-based tracking**: The joint "seeks" the target position naturally (no teleportation)
- **`forcerange`**: Clamps output torque to realistic motor limits (prevents infinite force)

**PD gain tuning** (critical for stability):

- Gains must be tuned relative to body inertia
- Low inertia + high kp = numerical explosion
- Safe starting point: `kp=5`, `kv=0.5` with `mass=1`, `diaginertia=0.01`
- Add joint `damping` to improve stability further

**Available position controllers** (`worker.utils.controllers`):

- `waypoint(schedule: list[tuple[float, float]])`: Move to target positions at scheduled times
- `hold_position(target: float)`: Hold a fixed target position  
- `oscillate(center, amplitude, frequency, phase)`: Sinusoidal position oscillation

"""

Notably, we have a set of COTS motors in COTS section below. We need to assume/research COTS actuator strength and parameters.

#### Actuator force limits (forcerange)

MuJoCo's `forcerange` attribute clamps the actuator output to realistic torque limits:

```xml
<!-- Example: MG996R hobby servo with ~1.1 N·m max torque -->
<position name="servo" joint="arm" kp="15" kv="0.8" forcerange="-1.1 1.1"/>
```

**Behavior**:

- If PD control computes torque > `forcerange`, it's clamped to the limit
- Motor "struggles" realistically when overloaded (can't reach target)
- Simulation does NOT fail from clamping alone (see below for failure logic)

**Source of values**: `forcerange` comes from COTS servo catalog (`max_torque_nm` field).

#### Motor overload failure

We don't want motors to break; set the maximum *sustained* load threshold above the servo's rated torque.
If a motor is clamped at `forcerange` for more than **2 seconds continuous**, the simulation fails with `motor_overload`.

This forces agents to:

1. Pick appropriately-sized motors for the load
2. Design mechanisms that don't exceed torque limits
"""
Note: AI-written, I'm not a pro in MuJoCo motors.

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

##### Checking for objective interaction

As above, "The forbid or goal zone is triggered if the agent touches it even slightly". This means that if any vertex (let's limit to vertices for simplicity/speed) touches the simulation, it fails.

#### Randomization

<!-- LLM-generated from my other spec. -->
The benchmarks are randomized to enable a wider data distribution with less generation effort.

"Static" randomization is stronger than the "runtime" randomization. Static randomization are complete variations of the environment - stretching the entire space, stretching objectives, etc. Whereas runtime randomization - meant to make the engineer less prone to "overfitting" their CAD to the exact environment - is smaller.

##### Static randomization

- The benchmark volume size can vary 2x in all sides, and will be rescaled to random values, e.g. 1.68\*0.8\*1.3; the benchmark generator agent can narrow the scaling down if somehing is expected to break; however it is undesirable as we want to keep randomization higher.
  - The environment - objectives (goals, forbids, build zones) are rescaled respectively.
- Goal, and obstacle positions are randomized by up to 40% of their size inwards (meaning they are becoming smaller and repositioned anywhere in the region where they are becoming smaller; smaller by their own size. They will always stay within original (maximum) bounds for safety).
- The models that make up the scene can and should be different. Up to the point where it can be solved; but the benchmark generation agent must ensure randomization of the environment too; which can be also made quite extreme (which is preferred - we are not to make it easy.)

###### Material static randomization

If a part is moving, (has degrees of freedom), let us randomly switch its material for a more randomly generated environment - e.g., a part would be heavier, lighter, more/less stiff, have more/less restitution, have more/less friction coefficient; the material would be from predetermined files.
The engineer would be informed about materials of various parts ahead of time.
(notably, the benchmark generator should probably allow constraining some materials but only optionally so - e.g. if something is translated by the motor, in probably isn't ABS plastic, it's at least a metal. Allow the "minimum strength" or similar material selection.)

###### Visual static randomization

To allow for better visual generalization and more realistic environments, environment parts will change their colors to alongside of the material change, defined in materials config.

##### Runtime randomization

Notably, the engineer is informed about runtime randomization to prevent unexpected issues.

- The spawned "moved" object will also include some position jitter to ensure the CAD model's robustness against variable input.

###### Runtime randomization verification

The runtime randomization will run the simulation multiple times (e.g. 5) to ensure consistency. This is trivial to do, as MuJoCo is made for paralel simulations with slightly varying input positions.

#### Failure

Failure is achieved via either of:

1. Timeout of the simulation
    - How to define timeout of the simulation? that's a question. I suggest putting a hard cap of 30 seconds on all simulations and letting the benchmark planner decide how quickly a given goal should be achieved (with leeway); no more than 30 seconds though.
2. Any of components going out of bounds of the workspace
3. Instability in simulation (e.g. NaNs, parts interference)
4. Any part going into forbid zones.
5. Any part is broken:

    - With "passive/static" parts: break upon stress which is higher than max stress - safety factor(note: not applicable for now as we are simulating rigid-body only).
    - Some parts have custom breaking logic - e.g. motors can be overloaded on shaft.

### Conversion of CAD to mesh and to MuJoCo

We will convert `build123d` CAD files into `obj` format (not STL, because the it is very bulky), and then put the obj into mesh. I think build123d allows a `export_obj(Compound|Part)` function.

The conversion pipeline is - putting every part into mesh;

When I implemented a similar pipeline some time ago, it was helpful to: recenter all build123d parts so that they are at (0,0,0), then export them, then add them to MuJoCo with confidence at their known position (and trivially) because they are at (0,0,0). We need to know build123d part position ahead of time though.

<!-- Note: Genesis if we'll migrate to it, supports GLB. -->

### Preview/visualization

We want the agent to be able to preview their own CAD models (likely done more often). We will render CAD images, not MuJoCo for it. The materials will have their colors.

#### Mesh limits and Simplification

The mesh is unbounded in vertex counts because we are simulating engineering-grade materials. That said, the mesh should be simplified where *safe* to do so; however the higher quality is desired.

<!-- For rigid mesh only - not for deformable materials(!), we do this:
The mesh is unbounded in vertex counts because we are simulating engineering-grade materials. However, to ensure **simulation stability** and **performance**, we use a dual-mesh strategy:

1. **Visual Mesh**: High-quality, high-poly mesh (e.g., `angular_deflection=0.1`).
    - Used for rendering and visual inspection.
    - Preserves cosmetic details.
2. **Collision Mesh**: Simplified, decimated mesh (e.g., `angular_deflection=0.5` or `trimesh.decimate`).
    - Used for physics calculation and V-HACD decomposition.
    - **Loss**: Curved surfaces become faceted (spheres look like polyhedrons). Small features (threads, text) are smoothed over.
    - **Gain**: 10x-100x faster collision detection, fewer "thin triangle" artifacts, more stable contacts.

**Implementation**:

- `builder.py` exports two OBJ files per part: `part_visual.obj` and `part_collision.obj`.
- V-HACD is run ONLY on the collision mesh.
- MuJoCo XML references the collision mesh for `<geom class="collision">` and visual mesh for `<geom class="visual">`.

Watertightness is required for both. -->

<!-- Note: when implementing this logic, don't overcomplicate it. We'll migrate to native logic in Genesis relatively soon (which simplifies it without any extra config at all, including mesh decomposition). I don't care too -->

### Materials

We have a set of materials defined in `manufacturing_config.yaml`, which defines: `materials` section, and which materials can be used for the simulation - their weight, price, and cost per KG. The config is auto-validated with unit tests (e.g., can't reference and inexisting material).

`manufacturing_config.yaml` can be read-only for the agents to gauge the pricing ahead of time. It is also used during programmatic validation of manufacturability.

`manufacturing_config.yaml` sample schema:

```yaml
manufacturing_processes:
  cnc:
    setup_price: [...]
    price_per_X: [...]
    price_per_Y: [...]

  injection_molding:
  # [...]

materials:
  alu-6061:
    color: #
    elongation_stress:
    restitution: 
    friction_coef: 
    # and others
  ... 
```

The materials are only ever chosen from the config.

## Observability

To track all agent movements and to persist data, we encode the following:

1. The agent pass/fail reasons
2. Error messages from script execution, linting,
3. All agent thoughts.
4. A mechanism to reconstruct those - e.g. we record the entire conversation and tool-calling structure, so how can we read it? How can we show it to users that use this prompt? How can we use it for debugging? basically, some order matters. Or, maybe just dump the conversation in/out to schema, that could also work.
5. Renders (visuals) of what the agent sees (optionally; if it doesn't take too much space; it may very well be more economical to reconstruct at runtime using a scripts. So record what comes into the agent, all parameters, code, setup, etc, and rebuild at runtime.)
6. Feedback from the user.
7. All detailed metadata as required by "Evaluation" database, and more. All bits of data on what called when should be extracted.

These will be later used for querying, preproc and model training.

### LangFuse for observability, DB for deeper logs

We use LangFuse for LLM observability. We will use a Railway template for deployment / deploy a separate container on docker compose locally.

Langfuse is deployed locally/on Railway.

For deeper observability, e.g. requirements like "fasteners were used in at least 70% of cases", store fastener usage in the application database.

#### All collected events

We track the following structured domain events to compute the evaluation metrics defined in the [Agent Evaluations](#agent-evaluations) section:

1. Component usage,
2. Tool invocation,
    - separate events for each tool call - easier to track later.
3. Manufacturability and price check (engineer)
    - store all metadata too - verified which part, for which manufacturing method, result(pass/fail; weight price, etc.)
4. Scene valiation (Benchmark CAD engineer)
5. Render request (engineer)
6. Render request (benchmark)
7. Simulation request (engineer)
8. Simulation result (engineer)
    - Failure/pass reason -
        - Success: hit green zone
        - Failure in [failures](#failure):
            - Timeout,
            - Out of bounds
            - Forbid zone hit
            - Part breakage
    - Simulation time elapsed
    - Computing time

9. COTS search (engineer/planner?)
10. Plan submission (benchmark)
11. Plan submission (Engineer)
12. Price/weight failure escalation request (CAD engineer)
13. Price/weight failure escalation decision (reviewer)
14. Lint failure - code
15. Lint failure - Markdown/YAML
16. Logic/constraint failure - YAML
17. Skill edit (skill editing agent)
18. Skill file read (any agent) (note: track reading of skills or even particular files or even lines may link to success rates.)
19. Instability in the simulation (if an agent produced an instable solution, or a NaN somehow and we didn't catch it)
20. Submission attempt without creating all necessary files.
    - if planner tried submitting the result without either of `plan.md`, `objectives.yaml`, `preliminary_cost_estimation.yaml`, OR they were left equal to their templates (don't allow submission), and note an event.
21. Submission from reviewers - Review decision events for every reviewer stage (benchmark reviewer, engineer reviewer) with decision, reason category, and evidence used (images viewed count, video viewed, files checked).
22. Plan refusal events with an explicit refusal reason and proof-of-impossibility evidence (note: no agent currently for plan refusal)
23. Forbidden joint creation/adding logic.

<!-- 20. Metric for "Jamming Rate"
    - Definition: Object velocity = 0 for > X seconds while Actuator Force > 0.
    - Why: Distinguishes between "I didn't try" and "I tried but the mechanism jammed". Crucial for debugging friction/tolerance issues -->

<!-- Code quality: all events are enums and all events have models for strict schema.-->

##### Not just events, but numeric events

Notably, outside of just events, we want to track crucial numbers; especially for metrics below. E.g. cost/weight estimate events from planners and actual cost/weight from validation, for error analysis. *Technically*, we don't exactly *need* an event because we have the objectives.yaml in s3, but extracting the data from `objectives.yaml` and other "numeric" files (yaml frontmatter included) will make it very handy for downstream analysis (and would save time for analysis downstream.)

##### Tracking seeds

In addition, we track randomization seed and variant events for every run (static variant id, runtime seed). Needed for robustness/flakiness metrics.

#### Metrics

We defive (a growing list of) (aggregate) metrics:

<!-- All below are LLM suggested, but are good. -->
1. Benchmark solvability rate: % of generated benchmarks solvable within constraints by the engineer (or baseline solver).
2. Benchmark diversity coverage: distribution across physics principles (gravity, friction, motors), object types, DOF counts, moving parts, and environment templates.
3. Robustness across seeds: success rate across runtime jitter seeds and static variants.
4. Plan adherence rate: how often CAD output matches plan (geometry, constraints, objectives).
5. Price/weight estimation error: planner estimated vs actual validated cost/weight, by agent and benchmark type.
6. Time-to-solution metrics: median time/tool-calls to first valid benchmark and first valid solution.
7. Reasoning trace capture rate: % of runs with stored traces; trace sufficiency score (presence of required sections).
8. Journal quality score: % of entries with intent/result/reflection/next-step; frequency per run; correlation with success.
9. Optimization capture rate: % of runs where notable optimization is logged (objective #5).
10. Skill effectiveness: performance delta before/after a new skill version.
11. Reviewer precision/recall: false accept/reject rate based on downstream outcomes.
12. Simulation stability rate: % of solutions with no instabilities, NaNs, penetrations, or joint violations.
13. Dataset readiness score: % of runs meeting training-dataset criteria (complete artifacts + verified solution + valid reasoning trace).
14. Cost/weight delta heuristic: if cheaper/lighter alternative was computed (simulated) but final solution is worse, log event.

<!-- 1. Infrastructure/framework stability:
    - % of sessions completed successfully to their expected end and not failing under timeouts, container crashes, etc.LLM-suggested. -->
<!-- 2. Denser reward signal - track normalized distance to target, something like `Score = 1.0 - (min_distance_achieved / initial_distance)`. Ideally (not mandatory, probably skip for now), also measure a performance of simulation simply without any changes (what if we do nothing - how good is the result?)
    - it's a metric that was more used in RL, but it's infromative... sometimes. LLM-suggested. -->

<!-- LLM-suggested. -->

<!-- 11. Library growth and reuse: new build123d modules added per period and reuse rate across tasks. -->

#### Bulk uploading events

<!-- This part was LLM-suggested, but is OK -->
We decided on persisting a local `events.jsonl` file with all events for deeper observability instead of sending individual events or a sqlite DB. This would allow sending all the events in one go instead of dozens of small requests. For 100 events per a 3-5 minute session (if that), it is acceptable. In fact, it wins a bit of performance - opening DB a hundred connections is slower than opening one and batch uploading it.

### Best practice: Give LLMs a way to complain

We have a "sidecar" model that checks where the agent was confused and updates skills.

We can reuse the same approach for debugging.  A LLM that explicitly checks where the model got confused and sends a tool call if something was really wrong. The developer gets that into a DB/observability DB. **The developer can use the LLM output to debug.**
(ideally, we would also heap those and send them to Google Jules which will fix bugs/propose functionality/ etc.

<https://jules.google/docs/api/reference/>)

Create another sidecar model running async (maybe batch) that would read reasoning traces of the agent and populate the issues. Basically an agent that debugs. It saves me, a developer, a pain in the of debugging.

### Backups

In prod we will backup the database(s) daily.
<!-- Notably, the file could be quite big, as we persist sqlite text. Max compression it before backing up. -->

One way to do it is by sending a `cron` job daily. Thus, implement an endpoint which will accept a cron call, and will back up the SQLite folder to the s3. Again, this is in production.

### User Reviews

I want to track successful/failed reasoning via user thumbs up/down during:

1. Benchmark generation planning
2. Benchmark Generation implementation; specifically voting on individual file outputs and voting on the output "as a whole". Ideally, the users would also be able to input their feedbacks on individual subassembly parts, as defined by the assembly view in the frontend, by selecting them and pressing thumbs up/down and inserting a comment.
3. Implementation Planning
4. Implementation Generation.

We are using LangFuse and it has those capabilities natively.

#### Training on user reviews

When user stores the review, first of all, store it in the observability DB, and then we'll also need to incorporate an automatic prompt improvement with powerful models like Claude 4.6 Opus with human review that will be merged/skipped.

We temporarily won't do auto-improvement through a model, though I bet we'll need it in the near future.

<!-- Note: I don't know why, but the Gemini models are good at coding but horrible at making prompts/interpreting desired prompt behavior; don't use them for this purpose. -->

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

The rest (submitting the work, testing for design validity, etc) is called via and calling python functions in the code. (as described below)

#### The "tools" as Python functions - Utils

I propose the following set of tools (their usage is below). Notably, the tools are python imports and functions, called right in one of the edited files!

##### Engineer tools

- `validate_and_price(component: Part|Compound) -> float|dict[str, float]`: validates a part by for manufacturability, then prices it if valid using its workbench's cost calculation interface, or returns an error with a description and a location
  - If validating a compound, it will also check for unusal DOFs, e.g. a part has >=4 DOFs, which is unusual in engineering. It won't raise immediately, but it will throw a "warning". The reviewer will also get notified that DOFs are excessive in this part in particular, and will be more strict in review.
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
- `submit_for_review(Compound)` - submits the whole benchmark compound for a review to `Reviewer` agent node, which can later approve it and thus putting it to the "to solve" pipeline.
- `get_docs_for(type)` - a util invoking a documentation subagent that parses skill and then b123d documentation (local copy, built into container) in search of documentation <!--note: it's probably ideal to have some service like Context7 which does it for us-->
<!-- Note 2: `deepagents` framework supports subagents this is what we'll use here.-->

#### Planner tools

`validate_and_price` except for manufacturability.

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

<!-- In the future, it's very interesting to support topology optimization, but that's a separate project. -->

### Off-the-shelf parts (COTS)

It is self-understanding that engineers will use off-the-shelf parts - motors, fasteners, gears, etc. The catalog is well-defined in spec 007, but the model should have an access to a CLI tool or a set of python scripts to find something in a codebase. Again, we use CodeAct.

I suggest using a subagent for this. Give a prompt of what's necessary and let the subagent execute a series of read-only SQL prompts over a catalog DB. The agent will return a series of catalogs to use.

Both planner agent and engineer can only prompt the searching agent for searching.

## Code and responsibility separation

We are building open-source, and we trust our compute nodes, so no complex "zero-trust" architecture is necessary.

All the code that runs in the controller app will be in the controller node directory, and all the worker node will be packaged into the controller node.
<!-- note: the above was failed. -->

Both controller and worker will have their own container files.

Controller will be responsible for scheduling the jobs and using LLMs (and containing all secrets, except github push personal access token for skill repo - and even that is for simplicity) and worker should be responsible for doing all the heavy lifting - from linting, formatting, (and have ephemeral storage) .

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
evals/
frontend/
worker/
controller/
<!-- optionally, workspace, to store agent output during testing to not have them appear in the file. It is basically a tempdir.(perhaps, should be moved to `/tmp/`.)-->
/workspace/ 
```

and a schema with a OpenAPI schema.
<!-- Ideally, The worker and controller have separate `pyproject.toml` and `uv` locks. But it's fine to ignore for now as they have shared tests.-->

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

#### Multiple agents per machine CPU

A single agent would consume a single cpu only during simulation. Thus, it's ideal to allow multiple agents to run on a single node; but don't allow multiple simulations at the same time to avoid OOM issues or CPU overload.
By default, there should be 4 agents per a machine or two, but only one can simulate.
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

The APIs are to be strict. OpenAPI schemas will be autogenerated on git commit hooks of controller and `schemathesis` will fuzz API endpoints. We have two schemas - one between worker and controller, another between the frontend and the controller.

We use Pydantic, and we use Beartype for hard type checking.

### Batch support as a first-class citizen

Both batch generation (batch solution, or batch simulation) support and frontend (as below) are first-class citizens.
Batch generation will be ran from a script or a container, offloading to multiple workers.

#### Async execution

For both frontend and for backend, the workers will run async, with a parallel execution requirement.

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

##### Benchmark generation workflow

I need a place to enable "create benchmark" functionality. So go from a high-level prompt to a benchmark plan, through the confirm and implementation.

Here's what I suggest: add a "+ Create new" primary button instead of "history" icon next to "benchmark runs". Upon this, the data from the the current will clear (from the UI, not from the DB), and we'll go into "benchmark creation" mode. Here the "traces" section will have a textbox on top of it, prompting to create a benchmark.

Afterwards (this should be the standard agent flow), the plan.md wil be created and placed into the folder directory. the plan file will have a "start implementation" on the top of it; with it the benchmark generation will start.
The UI will be automatically updated with new models as the build123d generation will progress.

##### Interrupting the worker and progress bars

If we want to stop the generation in the controller, it will also halt the job(s) in the workers.

Notably `deepagents` has a [support for this](https://docs.langchain.com/oss/python/deepagents/human-in-the-loop.md) - for reviewing and interruption.

##### Viewing code, plans, reviews, simulation

For both benchmark generator and engineer we want to view:

1. Markdown plans
2. Generated code (python, not MJCF) (as artifacts)
3. Final/starting renders; latest renders. (as artifacts)
4. Reasoning traces; meaning LLM reasoning steps and how it arrives to its actions.
5. 3d view.

(additionally), history of the implementation.

###### Component layout

Benchmark generator and engineer viewer have a very similar structure (I wouldn't make them one piece of code yet, just similar/shared components)...

1. Sidebar panel on the left,
2. We separate the entire UI to 3 columns about 3:3:6 split - 3 is the current sidebar column, 3 for reasoning traces (traces streamed realtime), and 6 for the rightmost column (adjustable, of course.)

The rightmost column is split vertically into:

1. a 3d view on top (like it currently is)
2. an artifact view. Artifacts are plans, reviews etc. it's like a mini-tree + cards on top view - the file tree is collapsible and cards on top can be used to switch files.

###### CAD viewer

Turns out viewing CAD is not as easy. In addition, special-purpose CAD viewing functionality is desired - for being able to "click" on a face and prompt something to a model - we'll need it in the future.

Ideally, this is solved via WASM in the browser. But I can't give two craps about debugging WASM yet, e.g. by "Yet Another CAD viewer" which runs in WASM.

So, use a Yet Another CAD Viewer" server-side rendering for now. integrate it into the worker, ideally.
The file format is GLB (not STL) because of lesser volume.

[!Note] Exception - the frontend will query the worker to get the GLB files. (we specified elsewhere that it will communicate only to controller first.) GET-method only though.
<!-- Note: in the future it should be WASM+b123d viewer in the browser though. Or at least a CAD viewer. -->

<!-- *Current hack*: the frontend would simply send the  -->

#### Frontend architecture

Vite, React. Autogenerated types on git hooks from Controller. Super-modern, powerful look

### Config

Config - benchmark generation config, linting config, manufacturability and pricing config, and prompts will be stored in YAML files.

## Inputs to the system

### Benchmark generator

The inputs to the system are a set of prompts of what the user wants generated. A brief note about what kind of problem is presented (e.g. the simplest is - "Put a ball to the funnel and assert that it will fall to the bottom".)

The Planner will generate a longer plan with rough dimensions and will guide the creation of the agent.

The CAD drafting agent will implement, the reviewer will send back reviews or accept the benchmarks. The reviewer will also review all randomness in the script.

Upon accepting, the environment and its randomized versions will be saved to Assets.

Notably, the environment will also pre-render a set of pictures (e.g. 24 pictures - each from different side) so that the engineering agent does not have to request their generation.

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

Notably, the production workflow is not an important part *right now* (February 4). We should prioritize the development workflows. -->
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
