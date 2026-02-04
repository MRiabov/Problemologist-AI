# Desired architecture (human written)

We have two agents (or agent graphs) - the benchmark generator and the engineer solving these issues.

## Benchmark generator agent (or graph)

The benchmark generator writes code for the and gets internal reviews by the reviewer inside of the generator.

The benchmarks are randomized to ensure data distribution.

The benchmarks are consisting of CAD models which are converted into XML.

- The execution runs in isolated containers to prevent accidental harmful code execution in the main system.
- The benchmarks are verified for being creatable in MuJoCo. They are not solved by an engineer yet, just benchmarks are created and verified for validity.
  - Validity means no intersections and other problems; It also means that the code will compile in the first place.
  -
  - MJCF is verified for the correctness by XML schema. And also by running a few frames of a simulation

### Benchmarks

The environments are made with static objects, dynamic objects, and motors. <!-- note - not sure if I handle dynamic objects atm. If I do, how are they specified in CAD? -->

Motors require power to run, however, we don't bother with "wiring" yet.

Problems with motors and moving parts are verified more consistently because they are more prone to error.

<!-- note: it would be useful to persist an "expected solution" from the planner during task generator. It'll help guide exploration and maybe improve LLM optimization (prompt reflection/RL) with more data. -->

## Engineer (problem solver)

Writes CAD code to solve the problems.

Is constrained by cost and manufacturability. Also constraints such as weight.

Has access to all realistic CAD tools.

The agent can preview their tools and check for whether the designs are manufacturable without being "punished".

It is OK if agents are running for 10 or more minutes (in production).

Again, the execution runs in isolated containers to prevent accidental harmful code execution in the main system.

The agents can create and update their skills to improve the future runs.

- If the agent fails too much, after resolving a solution, they will persist it to the skill.

### Engineer agent details

Engineer has a planner, which is responsible for architecting the solution, the engineer which actually implements the solution, and the critic.

The architect will create and persist a TODO list. The engineer must implement. The agent will have easy access to the TODO list.

- The engineer, after *proving* the TODO list is impossible, can refuse the plan.

<!-- Standard best practices will be used. No point in making anything weird in this application. -->

## Strict schema

We will run `schemathesis` checks against the OpenAPI. Strictly type all schema to avoid ANY issues.

### Schema autogeneration

We autogenerate python schemas, keeping in sync to the workers.

## Containerization and paralel execution

We will run benchmark generation and CAD generation/sim in paralel. The system should scale to approx 4 container on a 4-code CPU, at least. In the future we may well refactor to run on distributed notes, perhaps even IPv6.

### Persisting files

The files are generated locally by the agent. They are then sent to the container to the prod.

We need to maintain a copy of files locally, and only send requests later, such that files complete their transfer.

#### Debugging processes

We need to debug processes and that means we need a folder to store the files locally. During dev, we can use a local podmanfile that mounts/links its subfolders to the local `/workspace/[run-id]` dir. However, in production, we will containerize the main app it would store these in a volume.
And of course, we persist all of the files to the SQLite (with SQLAlchemy and Alembic) for observability.

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

## Other notes

1. Skills in the `.agent/skills/` in the repo root are different from the agent skills we are learning in the database! The repo root skills are for the coding agent to write this codebase. The learned skills should be, e.g. in workspace/ folder.
2. There is no need to reinvent the wheel here. The codebase is to use the best practices. I don't want "innovative" code that is hard to work with and demands 2x of my time.
3. "Fallbacks" lead to bad code. Early termination is preferred. When we are making 3-4 fallbacks which lead to more logic and outdated codebases, it leads to issues to everybody. Need to refactor something? confirm it and stay lean. Fail fast if the application fails, because the "happy path" isn't met.

<!-- ### Production workflow

Norably, the production workflow is not an important part *right now* (February 4). We should prioritize the development workflows. -->
<!-- Production workflow became a priority -->
