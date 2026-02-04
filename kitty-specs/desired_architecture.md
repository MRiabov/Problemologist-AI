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
