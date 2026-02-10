# Agent Evaluation Scrutiny (Missing Events and Metrics)

## Missing Event Capture

- Artifact creation events for every required output: `plan.md`, `todo.md`, `journal.md`, `objectives.yaml`, CAD scripts, MJCF, renders, videos, reviews, optimization notes, skills. Needed to compute dataset completeness and reasoning trace coverage.
- Review decision events for every reviewer stage (benchmark reviewer, engineer reviewer) with decision, reason category, and evidence used (images viewed count, video viewed, files checked).
- Plan refusal events with explicit refusal reason and proof-of-impossibility evidence.
- Plan compliance check events for CAD vs plan (geometry, constraints, objectives). Needed to measure plan adherence.
- Benchmark solvability events (baseline solver/expected-solution check or solvability verdict). Core to benchmark quality.
- Randomization seed and variant events for every run (static variant id, runtime seed). Needed for robustness/flakiness metrics.
- Simulation state summary events (final state, min distance to goal, collision/penetration counts, joint violations). Currently only pass/fail + timing is tracked.
- Cost/weight estimate events from planners and actual cost/weight from validation, for error analysis.
- Component reuse events (use of existing build123d modules/parts) to track the library growth goal.
- Skill version/hash linkage on every run (read/write), not just skill file read. Enables pre/post skill impact tracking.
- Journal entry structure validation events (intent/result/reflection/next step) to measure journal quality and completeness.
- Optimization attempt events (“found but not applied”), which is an explicit objective.

## Missing Metrics

- Artifact completeness rate: % of runs that produce all required outputs (benchmarks, reasoning traces, solutions, optimization notes, skills, journal).
- Benchmark solvability rate: % of generated benchmarks solvable within constraints by the engineer (or baseline solver).
- Benchmark diversity coverage: distribution across physics principles (gravity, friction, motors), object types, DOF counts, moving parts, and environment templates.
- Robustness across seeds: success rate across runtime jitter seeds and static variants.
- Plan adherence rate: how often CAD output matches plan (geometry, constraints, objectives).
- Price/weight estimation error: planner estimated vs actual validated cost/weight, by agent and benchmark type.
- Time-to-solution metrics: median time/tool-calls to first valid benchmark and first valid solution.
- Reasoning trace capture rate: % of runs with stored traces; trace sufficiency score (presence of required sections).
- Journal quality score: % of entries with intent/result/reflection/next-step; frequency per run; correlation with success.
- Optimization capture rate: % of runs where notable optimization is logged (objective #5).
- Skill effectiveness: performance delta before/after a new skill version.
- Library growth and reuse: new build123d modules added per period and reuse rate across tasks.
- Reviewer precision/recall: false accept/reject rate based on downstream outcomes.
- Simulation stability rate: % of solutions with no instabilities, NaNs, penetrations, or joint violations.
- Dataset readiness score: % of runs meeting training-dataset criteria (complete artifacts + verified solution + valid reasoning trace).
