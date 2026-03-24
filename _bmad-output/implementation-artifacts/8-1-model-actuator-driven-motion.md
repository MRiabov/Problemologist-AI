# Story 8.1: Model Actuator-Driven Motion

Status: ready-for-dev

## Story

As a human operator, I want to define actuator behavior and have it simulated accurately so that powered motion is evaluated against the motion I actually declared.

## Acceptance Criteria

1. Given an actuator definition with a declared controller function or motion profile, when simulation runs, then the system applies that declared behavior to the motion model and records the resulting motion.
2. Given a benchmark that uses powered movement, when it is simulated, then the motion follows the declared actuator behavior rather than a hand-authored teleport or constraint shortcut.
3. Given a time-based or position-based actuator controller, when the benchmark is evaluated, then the actuator command is interpreted from the declared control function rather than inferred from unrelated simulation state.
4. Given an unsupported motion type, when simulation runs, then the run fails closed rather than inventing motion behavior.

## Tasks / Subtasks

- [ ] Audit the actuator-to-simulator mapping in `worker_heavy/simulation/builder.py` so the declared motion mode is translated explicitly and deterministically.
- [ ] Tighten the runtime control application in `worker_heavy/simulation/loop.py` so each actuator is driven from the declared controller function or position target, not from unrelated scene state.
- [ ] Update `worker_heavy/simulation/evaluator.py` and any adjacent backend plumbing so powered motion can be observed, classified, and fail-closed when the motion contract is unsupported.
- [ ] Extend shared motion schemas in `shared/models/schemas.py` and `shared/enums.py` only if the current strict models cannot represent the declared actuator behavior without ambiguity.
- [ ] Keep controller helpers in `worker_heavy/utils/controllers/` aligned with the runtime contract for time-based and position-based actuation.
- [ ] Add or refresh black-box integration coverage for powered motion and unsupported motion:
  - [ ] Extend `tests/integration/architecture_p0/test_architecture_p0.py` for the motor-overload and actuator-behavior baseline.
  - [ ] Extend `tests/integration/architecture_p1/test_infrastructure.py` for joint-to-actuator mapping and controller function behavior.
  - [ ] Refresh `tests/integration/architecture_p1/test_benchmark_workflow.py` only if the workflow needs powered-motion evidence or fail-closed routing for unsupported motion contracts.
- [ ] Run the affected integration slices through the real HTTP/service boundaries and verify the new behavior with state-based polling, not direct simulator internals.

## Dev Notes

- Epic 8 is the simulation family for powered motion, actuators, and motion limits. This story is the baseline powered-motion contract; overload handling and review-facing actuator evidence are separate stories.
- The codebase already contains actuator plumbing in `worker_heavy/simulation/builder.py`, `worker_heavy/simulation/loop.py`, `worker_heavy/simulation/evaluator.py`, and `worker_heavy/utils/controllers/`. Treat those as the primary implementation surface instead of inventing a second motion engine.
- The shared contract already defines `MotorControlMode` and moving-part metadata. If a new motion-profile abstraction is needed, add it in the strict shared schemas and propagate it through the simulator rather than accepting freeform strings.
- Benchmark-owned moving fixtures may be motorized or fully free only when the benchmark contract explicitly declares that behavior. Unsupported or contradictory motion must still fail closed.
- Do not satisfy this story by editing `/benchmark/validate` alone. The contract here is the powered simulation path.
- If any updated workflow emits render or video evidence, reviewers must inspect it with `inspect_media(...)`; text-only file inspection is not sufficient.
- The repository verification rule is integration-first. Use HTTP/service-level assertions and observable outputs such as traces, events, result payloads, and stored artifacts.
- Prefer deterministic failure reasons over silent adaptation. A motion mode the runtime does not understand should be rejected, not approximated.

## Project Structure Notes

- Keep motion semantics in the shared schema layer and `worker_heavy/simulation/*`. Do not add a parallel actuator contract only in prompts or templates.
- If the runtime needs a new supported actuator mode, add the enum/model in `shared/`, update the builder and loop, and then add integration coverage for both the accepted and rejected paths.
- Preserve existing rigid-body and preview behavior. This story is about powered motion in the simulation path, not about changing the validation-preview backend split.
- When the implementation touches motion evidence or terminal classification, keep the logs and event payloads machine-readable so downstream review and evals can distinguish passive motion from powered motion.

## References

- [Source: \_bmad-output/planning-artifacts/epics.md, Epic 8: Actuators: Simulation and Story 8.1]
- [Source: specs/architecture/simulation-and-dod.md, benchmark fixture motion exception, actuator mapping, controller functions, and motor overload contract]
- [Source: specs/architecture/agents/roles.md, benchmark plan reviewer and benchmark reviewer responsibilities for declared motion]
- [Source: specs/architecture/agents/tools.md, benchmark moving-fixture motion contract and fail-closed submission gate]
- [Source: specs/architecture/evals-and-gates.md, powered-motion eval expectations and fail-closed policies]
- [Source: specs/integration-tests.md, INT-022, INT-037, INT-038, and INT-101]
- [Source: worker_heavy/simulation/builder.py, current actuator construction path]
- [Source: worker_heavy/simulation/loop.py, runtime control application and overload checks]
- [Source: worker_heavy/simulation/evaluator.py, overload and failure classification logic]
- [Source: worker_heavy/utils/controllers/time_based.py, time-based controller helpers]
- [Source: worker_heavy/utils/controllers/position_based.py, position-based controller helpers]
- \[Source: shared/enums.py, `MotorControlMode` and moving-part enums\]
- [Source: shared/models/schemas.py, motion and moving-part schema definitions]
- [Source: tests/integration/architecture_p0/test_architecture_p0.py, motor-overload baseline coverage]
- [Source: tests/integration/architecture_p1/test_infrastructure.py, joint-to-actuator and controller-function coverage]
- [Source: tests/integration/architecture_p1/test_benchmark_workflow.py, benchmark workflow motion-contract rejection coverage]

## Dev Agent Record

### Agent Model Used

TBD

### Debug Log References

- Story created from Epic 8 baseline requirements and the current actuator/runtime implementation surface.

### Completion Notes List

- Pending implementation.

### File List

- `_bmad-output/implementation-artifacts/8-1-model-actuator-driven-motion.md`
- `_bmad-output/implementation-artifacts/sprint-status.yaml`

### Change Log

- 2026-03-24: Created the story context for powered actuator-driven motion.
