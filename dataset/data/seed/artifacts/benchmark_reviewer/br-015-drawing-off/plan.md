## 1. Learning Objective

Review a fast-transfer benchmark package that keeps a single driven roller on
paper while trying to hide a second benchmark-side brake motion in the final
assembly and review artifacts.

## 2. Geometry

- `launch_lane`: static launch lane.
- `accelerator_roller`: single driven roller with the only justified
  benchmark-side motion.
- `speed_bump_guard`: central keep-out guard.
- `goal_brake_tray`: passive-looking brake tray that should not introduce a
  second benchmark-side axis.

## 3. Objectives

- Verify the goal zone is still reachable through the fast-transfer lane.
- Reject any hidden second benchmark-side axis beyond the driven roller.
- Keep the speed-bump keep-out and wiring corridor coherent.

## 4. Review Artifacts

- `benchmark_script.py`, `validation_results.json`, and
  `simulation_result.json` must agree on the latest benchmark revision.
- `benchmark_review_manifest.json` should remain consistent with the current
  render bundle if rendered evidence is present.

## 5. Implementation Notes

- Use Build123d primitives only.
- Keep the benchmark review package import-safe and deterministic.
- Reject the package if the hidden brake axis is not explicitly accounted for
  in the benchmark assembly contract.
