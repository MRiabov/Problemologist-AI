# Engineering Plan

## 1. Solution Overview

Use a single pivoting diverter gate to switch the projectile ball from the neutral chute into the goal chute. The gate is driven by one 12V DC gearmotor mounted on the left wall so the electrical system can stay outside the moving envelope of the gate.

## 2. Parts List

- **diverter_gate**: HDPE gate blade that rotates about Z to redirect the ball.
- **gate_bracket**: CNC aluminum bracket that mounts the motor and pivot support.
- **mount_base**: Fixed base plate that locates the bracket inside the build zone.
- **gate_motor**: 12V DC gearmotor that rotates the diverter gate.

## 3. Assembly Strategy

1. Bolt the `gate_bracket` to `mount_base` and keep both fixed.
2. Couple `gate_motor` output to `diverter_gate` through the left-wall mount.
3. Route electrical wiring through a left-wall cable corridor so no wire crosses the gate swing volume.

## 4. Cost & Weight Budget

- Benchmark caps: `benchmark_max_unit_cost_usd <= 65 USD`, `benchmark_max_weight_g <= 1200 g`
- Planner target: `planner_target_max_unit_cost_usd <= 52 USD`, `planner_target_max_weight_g <= 900 g`
- Keep the electrical additions within the existing planner target.

## 5. Risk Assessment

- Wire snagging across the gate swing path would invalidate the design.
- Excessive wire length can violate the wiring constraint and add slack-driven failure.
- Overcurrent must stay below the available 12V / 2.0A supply rating.
