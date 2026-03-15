## 1. Learning Objective

Test whether an engineer can receive a ball near floor height and lift it onto a raised shelf while respecting a fixed support-clearance keep-out and benchmark-owned lift motion.

## 2. Environment Geometry

- `floor_plate`: box centered at `[20, 0, 10]` with size `[560, 180, 20]`.
- `support_tower`: box centered at `[260, 0, 105]` with size `[140, 120, 210]`.
- `raised_goal_shelf`: box centered at `[365, 0, 235]` with size `[170, 120, 30]`; the front shelf lip projects forward to maximize capture area but leaves only a narrow open pocket behind the tower face.
- `lift_carriage`: plate centered nominally at `[-130, 0, 60]` with size `[120, 110, 18]`; benchmark-side motion is one `slide_z` axis to hand the ball up toward the shelf.

## 3. Input Objective

- Shape: `sphere`
- Label: `projectile_ball`
- Static randomization:
  - radius in `[44, 46]` mm
- Nominal start position: `[-200, 0, 60]`
- Runtime jitter: `[10, 8, 6]` mm

## 4. Objectives

- `goal_zone`: min `[320, -55, 220]`, max `[430, 55, 300]`
- `forbid_zones`:
  - `shelf_support_clearance`: min `[180, -95, 0]`, max `[280, 95, 210]`
- `build_zone`: min `[-260, -180, 0]`, max `[470, 180, 340]`

## 5. Simulation Bounds

- min `[-320, -220, -10]`, max `[520, 220, 380]`

## 6. Constraints Handed To Engineering

- Benchmark/customer caps: `max_unit_cost <= 95 USD`, `max_weight <= 1800 g`
- Benchmark-owned motion must stay limited to the single lift carriage axis; engineering should not assume any other powered benchmark fixture.

## 7. Success Criteria

- Success if the moved object's center enters `goal_zone` without entering `shelf_support_clearance`, even though the tower face and shelf lip now sit close to the left side of that capture region.
- Fail if the moved object leaves `simulation_bounds` or requires any undeclared benchmark-side motion.

## 8. Planner Artifacts

- `todo.md` captures the benchmark implementation checklist.
- `benchmark_definition.yaml` mirrors the benchmark-owned geometry, objectives, and caps.
- `benchmark_assembly_definition.yaml` captures benchmark-local part DOFs and cost estimates for the floor, tower, shelf, and lift carriage.
