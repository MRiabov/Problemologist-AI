# Benchmark Journal — bc-012-raised-shelf-motor

## Session Summary

Seeded planner handoff for a motorized raised-shelf benchmark where a
benchmark-owned `ServoMotor_DS3218` drives a `lift_carriage` along a single
`slide_z` axis to raise a projectile ball from floor height onto a raised
goal shelf.

## Key Context

- **Learning objective**: Test powered fixture motion with COTS motor import,
  clearance-zone avoidance, and class-aware catalog resolution.
- **Payload**: ABS sphere, radius 44–46 mm, spawns at `(-200, 0, 60)`
  with `±[10, 8, 6]` mm runtime jitter.
- **Goal zone**: `(320, -55, 220)` to `(430, 55, 300)`.
- **Forbid zone**: shelf-support clearance `(180, -95, 0)` to `(280, 95, 210)`.
- **Build zone**: `(-260, -180, 0)` to `(470, 180, 340)`.
- **Simulation bounds**: `(-320, -220, -10)` to `(520, 220, 380)`.
- **Benchmark motion**: `lift_carriage` — single `slide_z`, motorized,
  z ≈ 10 → 220 mm, `ON_OFF` control at speed 1.0.
- **COTS motor**: `ServoMotor_DS3218` (pololu, $18.00, 1.96 N·m torque).
- **Electronics**: 24 V DC supply, max 10 A, wire routing restricted from
  the shelf-support clearance zone.

## Implementation Approach

1. Reconstruct all five benchmark fixtures from the planner handoff:
   `floor_plate`, `support_tower`, `raised_goal_shelf`, `lift_carriage`,
   and `drive_motor`.
2. Instantiate the drive motor through the catalog path:
   `ServoMotor.from_catalog_id("ServoMotor_DS3218")`.
3. Build the payload from `benchmark_definition.yaml.payload`.
4. Reconstruct objective zones through `objectives_geometry()`.
5. Validate geometry, simulate, and submit for review via external shell
   self-checks (not in-module calls).

## Drafting Context

- `benchmark_plan_evidence_script.py` — preview compound matching the full
  planner inventory (floor_plate, support_tower, raised_goal_shelf,
  lift_carriage, drive_motor).
- `benchmark_plan_technical_drawing_script.py` — A3 orthographic drawing
  using build123d `TechnicalDrawing` with lift-carriage focus.
