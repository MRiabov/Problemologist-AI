## 1. Learning Objective

Test whether benchmark-owned motorized lift hardware can raise a ball from floor height onto a shelf while keeping the shelf-support keep-out clear and preserving the class-aware COTS import path. The engineer must reason about powered fixture motion, COTS motor torque limits, and clearance geometry.

## 2. Geometry

All dimensions in mm. Positions are world-frame centers unless noted.

### `environment_fixture`

The fixed simulation context that anchors all benchmark fixtures. Provides the reference frame for the lift mechanism, floor plate, support tower, and raised goal shelf. Not engineer-modifiable.

### `floor_plate` (fixed, aluminum_6061)

- Center: `(20, 0, 10)`
- Size: `560 × 180 × 20`
- Ground plane for the entire benchmark. Non-drillable by default.

### `support_tower` (fixed, aluminum_6061)

- Center: `(230, 0, 105)`
- Size: `80 × 120 × 210`
- Vertical structural column between floor and shelf. Defines the shelf-support clearance keep-out zone.

### `raised_goal_shelf` (fixed, aluminum_6061)

- Center: `(375, 0, 235)`
- Size: `150 × 120 × 30`
- Raised target platform where the projectile ball must end up.

### `lift_carriage` (moving, hdpe, single `slide_z` DOF)

- Center: `(-130, 0, 60)` at rest
- Size: `120 × 110 × 18`
- Moves vertically along the z-axis driven by the benchmark-owned servo. Travel range: z ≈ 10 mm (rest) to z ≈ 220 mm (shelf height).

### `drive_motor` (fixed, COTS `ServoMotor_DS3218`)

- Mounting center: `(-130, 0, 10)`
- Catalog dimensions from `ServoMotor_DS3218`: `40 × 20 × 40.5` mm
- Benchmark-owned fixture. Imported through `ServoMotor.from_catalog_id("ServoMotor_DS3218")`.
- Control mode: `ON_OFF`, speed `1.0` (units/s for slide_z actuation).
- Engineer must not modify or reposition this fixture.

### `projectile_ball` (moved object, ABS plastic)

- Shape: `sphere`
- Radius range (static randomization): `[44, 46]` mm
- Spawn position: `(-200, 0, 60)`
- Runtime jitter: `±[10, 8, 6]` mm
- Material: `abs`

## 3. Objectives

### Goal zone

- AABB min: `(320, -55, 220)`
- AABB max: `(430, 55, 300)`
- Success when the projectile ball center enters this volume.

### Forbid zone: `shelf_support_clearance`

- AABB min: `(180, -95, 0)`
- AABB max: `(280, 95, 210)`
- Any contact with this zone by any simulation object = failure.

### Build zone

- AABB min: `(-260, -180, 0)`
- AABB max: `(470, 180, 340)`
- The engineer may only build within these bounds.

### Simulation bounds

- AABB min: `(-320, -220, -10)`
- AABB max: `(520, 220, 380)`
- Any object exiting this volume = failure.

## 4. Benchmark-Side Motion Contract

Only one moving benchmark fixture exists:

| Fixture | DOF | Motion kind | Axis | Travel range | Control | Engineer interaction |
| -- | -- | -- | -- | -- | -- | -- |
| `lift_carriage` | `slide_z` | motorized_prismatic | +Z | z ≈ 10 → 220 mm | `ON_OFF`, speed 1.0 | Engineer may rely on this motion |

The `drive_motor` (`ServoMotor_DS3218`) actuates the lift carriage. The motor is benchmark-owned, fixed, and not engineer-interactable (`allows_engineer_interaction: false`). The engineer may use the carriage motion as environmental context but may not redefine it.

## 5. Electronics and COTS Fixture

- One benchmark-owned servo: `ServoMotor_DS3218` (pololu, $18.00).
- Power supply: 24 V DC mains-rectified, max 10 A.
- Max total wire length: 850 mm.
- Wire restricted zones mirror the `shelf_support_clearance` forbid zone.
- Circuit validation is required.
- The motor and wiring are benchmark fixture context, not engineer-owned electrical design.

## 4. Randomization

- Static: projectile ball radius varies in `[44, 46]` mm per benchmark variant.
- Runtime: projectile ball spawn position jitters by `±[10, 8, 6]` mm per simulation run.
- The solution must handle all positions within the jitter range robustly.

## 7. Constraints Handed to Engineering

- Planner estimate: `$63.33` / `1200 g`
- Runtime-derived caps: `$95.00` / `1800 g`
- Physics backend: `GENESIS`
- FEM: disabled (rigid-body only)

## 8. Planner Artifacts

- `todo.md` — implementation checklist
- `benchmark_definition.yaml` — objective zones, randomization, constraints, electronics
- `benchmark_assembly_definition.yaml` — fixture structure, DOFs, COTS metadata, drafting spec
- `benchmark_plan_evidence_script.py` — build123d evidence matching this inventory
- `benchmark_plan_technical_drawing_script.py` — orthographic drawing of the lift carriage
- `submit_plan()` — explicit submission gate

## 5. Implementation Notes

- Use build123d primitives for all geometry.
- Instantiate the drive motor via `ServoMotor.from_catalog_id("ServoMotor_DS3218")`.
- Preserve `benchmark_plan_evidence_script.py` and `benchmark_plan_technical_drawing_script.py` as read-only planner context.
- Keep `benchmark_script.py` import-safe: no `__main__` block, no in-module review submission call.
- Run validate/simulate/review submission only from external shell self-check commands.
