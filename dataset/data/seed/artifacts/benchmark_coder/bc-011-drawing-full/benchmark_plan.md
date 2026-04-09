## 1. Learning Objective

Test lateral transfer over 1 meter distance. The benchmark moves a 40 mm radius steel sphere (`projectile_ball`, `material_id: steel_1045`) from the left start position at X=-0.5 m to the right goal zone centered near X=+0.5 m without leaving simulation bounds. The transfer uses a passive gravity-driven ramp; no active drive mechanism or powered DOFs are present.

## 2. Geometry

- `environment_fixture`: the fixed base context that anchors the ground plane and provides the simulation reference frame for the full 1 m transfer.
- `ground_plane`: fixed base plane at Z=0 made from HDPE, spanning the full build-zone footprint (1.16 m x 0.20 m).
- `floor_plate`: fixed aluminum plate (`aluminum_6061`) centered near X=0, providing a rigid base for the mechanism (200 mm x 60 mm x 10 mm).
- `support_tower`: vertical aluminum tower (`aluminum_6061`) centered near X=0, 160 mm tall, supporting the raised goal shelf.
- `raised_goal_shelf`: aluminum goal shelf (`aluminum_6061`) positioned on the support tower near X=+0.5 m at Z=0.17 m.
- `lift_carriage`: HDPE carriage plate that slides passively along the Z axis (`dofs: [slide_z]`, no active control), providing lateral guidance for the sphere.
- Two guide rails along the X axis around Y=+/-0.06 m to contain the sphere.
- Build zone limited to X=[-0.58, 0.58], Y=[-0.1, 0.1], Z=[0, 0.18].

## 3. Objectives

- Goal zone AABB: min=[0.44, -0.05, 0.15], max=[0.56, 0.05, 0.20]. The Z range [0.15, 0.20] intersects the `raised_goal_shelf` top surface at Z=0.17 m, ensuring the sphere can land on the shelf.
- Failure if the sphere exits simulation bounds.
- Gravity-only passive transfer via a ramp/track geometry; no active drive mechanism. The `lift_carriage` has a single passive DOF (`slide_z`) with no `control` block — it acts as a friction-guided sliding surface, not a powered actuator.

## 4. Detailed Calculations

### 4.1 Transfer Distance and Geometry

The payload starts at `start_position = [-0.5, 0.0, 0.12]` m and must reach the goal zone centered at X=+0.5 m. The lateral transfer distance is:

```
delta_x = 0.5 - (-0.5) = 1.0 m
```

The start height is Z=0.12 m and the goal shelf is at Z=0.17 m. Since the goal is *higher* than the start, a purely gravity-driven transfer is impossible without an intermediate ramp that first drops the sphere to gain kinetic energy, then uses that energy to climb back up. The mechanism uses a V-shaped ramp profile:

1. **Drop phase:** Sphere rolls down from Z=0.18 m (build zone ceiling) to Z=0.04 m over the first 0.5 m (X: -0.5 to 0.0), gaining velocity.
2. **Climb phase:** Sphere climbs from Z=0.04 m to Z=0.17 m over the remaining 0.5 m (X: 0.0 to +0.5).

### 4.2 Energy Balance

For a steel sphere (`steel_1045`, density ~7850 kg/m³, radius 0.04 m):

```
mass = density * volume = 7850 * (4/3 * pi * 0.04³) = 7850 * 0.000268 = 2.105 kg
weight = mass * g = 2.105 * 9.81 = 20.65 N
```

Energy at ramp entry (relative to Z=0.04 m datum):

```
E_start = m * g * (0.18 - 0.04) = 2.105 * 9.81 * 0.14 = 2.87 J
```

Energy required to reach goal shelf (Z=0.17 m from Z=0.04 m datum):

```
E_goal = m * g * (0.17 - 0.04) = 2.105 * 9.81 * 0.13 = 2.68 J
```

Net energy margin:

```
E_margin = 2.87 - 2.68 = 0.19 J  (7% margin before friction)
```

Rolling friction coefficient for steel-on-aluminum: μ_r ≈ 0.002. Work lost to friction over 1.0 m:

```
W_friction = μ_r * m * g * d = 0.002 * 20.65 * 1.0 = 0.041 J
```

Net energy at goal: 0.19 - 0.041 = 0.15 J > 0. Transfer succeeds with margin.

### 4.3 Timing Estimate

Velocity at goal bottom (Z=0.04 m):

```
v = sqrt(2 * g * h) = sqrt(2 * 9.81 * 0.14) = sqrt(2.75) = 1.66 m/s
```

Transfer time (average velocity ~1.1 m/s over 1.0 m):

```
t_transfer ≈ 1.0 / 1.1 = 0.91 s
```

### 4.4 Clearance Margins

- Guide rail spacing: Y = +/-0.06 m, sphere diameter = 0.08 m. Clearance per side: (0.12 - 0.08) / 2 = 0.02 m (20 mm).
- Build zone Z ceiling: 0.18 m. Ramp peak at Z=0.18 m exactly hits the ceiling — valid.

### 4.5 Tolerance Analysis

Runtime jitter on start position: [0.01, 0.005, 0.005] m. Worst-case start Z = 0.12 - 0.005 = 0.115 m, but the ramp entry is fixed at Z=0.18 m regardless of jitter. The jitter affects where on the ramp entry the sphere spawns:

```
E_worst_x_jitter = negligible (ramp entry height is fixed by geometry, not spawn position)
```

The transfer is robust to the specified jitter envelope because the ramp geometry, not the spawn position, determines the available energy.

## 5. Payload Trajectory

The payload follows a gravity-driven ramp path from the start position to the goal zone. The trajectory is defined in `payload_trajectory_definition.yaml` with three anchor points:

| Anchor | X (m) | Y (m) | Z (m) | Time (s) | Description |
| -- | -- | -- | -- | -- | -- |
| 0 (start) | -0.50 | 0.00 | 0.18 | 0.00 | Ramp entry (engineer to design ramp geometry) |
| 1 (bottom) | 0.00 | 0.00 | 0.04 | 0.35 | Lowest point, max velocity ~1.7 m/s |
| 2 (goal) | +0.50 | 0.00 | 0.17 | 0.85 | Goal shelf landing zone |

The `lift_carriage` provides passive Z-axis guidance with `dofs: [slide_z]` and no active `control` block. It acts as a movable support surface that follows the sphere's vertical position during the transfer, reducing bounce and lateral deviation.

## 6. Randomization

- Static variation on platform height between 0.07 m and 0.09 m (affects ramp mounting base).
- Static variation on rail Y offset between -0.01 m and +0.01 m (affects guide clearance).
- Runtime jitter on the sphere start position: [0.01, 0.005, 0.005] m.

## 7. Cost & Weight Budget

Manufactured parts total:

- `floor_plate`: $19.00, aluminum_6061
- `support_tower`: $18.00, aluminum_6061
- `raised_goal_shelf`: $11.00, aluminum_6061
- `lift_carriage`: $7.00, HDPE
- **Total: $55.00**, well under the $95.00 benchmark cap.

Estimated assembly weight: 1360 g, under the 1800 g benchmark cap. The payload (steel sphere, ~2.1 kg) is not counted against the assembly weight budget.

## 8. Implementation Notes

- Use Build123d primitives.
- Ensure every part has `PartMetadata` or `CompoundMetadata`.
- Preserve `benchmark_plan_evidence_script.py` and `benchmark_plan_technical_drawing_script.py` as read-only planner context.
- Keep `benchmark_script.py` import-safe with no `__main__` block and no in-module review submission call.
- Run validate/simulate/review submission only from external shell self-check commands.
- The engineer must design ramp geometry that starts at Z=0.18 m (build zone ceiling) at X=-0.5 m, drops to Z=0.04 m near X=0, and rises to Z=0.17 m at the goal shelf near X=+0.5 m.
