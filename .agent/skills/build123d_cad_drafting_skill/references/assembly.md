# build123d Assembly & MJCF Physics Conventions

This guide defines how to structure `build123d` models for correct translation into MuJoCo physics (MJCF).

## 1. The "Magic Label" System (Implicit Joints)

The physics compiler detects specific strings in the `.label` attribute of solids to automatically inject joints and actuators.

### 1.1. Motor Pattern

To define a motor, group your motor parts into a `Compound` and label the inner solids as follows:

| Label | MuJoCo Translation |
| :--- | :--- |
| `stator` | Remains in the parent `<body>` (static relative to motor). |
| `rotor` | Moved to a child `<body>` connected by a `hinge` joint. |

**Automatic Behavior:**

- A `hinge` joint is placed at the **center of mass** of the `rotor`.
- A `<motor>` actuator is added to the MJCF.
- Constants (gear, damping) are applied from `src/assets/cots_descriptions.json` if the part ID matches (e.g., "Nema17").

**Example:**

```python
with BuildPart() as motor:
    # ... geometry ...
    stator_solid.label = "stator"
    rotor_solid.label = "rotor"
```

## 2. Explicit Joint Control

For mechanisms that aren't motors (e.g., slides, free-hinges), the agent provides a joint list during compilation.

### 2.1. Joint Mapping logic

The compiler maps the `Compound` children to MuJoCo bodies:

- **Child 0**: The "Base" (Parent).
- **Child i+1**: "Link i" (Child of Joint i).

**Joint Object Schema:**

```python
{
    "name": "slide_j",
    "type": "slide",   # 'hinge', 'slide', 'ball', 'free'
    "pos": (0, 0, 0),

    "axis": (0, 0, 1),
    "damping": 0.1     # Optional
}
```

## 3. Component Conventions

### 3.1. Bearings

Bearings are NOT currently automatic. For a functional bearing:

1. Ensure the inner and outer rings are separate solids.
2. Place them in different indices of the parent `Compound`.
3. Provide an explicit `hinge` joint at the bearing's center.

### 3.2. Fasteners

Screws and nuts are typically fused (rigid) unless specifically being simulated for torque testing. Use `part_a += part_b` to fuse them into the same physics body.

## 4. Environment Zones

Labeling volumes in the environment `Compound` triggers special physics logic:

| Label Pattern | Visual | Physics Effect |
| :--- | :--- | :--- |
| `zone_goal` | Green | Success trigger (Target enters). |
| `zone_forbid`| Red | Failure trigger (Any collision). |
| `zone_start` | Blue | Agent spawn point marker. |
| `obstacle_*` | Gray | Standard static collider. |

## 5. Collision Best Practices

- **Convexity**: MuJoCo uses convex hulls for collisions. Avoid deep concavities in a single solid; split them into multiple solids within a `Compound` if collision accuracy is critical.
- **Clearance**: Leave ~0.1mm clearance between "moving" parts in CAD to avoid "explosive" overlaps in physics start.
- **Labels**: Always use **lowercase** for magic labels (`stator`, `rotor`).
- **Welding**: Any solids in the same `body` are effectively welded. To "unweld" them, they MUST be in different bodies connected by a joint.
