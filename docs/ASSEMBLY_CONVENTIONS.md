# Assembly & Physics Conventions

This document defines the contract between the CAD geometry (Python/`build123d`) and the Physics Engine (MuJoCo/MJCF). Following these conventions ensures that your static 3D models are correctly translated into functional, simulated mechanisms.

## 1. The "Magic Label" System (Implicit Joints)

The `SceneCompiler` automatically detects specific strings in the `.label` attribute of `build123d` objects to inject physics components.

### 1.1. Motors
To define a motor without writing XML, group your motor parts into a `Compound` and label the solids as follows:

| Label | Role | Translation to MuJoCo |
| :--- | :--- | :--- |
| `stator` | The static part of the motor. | Remains in the parent `<body>`. |
| `rotor` | The spinning part of the motor. | Moved to a child `<body>` with a `<joint>`. |

**Behavior:**
- A `hinge` joint is automatically created at the **center of mass** of the `rotor` solid.
- A `<motor>` actuator is automatically added to the `<actuator>` section.
- Gear ratio and damping are pulled from `src/assets/cots_descriptions.json` if the part ID matches (e.g., "Nema17"), otherwise defaults are used.

**Example:**
```python
motor = StepperMotor("Nema17")
# The provider usually does this for you:
motor.solids()[0].label = "stator"
motor.solids()[1].label = "rotor"
```

---

## 2. Explicit Joint Definitions

For complex mechanisms or joints that aren't motors (e.g., slides, free-spinning bearings), use the `agent_joints` parameter in the `compile()` method.

### 2.1. Joint Specification Object
A joint is defined as a dictionary:

```python
{
    "name": "my_joint",
    "type": "hinge",    # 'hinge', 'slide', 'ball', 'free'
    "pos": "0 0 0",    # Position in local coordinates
    "axis": "0 0 1",   # Rotation/Slide axis
    "damping": "0.1"   # Optional
}
```

### 2.2. Part Mapping logic
When `agent_joints` are provided, the `SceneCompiler` maps parts based on their index in the `agent_compound`:
- **Index 0**: Becomes the "Base" (Parent of Link 0).
- **Index i+1**: Becomes "Link i" (Child of Joint i).

---

## 3. Working with Bearings

Bearings do not currently have a "Magic Label" like motors. To make a bearing functional in simulation:

### 3.1. Option A: Rigid (Static)
If the bearing is just for visual/fit checking, do nothing. All solids will be fused into a single rigid body.

### 3.2. Option B: Functional (Free-Spinning)
To simulate a shaft spinning inside a bearing:
1.  **Separate the solids**: Ensure the inner ring and outer ring are separate solids in your `Compound`.
2.  **Labeling**: 
    - Label the outer ring (and housing) as `part_0`.
    - Label the inner ring (and shaft) as `part_1`.
3.  **Explicit Joint**: Pass a `hinge` joint at the bearing's center to the compiler.

---

## 4. Zone Conventions (Environment)

Special volumes in the environment that trigger logic instead of collisions:

| Name Pattern | Behavior |
| :--- | :--- |
| `zone_goal` | Transparent green. Success trigger when the target object enters. |
| `zone_forbid`| Transparent red. Failure trigger if touched by ANY part. |
| `zone_start` | Spawn point marker. |
| `obstacle_*` | Standard gray solid with collisions. |

---

## 5. Troubleshooting & Limitations

- **Case Sensitivity**: Labels are currently case-sensitive or converted to lowercase in some parts of the code. **Always use lowercase** (`stator`, `rotor`) to be safe.
- **Convexity**: MuJoCo works best with convex geoms. The compiler uses the convex hull of your meshes for collisions. If you have a U-shaped part, the opening will be "filled" for collision purposes.
- **Welding**: Any solids in the same `body` are effectively welded. To "unweld" them, they MUST be in different bodies connected by a joint.
