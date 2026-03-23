---
name: manufacturing-knowledge
description: Technical specifications, material properties, and cost models for CNC Milling and Injection Molding. Use this when the task specifies a 'max_unit_cost' or 'target_quantity', or when planning for specific manufacturing processes.
---

# Manufacturing & Economic Knowledge

This skill provides the procedural and domain knowledge required to design parts that meet specific manufacturing constraints and economic targets.

## 1. Dynamic Data Access

To get the current material properties, density, and cost constants, use the runtime surfaces that are actually exposed in the workspace. **Do not rely on your internal knowledge or hardcoded values in this file.**

Use:

- `read_file("/config/manufacturing_config.yaml")` for material properties, process costs, and catalog data.
- Planner roles: `validate_costing_and_price()` to validate and price `assembly_definition.yaml`.

Do not browse `/scripts` or invent helper tools that are not exposed to your role.

## 2. CNC Milling (3-Axis)

**Best for**: Low volumes (1-100 units), high strength, aluminum parts.
**Detailed Reference**: [references/cnc.md](references/cnc.md)

### Cost Formula

$$Total = Setup + (Material + Run) \\times Quantity$$

- **Setup Cost**: Fixed cost (~$80.00). 50% discount on reuse.
- **Material Cost**: Based on **Stock Bounding Box** volume.
- **Run Cost**: Machining time based on **Removed Volume** ($Stock - Part$).

### Design Constraints

- **Undercuts**: Forbidden. All faces must be accessible from +Z.
- **Internal Corners**: Minimum tool radius is **3.0mm**. Use `fillet(radius=3.1)`.

______________________________________________________________________

## 3. Injection Molding (IM)

**Best for**: High volumes (>1,000 units), plastic parts, low unit cost.
**Detailed Reference**: [references/injection_molding.md](references/injection_molding.md)

### Cost Formula

$$Total = Tooling + (Material + Cycle) \\times Quantity$$

- **Tooling Cost**: High fixed cost. Amortized over volume. 90% discount on mold reuse.
- **Material Cost**: Based on actual part volume.
- **Cycle Cost**: Driven by **Cooling Time** ($CoolingTime \\propto Thickness^2$).

### Design Constraints

- **Draft Angles**: Mandatory for vertical faces. Minimum **2.0 degrees**.
- **Wall Thickness**: Keep between **1.0mm and 4.0mm**. Thick walls exponentially increase cycle cost.
- **Undercuts**: Faces must be reachable from either +Z or -Z (2-part mold).

______________________________________________________________________

## 4. Economic Strategy

- **Quantity < 100**: Prefer **CNC** or **3D Printing**.
- **Quantity > 1000**: Always prefer **Injection Molding** if geometry allows.
- **Method comparison**: Compare candidate methods at the requested quantity, not as single-unit quotes. Use setup/tooling cost separately from per-unit variable cost so the chosen process reflects the real production volume.
- **Volume Optimization**: Reducing part volume directly reduces material cost and run/cycle time.
- **Part Reuse**: Using multiple instances of the *same* part ID is significantly cheaper than multiple unique parts due to shared setup/tooling costs (50% discount for CNC setup, 90% discount for IM tooling).

## 5. Technical Design Patterns

### Pattern: CNC Fillet Strategy

```python
# Expert Pattern: Automatic filleting of internal vertical edges
internal_edges = part.edges().filter_by(Axis.Z).internal()
part = fillet(internal_edges, radius=3.1) # 3.1mm for 3.0mm tool clearance
```

### Pattern: Injection Molding Shelling

```python
# Expert Pattern: Creating a shelled plastic part
part = shell(part, openings=part.faces().sort_by(Axis.Z).last(), amount=-2.0)
```

### Pattern: Draft for Release

```python
# Expert Pattern: Applying 2-degree draft
part = draft(part, faces=part.faces().filter_by(Axis.Z), angle=2.0, pull_direction=(0,0,1))
```
