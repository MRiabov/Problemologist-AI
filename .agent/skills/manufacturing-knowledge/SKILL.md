---
name: manufacturing-knowledge
description: Technical specifications, material properties, and cost models for CNC Milling and Injection Molding. Use this when the task specifies a 'max_unit_cost' or 'target_quantity', or when planning for specific manufacturing processes.
---

# Manufacturing & Economic Knowledge

This skill provides the procedural and domain knowledge required to design parts that meet specific manufacturing constraints and economic targets.

## 1. Dynamic Data Access

To get the most up-to-date material properties, density, and cost constants, you MUST run the provided data script.

**Action**: Use `run_skill_script(skill_name="manufacturing-knowledge", script_name="get_material_data.py")`

This will return a JSON object containing:
- CNC material properties (Aluminum 6061)
- Injection Molding material properties (ABS)
- Machine hourly rates
- Tooling/Setup costs

## 2. Design Constraints (Summary)

### CNC Milling (3-Axis)
- **Undercuts**: Strictly forbidden from the +Z approach.
- **Internal Corners**: Must be filleted (Minimum tool radius is usually 3mm).
- **Process**: Subtractive. Use shared setups (identical parts) to reduce cost.

### Injection Molding (IM)
- **Draft Angles**: Mandatory for vertical faces (Minimum 2.0 degrees).
- **Wall Thickness**: Keep uniform (1.0mm - 4.0mm) to prevent defects.
- **Undercuts**: Forbidden in simple 2-part molds.

## 3. Economic Strategy

- **Setup vs. Unit Cost**: CNC has low setup but high unit cost. IM has high setup (tooling) but very low unit cost.
- **Break-even**: High quantities (>1000) strongly favor Injection Molding.
- **Volume**: Reducing part volume is the most effective way to lower cost in both processes.
- **Reuse**: Reusing the same part multiple times reduces total setup/tooling overhead.

## 4. Technical Design Patterns

### Pattern: CNC Fillet Strategy
When designing for CNC, internal vertical corners must have a radius $\ge$ tool radius.
```python
# Expert Pattern: Automatic filleting of internal vertical edges
internal_edges = part.edges().filter_by(Axis.Z).internal()
part = fillet(internal_edges, radius=3.1) # 3.1mm for 3.0mm tool clearance
```

### Pattern: Injection Molding Shelling
To maintain uniform wall thickness (1.0 - 4.0mm):
```python
# Expert Pattern: Creating a shelled plastic part
part = shell(part, openings=part.faces().sort_by(Axis.Z).last(), amount=-2.0)
```

### Pattern: Draft for Release
All faces parallel to the pull direction (+Z) must be tapered.
```python
# Expert Pattern: Applying 2-degree draft
part = draft(part, faces=part.faces().filter_by(Axis.Z), angle=2.0, pull_direction=(0,0,1))
```

## 5. Optimization Workflow

1. **Query Database**: Call `run_skill_script` to get current rates.
2. **Select Process**: Based on `target_quantity` (IM for >1000, CNC for <100).
3. **Model Base Geometry**: Focus on function first.
4. **Apply DFM**: Add fillets (CNC) or Shell/Draft (IM).
5. **Analyze Cost**: Call `check_manufacturability` to see the estimate.
6. **Iterate**: If `unit_cost > max_unit_cost`, reduce volume or simplify geometry.