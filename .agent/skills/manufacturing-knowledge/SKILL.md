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