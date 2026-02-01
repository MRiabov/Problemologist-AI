# Quickstart: Advanced Manufacturing Workbenches

## 1. Installation
Ensure dependencies are installed:
```bash
uv sync  # installs trimesh
```

## 2. Running a Manual Check

You can use the python API directly to test the workbenches.

```python
from src.workbenches.cnc import CNCWorkbench
from src.workbenches.injection_molding import InjectionMoldingWorkbench
from build123d import Box, Cylinder

# Create a test part (simple cylinder is friendly for both)
part = Cylinder(radius=10, height=20)

# 1. Check CNC Manufacturability
cnc = CNCWorkbench()
report_cnc = cnc.analyze(part, quantity=1)
print(f"CNC Status: {report_cnc['status']}")
print(f"CNC Cost: ${report_cnc['cost_analysis']['total_cost']}")

# 2. Check Injection Molding
im = InjectionMoldingWorkbench()
report_im = im.analyze(part, quantity=10000)
print(f"IM Status: {report_im['status']}")
print(f"IM Unit Cost: ${report_im['cost_analysis']['unit_cost']}")
```

## 3. Configuration
Modify `src/workbenches/manufacturing_config.yaml` to adjust material costs or machine rates.

```yaml
cnc:
  materials:
    aluminum_6061:
      cost_per_kg: 6.00  # Updated price
```
