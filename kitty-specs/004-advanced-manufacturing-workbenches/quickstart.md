# Quickstart: Advanced Manufacturing Workbenches

## 1. Agent Usage (In Sandbox)

The agent interacts with the workbenches via the `validate_and_price` utility imported from `utils`.

```python
from build123d import Box
from utils import validate_and_price, ManufacturingMethod

# 1. Create part
part = Box(10, 10, 10)
part.label = "bracket"
part.metadata = {
    "manufacturing_method": ManufacturingMethod.CNC,
    "material": "aluminum-6061"
}

# 2. Validate and get price (prints results to stdout for the agent)
validate_and_price(part)
```

## 2. Testing Logic (Direct API)

For internal testing on the worker node:

```python
from src.workbenches.cnc import CNCWorkbench
from build123d import Box

cnc = CNCWorkbench()
report = cnc.analyze(Box(10, 10, 10))

print(f"Status: {report.status}")
print(f"Unit Cost: {report.cost.unit_price}")
```

## 3. Configuration

Materials and machine rates are configured in `src/workbenches/manufacturing_config.yaml`.
