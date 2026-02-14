# WP3 Quickstart: Electromechanical Integration

## 1. Environment Setup

Ensure `ngspice` is installed on your system.

```bash
# Ubuntu/Debian
sudo apt-get install ngspice libngspice0-dev
```

The system will automatically find `libngspice.so` in `/usr/lib/x86_64-linux-gnu/`.

## 2. Running a Circuit Validation

You can test the circuit logic independently of the physics simulation.

```python
from shared.models.schemas import ElectronicsSection
from shared.circuit_builder import build_circuit_from_section
from shared.pyspice_utils import validate_circuit

# Load your electronics section
section = ElectronicsSection.parse_file("path/to/electronics.json")

# Build and validate
circuit = build_circuit_from_section(section)
result = validate_circuit(circuit, section.power_supply)

print(f"Is valid: {result.valid}")
print(f"Total current: {result.total_draw_a}A")
```

## 3. Wire Routing Validation

Test wire clearance against mechanical parts.

```python
from build123d import Box
from shared.wire_utils import check_wire_clearance

# Create a sample assembly
box = Box(10, 10, 10)
assembly = box

# Define a path that goes through the box
wire_path = [(0, 0, -10), (0, 0, 10)]

is_clear = check_wire_clearance(wire_path, assembly, clearance_mm=1.0)
print(f"Wire is clear: {is_clear}") # Should be False
```
