# Formula-Driven Modeling Example

Keep all fixed measurements in one source block, then derive every downstream
dimension from those values. If a number comes from a spec sheet, spreadsheet,
or COTS datasheet, treat it as an input, not as something to retype all over the
model.

## Pattern

1. Collect the source values at the top of the file.
2. Compute secondary dimensions with sums and differences.
3. Place parts from datums, joints, or mating faces.
4. Use the derived values everywhere else.

## Example: motor cradle on a base plate

```python
from build123d import Align, BuildPart, Box, Compound, Location

# Source values from the spec sheet or spreadsheet.
motor_w = 42.0
motor_d = 42.0
motor_h = 38.0
wall = 3.0
clearance = 0.5
base_t = 6.0
standoff = 12.0

# Derived dimensions.
pocket_w = motor_w + 2 * clearance
pocket_d = motor_d + 2 * clearance
cradle_w = pocket_w + 2 * wall
cradle_d = pocket_d + 2 * wall
cradle_h = motor_h + wall
plate_w = cradle_w + 2 * standoff
plate_d = cradle_d + 2 * standoff

with BuildPart() as base:
    Box(plate_w, plate_d, base_t, align=(Align.CENTER, Align.CENTER, Align.MIN))

with BuildPart() as cradle:
    Box(cradle_w, cradle_d, cradle_h, align=(Align.CENTER, Align.CENTER, Align.MIN))

base_part = base.part
base_part.label = "base_plate"

# Place the cradle from the base datum, not from a guessed world coordinate.
cradle_part = cradle.part.moved(Location((0, 0, base_t)))
cradle_part.label = "motor_cradle"

assembly = Compound(children=[base_part, cradle_part])
```

## Why This Works

- The only hand-entered dimensions are the source values.
- Any change to the motor or wall thickness propagates through the formulas.
- The cradle pose comes from the base datum, so the placement is traceable.
