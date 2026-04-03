# Electromechanical Circuit Patterns

## Terminal Mapping

- Motor terminals: `+`, `-` and legacy `a`, `b` normalize to the same nodes.
- Switch and relay terminals: `in`, `out`
- Main PSU nodes: `supply_v+`, `0`

## Direct-Powered Motor

Use when the motor should always be energized.

- `supply_v+ -> motor.+`
- `0 -> motor.-`

## Switched Motor

Use when a relay or switch gates motor power.

- `supply_v+ -> switch.in`
- `switch.out -> motor.+`
- `0 -> motor.-`

Review checks:

- switch path is actually in series with the motor
- motor still has a return path to `0`

## Parallel Loads On One Supply

Use when multiple loads share one PSU.

- branch each positive feed intentionally
- tie returns back to the same ground reference
- compare total draw against `power_supply.max_current_a`

Do not assume each branch is safe in isolation; the PSU gate is on total current.

## Secondary Supply Component

If a design needs a separate battery or secondary source, model it as a `POWER_SUPPLY` component with its own component ID and voltage.

## Failure Patterns

Reject these early:

- motor positive connected but no negative return
- component terminals present in `components` but never referenced by `wiring`
- PSU sized below the sum of realistic load demand
- wire gauge too small for expected current
