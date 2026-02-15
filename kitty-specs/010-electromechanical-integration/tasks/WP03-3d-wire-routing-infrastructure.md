---
work_package_id: WP03
title: 3D Wire Routing Infrastructure
lane: "doing"
dependencies: []
base_branch: main
base_commit: 2e6528e1ea3012b40a0b47a1dd5180fa8771bc02
created_at: '2026-02-15T08:51:59.988902+00:00'
subtasks: [T010, T011, T012, T013]
shell_pid: "46810"
agent: "gemini-cli"
---

# WP03: 3D Wire Routing Infrastructure

## Objective
Implement the logic for defining, validating, and calculating properties of physical wires in 3D space.

## Context
Wires in our system are more than just connections; they are 3D splines that must not pass through solid objects. They have physical properties like gauge (determining resistance and thickness) and tensile strength.

## Detailed Guidance

### T010: Implement `shared/wire_utils.py` (Splines)
**Purpose**: Generate 3D paths from waypoints.

**Steps**:
1. Use `scipy.interpolate` or `build123d` spline functions to create a smooth path through waypoints.
2. Implement a method to sample points along the spline for collision checking.

### T011: Implement `check_wire_clearance`
**Purpose**: Prevent wires from intersecting solid parts.

**Steps**:
1. Take the sampled spline points and the assembly's `Compound` geometry.
2. Use `build123d`'s `intersect` or `trimesh`'s proximity queries to detect penetrations.
3. Allow for small intersections at "attachment points" (start/end nodes).

### T012: Implement `route_wire` helper
**Purpose**: Provide a clean API for the agent to define wiring.

**Steps**:
1. Create a function that takes `wire_id`, `waypoints`, and `gauge`.
2. It should return a `WireConfig` object.
3. Include automatic length calculation from the spline.

### T013: Add wire gauge property lookup
**Purpose**: Map AWG to physical properties.

**Steps**:
1. Create a lookup table for standard AWG sizes (e.g., 14, 16, 18, 22).
2. Store: `resistance_per_meter`, `max_current_a`, `tensile_strength_n`, `diameter_mm`.

## Definition of Done
- `route_wire` returns correct lengths.
- `check_wire_clearance` correctly rejects wires passing through solids.
- AWG lookup provides accurate industrial values.

## Risks
- Collision checking being too slow for many wires.
- Spline interpolation creating paths that are physically impossible (too tight curves).

## Activity Log

- 2026-02-15T08:52:00Z – gemini-cli – shell_pid=46810 – lane=doing – Assigned agent via workflow command
