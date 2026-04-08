# Electronics API Reference

This is a generated snapshot of the current electronics schema and helper surface.
Refresh it from `shared/models/schemas.py` and `worker_heavy/utils/electronics.py` when fields or signatures change; do not hand-maintain the snapshot from memory.

## Current Schema Surface

`shared/models/schemas.py` currently defines the planner/runtime electronics data model.

### `WireConfig`

Use for one electrical connection plus optional physical routing.

- `wire_id: str`
- `from: WireTerminal`
- `to: WireTerminal`
- `gauge_awg: int`
- `length_mm: float`
- `waypoints: list[tuple[float, float, float]]`
- `routed_in_3d: bool`

### `ElectronicComponent`

Use for motors, switches, relays, connectors, and secondary supplies.

- `component_id: str`
- `type: ElectronicComponentType`
- `cots_part_id: str | None`
- `assembly_part_ref: str | None`
- `rated_voltage: float | None`
- `stall_current_a: float | None`

### `ElectronicsSection`

Planner-owned logical package embedded in `assembly_definition.yaml`.

- `power_supply: PowerSupplyConfig`
- `wiring: list[WireConfig]`
- `components: list[ElectronicComponent]`

### `AssemblyDefinition`

The live assembly schema includes:

- `electronics: ElectronicsSection | None`

If `electronics` is absent, legacy runs may still treat motors as implicitly powered. Do not rely on that mode for new explicit-electronics work.

## Current Helper Surface

`worker_heavy/utils/electronics.py` re-exports the live helper set:

- `build_circuit_from_section`
- `create_circuit`
- `validate_circuit`
- `simulate_circuit_transient`
- `calculate_power_budget`
- `route_wire`
- `check_wire_clearance`

Use that module as the current integration surface when editing repo/runtime code.

## Logical Circuit Helpers

### `build_circuit_from_section(section, switch_states=None, add_shunts=False)`

Builds a PySpice `Circuit` from an `ElectronicsSection`.

Important behavior:

- Main PSU nodes are normalized to `supply_v+` and `0`.
- Motor terminals normalize `a -> +` and `b -> -`.
- Wires are modeled as low-resistance links using AWG and length.

### `validate_circuit(circuit, psu_config=None, section=None)`

Runs the electrical pre-gate checks.

It can fail for:

- open circuit / floating nodes
- short circuit
- PSU overcurrent
- wire overcurrent

### `calculate_power_budget(circuit, psu_config)`

Use to estimate total draw against the PSU budget before simulation.

## Physical Wire Helpers

### `route_wire(...)`

Current signature:

```python
route_wire(
    wire_id: str,
    from_comp: str,
    from_term: str,
    to_comp: str,
    to_term: str,
    gauge_awg: int,
    waypoints: list[tuple[float, float, float]] | None = None,
    routed_in_3d: bool = False,
) -> WireConfig
```

Use this to create a `WireConfig` and compute `length_mm` from the path.

### `check_wire_clearance(wire_waypoints, assembly_meshes, clearance_mm=2.0, ...)`

Use this to reject routes that collide with assembly geometry.

Key behavior:

- samples along the route
- ignores a short distance near endpoints for attachment regions
- defaults to `2.0 mm` required clearance

## Authoring Guidance

- Keep planner artifacts in `assembly_definition.yaml`; do not invent parallel electronics schemas.
- Use explicit terminals and component IDs.
- Keep authored CAD/runtime scripts on the canonical `utils.*` contract; do not drag `shared.*` imports into authored scripts unless the task is explicitly repo-internal.
