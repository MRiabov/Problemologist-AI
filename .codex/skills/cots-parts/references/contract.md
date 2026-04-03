# COTS Part Contract

## Contents

1. Class-first COTS parts
2. Motor-specific placement
3. Fixtures vs solution parts
4. Validation checklist
5. Anti-patterns

## 1. Class-First COTS Parts

A COTS part is a concrete build123d-backed class instance that carries the part geometry and its metadata together.

Use the concrete class as the source of truth for:

- `PartMetadata.cots_id`
- part label
- price / weight data when present
- observability or usage events emitted on construction

Do not split the geometry into an anonymous solid plus a separate identity object unless the task explicitly requires that refactor.

## 2. Motor-Specific Placement

For motors, the class instance defines the local frame.

- Local origin stays at the mounting datum.
- Local `+Z` is the shaft axis.
- Placement in the assembly happens after construction with normal build123d transforms.
- Spin direction belongs in the motion or joint contract, not in the COTS body geometry.

If the runtime uses the current motor-label pattern, keep `stator` and `rotor` labels on separate solids only when the task explicitly requires that MJCF path.

## 3. Fixtures vs Solution Parts

Benchmark-owned COTS parts are read-only context.

Engineer-owned COTS parts count toward solution cost, weight, and validation just like other solution geometry.

The provider or class factory does not decide ownership. Ownership comes from the handoff context.

## 4. Validation Checklist

Before handoff, confirm:

1. the declared COTS part was actually instantiated,
2. the part label is stable and unique where required,
3. the part preserves the expected local frame,
4. the geometry is close enough for fit and clearance,
5. the part remains traceable to the catalog row or seed class that produced it.

## 5. Anti-Patterns

- Do not execute `import_recipe` text as code.
- Do not convert a COTS part into vendor-perfect geometry for its own sake.
- Do not mix motion direction logic into the geometry class.
- Do not invent a generic importer when the task only needs the existing concrete class.
- Do not drop provenance by copying the geometry into a new anonymous part.
