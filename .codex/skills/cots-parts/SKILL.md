---
name: cots-parts
description: Catalog-backed COTS part modeling and validation guidance for Problemologist. Use when a task involves motors, off-the-shelf parts, benchmark fixtures, or translating declared COTS components into build123d geometry and handoff artifacts.
---

# COTS Parts

Use this skill when a design includes a catalog-backed component. It is a router: it keeps part identity, geometry, motion, and ownership aligned.

## Read This

- `references/contract.md` for the class-first COTS contract and motor-specific notes.

## Core Rules

1. Treat the concrete COTS class instance as the part.
2. Preserve `PartMetadata.cots_id`, the part label, and COTS usage/provenance on the instance.
3. Place after construction; do not bake world-space placement into the COTS class.
4. Do not execute `import_recipe`; it is provenance, not runtime code.
5. Separate geometry from motion: the motor body defines fit and frame, while joint or control contracts define direction and actuation.
6. If benchmark or solution handoffs declare a COTS component, instantiate it in authored geometry. A declared-but-unused COTS part is a contract failure.
7. Keep benchmark-owned fixtures read-only and engineer-owned parts validated and priced according to ownership.

## Workflow

1. Identify the exact part family and catalog ID.
2. Instantiate the matching concrete COTS class or approved factory path.
3. Place it with `Location(...)`, `.move(...)`, or `.moved(...)` after construction.
4. Verify metadata, labels, interface frame, and clearance.
5. If the part is actuated, align the joint or motion contract, not the geometry, with the rotation direction.

## Notes

- For motors, use the current seed class and its canonical local frame.
- For authored scripts, follow the repo's approved public import surface. Do not invent new runtime helpers.
- If a task requires a family that is not modeled by a concrete class yet, stop and mark the gap instead of fabricating a vendor-perfect proxy.
