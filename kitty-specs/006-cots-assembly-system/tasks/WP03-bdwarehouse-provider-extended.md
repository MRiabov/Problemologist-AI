---
work_package_id: "WP03"
title: "BDWarehouse Provider (Extended)"
lane: "planned"
dependencies: ["WP02"]
subtasks:
  - T011
  - T012
  - T013
  - T014
---

# WP03: BDWarehouse Provider (Extended)

**Goal**: Extend the provider to support Bearings, Fasteners, and Beams.
**Role**: Expands the catalog to be useful.

## Subtasks

### T011: Index Bearings
**Purpose**: Add standard bearings (608, etc.).
**Location**: `src/cots/providers/bd_warehouse.py`
**Steps**:
1. Import `Bearing` types from `bd_warehouse.bearing`.
2. Iterate available types/sizes.
3. ID format: `bd_warehouse:bearing:608`.
4. Add to internal index.

### T012: Index Fasteners
**Purpose**: Add Screws and Nuts.
**Location**: `src/cots/providers/bd_warehouse.py`
**Steps**:
1. Import `SocketHeadCapScrew`, `HexNut`, etc.
2. Iterate standards (ISO, DIN) and sizes (M3, M4...).
3. ID format: `bd_warehouse:fastener:SocketHeadCapScrew:iso4762:M6-10`.
   - *Note*: Fasteners have many parameters (length, thread). Use a sensible default or list common lengths.
4. Add to internal index.

### T013: Index Beams
**Purpose**: Add structural profiles.
**Location**: `src/cots/providers/bd_warehouse.py`
**Steps**:
1. Import `Misumi` or generic T-slot profiles if available in library (check docs/source).
2. If not in `bd_warehouse`, skip or mock for now (as per spec "Beams" were requested, `bd_warehouse` has pipe/flange, check if it has structural framing. If not, use `build123d`'s beam objects if applicable, or skip). *Self-Correction*: `bd_warehouse` is mainly fasteners/piping/gears. If beams are missing, note it and skip. *Check*: `bd_warehouse` documentation mentions "Beams" in spec prompt. Assume it exists or use generic if available.

### T014: Recipe Validation
**Purpose**: Ensure recipes work for these new types.
**Location**: `tests/test_cots_bdwarehouse.py`
**Steps**:
1. Update tests to search for "bearing", "screw".
2. Verify recipes are importable/executable (optional: using `exec` in test or just string check).

## Definition of Done
- Bearings and Fasteners are indexed.
- Search returns them.
- Recipes are syntactically correct.
