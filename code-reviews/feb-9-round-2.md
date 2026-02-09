# Architecture Delta: Feb 9 Round 2

**Date**: 2026-02-09  
**Commits Covered**: `a89ca8e` → `0b88f80` (since ~06:41 today)  
**Status**: New specifications added to `desired_architecture.md` - implementation pending.

---

## 1. Motor Simulation Specification *(NEW)*

Detailed motor physics and failure handling have been added.

| Component | Description | Status |
|-----------|-------------|--------|
| **`forcerange` attribute** | Clamp actuator output to realistic torque limits from COTS catalog | **MISSING** |
| **Motor overload failure** | Fail simulation if motor clamped at limit for >2 seconds | **MISSING** |
| **PD gains source** | `kp`, `kv` values derived from COTS servo catalog | **MISSING** |

> [!IMPORTANT]
> This requires integrating COTS servo specs into simulation XML generation and implementing overload detection in the simulation loop.

---

## 2. Objective Zone Collision Detection *(NEW)*

New specification for vertex-based collision checking.

| Component | Description | Status |
|-----------|-------------|--------|
| **Vertex-based checking** | Check if any mesh vertex touches goal/forbid zones | **MISSING** |

> [!NOTE]
> Spec states: "limit to vertices for simplicity/speed".

---

## 3. Static Randomization *(NEW)*

Two new randomization systems for benchmark diversity.

### Material Static Randomization

| Description | Status |
|-------------|--------|
| Randomly switch materials for moving parts (heavier/lighter/different friction) | **MISSING** |
| Engineer informed about materials ahead of time | **PARTIAL** (config exists, injection not verified) |
| Optional "minimum strength" material constraints | **MISSING** |

### Visual Static Randomization

| Description | Status |
|-------------|--------|
| Parts change color alongside material change per materials config | **MISSING** |

---

## 4. Runtime Randomization Verification *(NEW)*

| Component | Description | Status |
|-----------|-------------|--------|
| **Multi-run verification** | Run simulation 5x with perturbed positions for consistency | **MISSING** |

> [!NOTE]
> MuJoCo supports parallel simulations natively - leverage this for efficiency.

---

## 5. Expanded Failure Modes *(NEW)*

The failure specification has been significantly expanded.

| Failure Mode | Description | Status |
|--------------|-------------|--------|
| **Timeout cap** | Hard cap of 30 seconds; planner sets per-benchmark timeout | **MISSING** |
| **Instability** | NaNs, parts interference detection | **PARTIAL** (basic NaN check exists) |
| **Forbid zone contact** | Any part touching forbid zone = failure | **MISSING** |
| **Part breakage** | Parts with custom breaking logic (e.g., motor overload) | **MISSING** |

---

## 6. CAD to Mesh Conversion Pipeline *(NEW)*

New pipeline specification for build123d → MuJoCo.

| Component | Description | Status |
|-----------|-------------|--------|
| **OBJ export** | Use `export_obj()` instead of STL (less bulky) | **MISSING** |
| **Recentering** | Recenter parts to (0,0,0) before export | **MISSING** |
| **Mesh limits** | Unbounded vertex count; prefer quality; simplify where safe | **MISSING** |
| **Watertightness** | Required for all meshes | **MISSING** (no validation) |

---

## 7. manufacturing_config.yaml Schema *(NEW)*

| Component | Description | Status |
|-----------|-------------|--------|
| **Materials section** | `color`, `elongation_stress`, `restitution`, `friction_coef` fields | **PARTIAL** |
| **Auto-validation** | Unit tests for config integrity (e.g., invalid material references) | **PARTIAL** |
| **Agent read access** | Agents can read config for pricing estimates | **PARTIAL** |

---

## 8. DOF Warnings in Validation *(NEW)*

| Component | Description | Status |
|-----------|-------------|--------|
| **DOF check** | Warn if compound has ≥4 DOFs (unusual in engineering) | **MISSING** |
| **Reviewer notification** | Reviewer gets stricter when DOF warning present | **MISSING** |

---

## 9. Constraints & Fasteners *(NEW)*

| Component | Description | Status |
|-----------|-------------|--------|
| **Realistic constraint validation** | Parts must be mechanically held (fasteners/gravity); "sticky" CAD constraints forbidden | **MISSING** |
| **Fixed parts metadata** | `fixed=True` allowed *only* for specific environment parts, not engineer parts | **MISSING** |
| **bd-warehouse integration** | Integration for standard nuts, bolts, screws geometry/props | **MISSING** |
| **Fastener Helper** | `fastener_hole(part, pos, hole_id, type, add_fastener)`: cuts hole + creates `RigidJoint` | **MISSING** |
| **RigidJoint System** | Use `RigidJoint` and `connect_to()` for mating (avoids manual transforms) | **MISSING** |
| **Hole Types** | Enum `HoleType`: `FlatHeadHole`, `CounterBoreHole`, `CounterSinkHole` | **MISSING** |
| **Simulation Physics** | Engineer parts cannot be static; they obey physics (gravity, friction, restitution) | **MISSING** |

> [!NOTE]
> The spec now explicitly mandates using `build123d`'s `RigidJoint` system to handle mating transforms automatically. The `fastener_hole` helper is the primary interface for this.

---

## Summary of Required Implementation

### High Priority

1. **Motor simulation physics** - forcerange, overload detection, PD gains from catalog
2. **Failure mode expansion** - 30s cap, forbid zones, part breakage
3. **Multi-run verification** - 5x simulation runs for robustness

### Medium Priority

1. **CAD→Mesh pipeline** - OBJ export, recentering, watertightness validation
2. **Static randomization** - material & visual randomization for benchmarks
3. **DOF warnings** - validation + reviewer integration

### Lower Priority

1. **Constraint/fastener validation** - bd-warehouse, realistic constraint checks
2. **Vertex collision detection** - goal/forbid zone vertex intersection
