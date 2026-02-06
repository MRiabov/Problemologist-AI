---
work_package_id: "WP01"
title: "Foundation & Data Models"
lane: "planned"
dependencies: []
subtasks: ["T001", "T002", "T003", "T004"]
---

# WP01: Foundation & Data Models

## Goal

Set up the core data structures, configuration system, and abstract base classes for the workbench system.

## Context

We are implementing the "Advanced Manufacturing Workbenches" feature (Spec 004). This WP lays the groundwork by defining the data models (Pydantic), the configuration loader (YAML), and the base class for all workbenches.

## Subtasks

### T001: Implement Core Data Models

**Objective**: Create the Pydantic models for workbench results and materials.
**Files**: `src/workbenches/models.py`
**Instructions**:

1. Create `src/workbenches/models.py`.
2. Determine necessary imports (pydantic `BaseModel`, `Field`, enum `Enum`).
3. Define `ManufacturingMethod` Enum:
   - `CNC = "cnc"`
   - `THREE_DP = "3dp"`
   - `INJECTION_MOLDING = "im"`
4. Define `WorkbenchResult` (BaseModel) with fields:
   - `is_manufacturable`: bool
   - `score`: float (0.0 to 1.0)
   - `unit_cost`: float
   - `setup_cost`: float
   - `total_cost`: float
   - `weight_g`: float
   - `material`: str
   - `method`: ManufacturingMethod
   - `violations`: List[str] (default factory list)
5. Define `MaterialDefinition` (BaseModel) with fields:
   - `name`: str
   - `density`: float (g/cm3)
   - `cost_per_kg`: float
   - `compatible_methods`: List[ManufacturingMethod]

### T002: Create Manufacturing Config

**Objective**: Create the YAML configuration file.
**Files**: `src/workbenches/manufacturing_config.yaml`
**Instructions**:

1. Create `src/workbenches/manufacturing_config.yaml`.
2. Add a `materials` list. Example entries:
   - Aluminum 6061 (density 2.7, cost ~5.0, CNC)
   - ABS (density 1.04, cost ~2.0, IM/3DP)
3. Add `cnc` config section:
   - `hourly_rate`: 50.0
   - `setup_fee`: 100.0
4. Add `im` config section:
   - `base_mold_cost`: 5000.0
   - `cycle_time_seconds`: 30
   - `hourly_rate`: 40.0

### T003: Implement Configuration Loader

**Objective**: Load the YAML config into Pydantic models.
**Files**: `src/workbenches/config.py`
**Instructions**:

1. Create `src/workbenches/config.py`.
2. Define Pydantic models mirroring the YAML structure:
   - `CNCConfig`
   - `IMConfig`
   - `ManufacturingConfig` (containing materials list, cnc, im)
3. Implement `load_config(path: str = "src/workbenches/manufacturing_config.yaml") -> ManufacturingConfig`.
   - Use `yaml.safe_load`.
   - Validate with Pydantic.
   - Cache the result (lru_cache or singleton).

### T004: Create Abstract Workbench Base Class

**Objective**: Define the interface for all workbenches.
**Files**: `src/workbenches/base.py`
**Instructions**:

1. Create `src/workbenches/base.py`.
2. Import `ABC`, `abstractmethod` from `abc`.
3. Import `Part`, `Compound` from `build123d`.
4. Import `WorkbenchResult` from `.models`.
5. Define `Workbench` class (inherits `ABC`).
6. Define abstract method `analyze(self, part: Part | Compound, quantity: int = 1) -> WorkbenchResult`.

## Verification

- Create `tests/workbenches/test_config.py`.
- Test `load_config()` ensures it reads the YAML correctly.
- Test that `WorkbenchResult` validates core fields.
- Run `pytest tests/workbenches/test_config.py`.

## Definition of Done

- All 4 files created.
- Config loads successfully.
- Models are strict.
- Tests pass.
