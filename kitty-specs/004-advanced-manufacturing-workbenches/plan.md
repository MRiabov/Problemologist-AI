# Implementation Plan: Advanced Manufacturing Workbenches

## Goal

Implement a deterministic manufacturing validation and cost estimation system ("Workbenches") for CNC Milling and Injection Molding, exposed via a unified Python utility for agents.

## User Review Required
>
> [!IMPORTANT]
> This plan assumes `build123d` and `trimesh` are available in the worker environment.
> We are using a config-driven approach for cost parameters (`manufacturing_config.yaml`).

## Proposed Changes

### Core Logic (`src/workbenches/`)

#### [NEW] [base.py](file:///src/workbenches/base.py)

- Abstract base class for workbenches.
- Defines `analyze(part) -> WorkbenchResult`.

#### [NEW] [cnc.py](file:///src/workbenches/cnc.py)

- Implements CNC constraints:
  - Tool accessibility (raycasting).
  - Internal corner radii check.
- Cost model: Material removal rate + setup time.

#### [NEW] [injection_molding.py](file:///src/workbenches/injection_molding.py)

- Implements IM constraints:
  - Draft angle check.
  - Wall thickness verification.
  - Undercut detection.
- Cost model: Mold base cost + cycle time.

#### [NEW] [config.py](file:///src/workbenches/config.py)

- Loads `manufacturing_config.yaml`.
- Pydantic models for configuration.

#### [NEW] [manufacturing_config.yaml](file:///src/workbenches/manufacturing_config.yaml)

- YAML file with material costs, machine rates, etc.

### Agent Utility (`src/worker/utils/`)

#### [NEW] [dfm.py](file:///src/worker/utils/dfm.py)

- Implements `validate_and_price(part, quantity)`.
- Facade that dispatches to appropriate workbench based on metadata.

### Data Models

#### [NEW] [models.py](file:///src/workbenches/models.py)

- `WorkbenchResult`, `ManufacturingMethod`, `MaterialDefinition`.

## Verification Plan

### Automated Tests

- `pytest tests/workbenches/test_cnc.py`: Verify constraints on known valid/invalid geometries (e.g., box with undercut).
- `pytest tests/workbenches/test_im.py`: Verify draft angle checks.
- `pytest tests/workbenches/test_costing.py`: Verify cost calculations against manually calculated examples.

### Manual Verification

- Run a `quickstart.py` script that creates a part and calls `validate_and_price`, ensuring the report is printed correctly.
