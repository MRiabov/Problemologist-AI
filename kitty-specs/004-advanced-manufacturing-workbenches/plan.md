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

#### [MODIFY] [base.py](file:///src/workbenches/base.py)

- Protocol or shared Pydantic models for workbench results.
- Defines standard interface for `analyze` functions.

#### [NEW] [cnc.py](file:///src/workbenches/cnc.py)

- Functional implementation of CNC constraints:
  - `analyze_cnc(part: Part | Compound, config: CNCConfig) -> WorkbenchResult`.
  - Helper functions for tool accessibility and corner radii.
- Integrated `structlog` for geometry analysis tracing.

#### [NEW] [injection_molding.py](file:///src/workbenches/injection_molding.py)

- Functional implementation of IM constraints:
  - `analyze_im(part: Part | Compound, config: IMConfig) -> WorkbenchResult`.
  - Helper functions for draft angles and wall thickness.
- Integrated `structlog` for molding analysis tracing.

#### [MODIFY] [config.py](file:///src/workbenches/config.py)

- Loads `manufacturing_config.yaml`.
- Strictly uses **Pydantic** models for configuration.

#### [NEW] [manufacturing_config.yaml](file:///src/workbenches/manufacturing_config.yaml)

- YAML file with material costs, machine rates, etc.

### Agent Utility (`src/worker/utils/`)

#### [MODIFY] [dfm.py](file:///src/worker/utils/dfm.py)

- Implements `validate_and_price(part, quantity)`.
- Functional facade that dispatches to `analyze_cnc` or `analyze_im`.
- Uses `structlog` to log the dispatch and result.

### Data Models

#### [MODIFY] [models.py](file:///src/workbenches/models.py)

- Strict **Pydantic** models: `DFMReport`, `WorkbenchResult`, `ManufacturingMethod`, `MaterialDefinition`.

## Verification Plan

### Automated Tests

- `pytest tests/workbenches/test_cnc.py`: Verify functions on known valid/invalid geometries.
- `pytest tests/workbenches/test_im.py`: Verify draft angle check functions.
- `pytest tests/workbenches/test_costing.py`: Verify cost calculation functions.

### Manual Verification

- Run a `quickstart.py` script that creates a part and calls `validate_and_price`, ensuring the `structlog` output and Pydantic-validated report are correct.
