# Feature Specification: Advanced Manufacturing Workbenches

**Feature**: 004-advanced-manufacturing-workbenches
**Status**: Draft
**Mission**: software-dev

## 1. Overview

The **Advanced Manufacturing Workbenches** feature provides the core logic for **Design for Manufacturing (DFM)** constraints and **Cost Estimation**. It is exposed to the Agent via the `utils.validate_and_price` Python function within the Worker environment.

It supports two primary processes:

1. **CNC Milling**: 3-axis machining constraints.
2. **Injection Molding (IM)**: Draft angles, wall thickness, and undercut detection.

## 2. Goals & Success Criteria

### 2.1. Primary Goals

1. **Strict Validation**: Reject designs that are physically impossible to manufacture with the selected process.
2. **Cost Awareness**: Provide differential pricing based on volume (e.g., IM is cheaper >10k units, CNC <100 units).
3. **Agent Feedback**: Return precise, actionable error messages (e.g., "Face X has 0 deg draft") via the `validate_and_price` return dict.

### 2.2. Success Criteria

- **Accuracy**: Correctly flags undercuts and sharp internal corners for CNC.
- **Performance**: Validation < 5s for standard parts.
- **Integration**: Seamlessly callable from Agent's `script.py` inside the Worker container.

## 3. User Stories

- **As an Agent**, I want to import `validate_and_price` and pass my `Part` object to see if I made a mistake.
- **As an Agent**, I want to know the "Unit Cost" at 1,000 units so I can optimize my design for mass production.

## 4. Functional Requirements

### 4.1. The `validate_and_price` Utility

The function signature in `utils` shall be:

```python
def validate_and_price(part: Part | Compound, quantity: int = 1) -> DFMReport:
    """
    Validates the part against inferred or specified manufacturing metadata.
    Returns validation status, violations, and cost or raises an error.
    Uses Pydantic for validation and structlog for observability.
    """
```

**Input**:

- `part`: `build123d` object. Must have `part.metadata` set (using `PartMetadata` or `CompoundMetadata`).
- `quantity`: Production volume for cost amortisation.

**Output (DFMReport Pydantic Model)**:

- `valid`: bool
- `unit_cost`: float
- `violations`: List[str] (e.g., "Undercut detected at (x,y,z)")
- `metadata`: Dict (for additional workbench-specific data)

### 4.2. CNC Workbench

**Constraints**:

- **Tool Access**: No internal features smaller than tool radius (3mm default).
- **Visibility**: All features accessible from +Z (3-axis assumption).
- **Corner Radii**: Checks for sharp internal corners.

### 4.3. Injection Molding Workbench

**Constraints**:

- **Draft Angle**: All vertical faces > 2 degrees relative to pull vector.
- **Wall Thickness**: Variance < 30%.
- **Undercuts**: No trapped geometry (Simple 2-part mold).

## 5. Technical Design

### 5.1. Tech Stack

- **Kernel**: `build123d` / `trimesh` (for raycasting/draft analysis).
- **Format**: All data exchange via **Pydantic Models**.
- **Observability**: **structlog** for structured logging of validation steps and errors.
- **Language**: Python 3.10+.
- **Config**: YAML-based cost parameters (material cost, machine time, setup).

### 5.2. Integration

This logic resides in `src/worker/utils/workbenches/` and is bundled into the Worker container. The `validate_and_price` function acts as the unified functional facade. The workbench assets and configuration are strictly read-only for the worker.
