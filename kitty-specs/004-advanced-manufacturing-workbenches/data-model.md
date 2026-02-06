# Data Model: Advanced Manufacturing Workbenches

## Core Models (Pydantic)

These models define the structured feedback returned to the engineering agent.

### 1. ManufacturingReport

```python
class ManufacturingReport(BaseModel):
    status: Literal["pass", "fail"]
    process: Literal["cnc_milling", "injection_molding", "3d_printing"]
    score: float = Field(ge=0.0, le=1.0)
    violations: List[DFMViolation]
    cost: CostBreakdown
    metadata: Dict[str, Any] = {}
```

### 2. DFMViolation

```python
class DFMViolation(BaseModel):
    type: str  # e.g., "undercut", "draft_angle", "thin_wall"
    description: str
    severity: Literal["critical", "warning"]
    affected_face_indices: List[int]
    centroid: Tuple[float, float, float]
```

### 3. CostBreakdown

```python
class CostBreakdown(BaseModel):
    quantity: int
    unit_price: float
    setup_cost: float
    material_cost: float
    processing_cost: float
    total_price: float
```

## Storage

1. **Reports**: Serialized to JSON and stored in the `StepTrace` tool output in Postgres.
2. **Visualizations**: Mesh annotations (highlighted faces) are rendered to static images by the worker and uploaded to S3.
