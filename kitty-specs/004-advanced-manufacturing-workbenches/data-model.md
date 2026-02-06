# Data Model: Advanced Manufacturing Workbenches

## Core Entities (Pydantic Models)

### 1. WorkbenchResult

The comprehensive feedback model for the agent.

```python
class WorkbenchResult(BaseModel):
    is_manufacturable: bool
    score: float  # 0.0 to 1.0
    unit_cost: float
    setup_cost: float
    total_cost: float
    weight_g: float
    material: str
    method: Literal["3dp", "cnc", "im"]
    violations: List[str]  # Detailed Markdown error strings
```

### 2. ManufacturingMethod (Enum)

```python
class ManufacturingMethod(str, Enum):
    CNC = "cnc"
    THREE_DP = "3dp"
    INJECTION_MOLDING = "im"
```

### 3. MaterialDefinition

```python
class MaterialDefinition(BaseModel):
    name: str
    density: float
    cost_per_kg: float
    compatible_methods: List[ManufacturingMethod]
```

## Persistence

1. **Global Cache**: Results are keyed by `mesh_hash + method + material`.
2. **Observability**: `WorkbenchResult` is attached to the `StepTrace` in the Observability DB for performance tracking.
