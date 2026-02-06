# Data Model: Benchmark Scenario Generator

## Core Models (Pydantic)

### 1. BenchmarkScenario

```python
class BenchmarkScenario(BaseModel):
    id: UUID
    tier: Literal["kinematic", "spatial", "dynamics"]
    description: str
    template_url: str  # S3 URL to build123d script
    mjcf_url: str      # S3 URL to validated MJCF
    preview_urls: List[str] # S3 URLs to renders
    max_unit_cost: float
    target_quantity: int
    validation_status: Literal["passed", "failed"]
```

### 2. GenerationRequest

```python
class GenerationRequest(BaseModel):
    prompt: str
    tier: str
    variation_count: int = 1
    seed_override: Optional[int] = None
```

### 3. ScenarioValidationReport

```python
class ScenarioValidationReport(BaseModel):
    is_stable: bool
    telemetry: Dict[str, Any]
    error_summary: Optional[str]
    sim_time_s: float
```

## Persistence Strategy

1. **Metadata**: Stored in the central **Postgres** DB on the Controller.
2. **Files (code, xml, images)**: Stored as permanent assets in **S3**.
3. **Logs**: Reasoning traces are persisted via the global `StepTrace` schema.
