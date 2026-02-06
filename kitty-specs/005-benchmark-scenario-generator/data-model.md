# Data Model: Benchmark Scenario Generator

## Core Entities (Pydantic Models)

### 1. BenchmarkAsset

Metadata for a validated, ready-to-solve benchmark.

```python
class BenchmarkAsset(BaseModel):
    benchmark_id: UUID
    mjcf_url: HttpUrl
    build123d_url: HttpUrl  # Original source code
    preview_bundle_url: HttpUrl # ZIP of 24 multi-view images
    random_variants: List[UUID]  # Links to specific seeded variations
    difficulty_score: float
```

### 2. GenerationSession

State of a benchmark generation graph run.

```python
class GenerationSession(BaseModel):
    session_id: UUID
    prompt: str
    status: Literal["planning", "executing", "validating", "accepted", "rejected"]
    validation_logs: List[str]
```

## Persistence strategy

1. **Relational**: Session state and Benchmark metadata are stored in Postgres.
2. **Assets (S3)**: All MJCF, meshes, and images are stored as permanent objects in S3 Railway buckets.
3. **Traceability**: LLM reasoning for benchmark design is persisted via LangGraph checkpoints.
