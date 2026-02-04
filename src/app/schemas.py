from typing import List, Optional, Dict, Any
from uuid import UUID
from datetime import datetime
from pydantic import BaseModel, Field

# --- Shared Models ---

class ArtifactSchema(BaseModel):
    id: UUID
    step_id: UUID
    file_path: str
    file_type: str
    created_at: datetime

    class Config:
        from_attributes = True

class StepSchema(BaseModel):
    index: int
    type: str
    agent_role: Optional[str] = None
    content: Optional[str] = None
    tool_name: Optional[str] = None
    tool_input: Optional[str] = None
    tool_output: Optional[str] = None
    metadata: Dict[str, Any] = Field(default_factory=dict)
    artifacts: List[str] = Field(default_factory=list)

    class Config:
        from_attributes = True

class EpisodeSummarySchema(BaseModel):
    id: str
    timestamp: datetime
    name: str

    class Config:
        from_attributes = True

class EpisodeDetailSchema(BaseModel):
    id: str
    name: str
    steps: List[StepSchema]

    class Config:
        from_attributes = True

# --- Benchmark Models ---

class BenchmarkScenarioSchema(BaseModel):
    id: str
    seed: str
    intent: str
    logic: List[str]
    given: str
    when: str
    then: str
    xml_script: str
    render_mode: str = "MUJOCO_NATIVE"

class ValidationCheck(BaseModel):
    name: str
    passed: bool
    details: str

class BenchmarkValidationSchema(BaseModel):
    checks: List[ValidationCheck]
    ready_to_promote: bool
