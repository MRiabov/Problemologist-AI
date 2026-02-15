from enum import StrEnum
from typing import Annotated, List, Optional

from pydantic import BaseModel, Field


Vector3 = Annotated[tuple[float, float, float], Field(min_length=3, max_length=3)]


class SelectionLevel(StrEnum):
    FACE = "FACE"
    EDGE = "EDGE"
    VERTEX = "VERTEX"
    PART = "PART"
    SUBASSEMBLY = "SUBASSEMBLY"


class GeometricSelection(BaseModel):
    level: SelectionLevel
    target_id: str
    center: Vector3
    normal: Optional[Vector3] = None
    direction: Optional[Vector3] = None


class CodeReference(BaseModel):
    file_path: str
    start_line: int
    end_line: int


class SteerablePrompt(BaseModel):
    text: str
    selections: List[GeometricSelection] = Field(default_factory=list)
    code_references: List[CodeReference] = Field(default_factory=list)
    mentions: List[str] = Field(default_factory=list)
