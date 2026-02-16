from enum import StrEnum
from typing import Annotated

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
    normal: Vector3 | None = None
    direction: Vector3 | None = None


class CodeReference(BaseModel):
    file_path: str
    start_line: int
    end_line: int


class SteerablePrompt(BaseModel):
    text: str
    selections: list[GeometricSelection] = Field(default_factory=list)
    code_references: list[CodeReference] = Field(default_factory=list)
    mentions: list[str] = Field(default_factory=list)
