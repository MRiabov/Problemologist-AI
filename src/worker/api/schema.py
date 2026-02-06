from typing import List, Optional
from pydantic import BaseModel
from ..filesystem.backend import FileInfo

from pydantic import BaseModel, Field

class ListFilesRequest(BaseModel):
    """Request to list files in a directory."""
    path: str = Field(default="/", min_length=1)

class ReadFileRequest(BaseModel):
    """Request to read a file."""
    path: str = Field(..., min_length=1)

class WriteFileRequest(BaseModel):
    """Request to write a file."""
    path: str = Field(..., min_length=1)
    content: str

class EditOp(BaseModel):
    """A single edit operation (find and replace)."""
    old_string: str = Field(..., min_length=1)
    new_string: str

class EditFileRequest(BaseModel):
    """Request to edit a file with one or more operations."""
    path: str = Field(..., min_length=1)
    edits: List[EditOp] = Field(..., min_length=1)

class ExecuteRequest(BaseModel):
    """Request to execute Python code."""
    code: str = Field(..., min_length=1)
    timeout: int = Field(default=30, ge=1, le=300)

class ExecuteResponse(BaseModel):
    """Response from executing Python code."""
    stdout: str
    stderr: str
    exit_code: int
    timed_out: bool = False
