from typing import List, Optional
from pydantic import BaseModel, Field, StrictStr, StrictInt, StrictBool
from ..filesystem.backend import FileInfo


from src.shared.enums import ResponseStatus


class ReadFileResponse(BaseModel):
    """Response from reading a file."""
    content: StrictStr


class StatusResponse(BaseModel):
    """Generic status response."""
    status: ResponseStatus


class ListFilesRequest(BaseModel):
    """Request to list files in a directory."""
    path: StrictStr = Field(default="/", min_length=1)

class ReadFileRequest(BaseModel):
    """Request to read a file."""
    path: StrictStr = Field(..., min_length=1)

class WriteFileRequest(BaseModel):
    """Request to write a file."""
    path: StrictStr = Field(..., min_length=1)
    content: StrictStr

class EditOp(BaseModel):
    """A single edit operation (find and replace)."""
    old_string: StrictStr = Field(..., min_length=1)
    new_string: StrictStr

class EditFileRequest(BaseModel):
    """Request to edit a file with one or more operations."""
    path: StrictStr = Field(..., min_length=1)
    edits: List[EditOp] = Field(..., min_length=1)

class ExecuteRequest(BaseModel):
    """Request to execute Python code."""
    code: StrictStr = Field(..., min_length=1)
    timeout: StrictInt = Field(default=30, ge=1, le=300)

class ExecuteResponse(BaseModel):
    """Response from executing Python code."""
    stdout: StrictStr
    stderr: StrictStr
    exit_code: StrictInt
    timed_out: StrictBool = False
