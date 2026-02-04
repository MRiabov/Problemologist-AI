from typing import Any

from pydantic import BaseModel, Field


class ToolRequest(BaseModel):
    """Request payload for executing a tool."""

    tool_name: str = Field(..., description="Name of the tool to execute")
    arguments: dict[str, Any] = Field(
        default_factory=dict, description="Arguments for the tool"
    )


class ToolResponse(BaseModel):
    """Response payload from a tool execution."""

    output: str = Field(..., description="String output from the tool")
    error: str | None = Field(None, description="Error message if execution failed")
    status: str = Field("success", description="Status of the execution: success/error")


class CommandRequest(BaseModel):
    """Request payload for executing a shell command."""

    command: str = Field(..., description="Shell command to execute")
    timeout: int = Field(30, description="Timeout in seconds")
    workdir: str | None = Field(None, description="Working directory")


class CommandResponse(BaseModel):
    """Response payload from a command execution."""

    stdout: str = Field("", description="Standard output")
    stderr: str = Field("", description="Standard error")
    return_code: int = Field(..., description="Exit code")


class ScriptRequest(BaseModel):
    """Request payload for executing a python script."""

    script_name: str = Field(..., description="Name of the script in the workspace")
    args: list[str] = Field(default_factory=list, description="Command line arguments")
    timeout: int = Field(30, description="Timeout in seconds")
