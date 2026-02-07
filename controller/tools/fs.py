from langchain_core.tools import StructuredTool
from pydantic import BaseModel, Field, StrictInt, StrictStr

from controller.middleware.remote_fs import RemoteFilesystemMiddleware


class LsInput(BaseModel):
    path: StrictStr = Field(default="/", description="Path to list contents of.")


class ReadInput(BaseModel):
    path: StrictStr = Field(..., description="Path to the file to read.")


class WriteInput(BaseModel):
    path: StrictStr = Field(..., description="Path to the file to write.")
    content: StrictStr = Field(..., description="Content to write to the file.")


class ExecInput(BaseModel):
    code: StrictStr = Field(..., description="Python code to execute.")
    timeout: StrictInt = Field(default=30, description="Execution timeout in seconds.")


class BenchmarkInput(BaseModel):
    script_path: str = Field(
        default="script.py",
        description="Path to the script containing the build() function.",
    )


def create_fs_tools(fs: RemoteFilesystemMiddleware) -> list[StructuredTool]:
    """Create a list of filesystem tools for the agent."""

    ls_tool = StructuredTool.from_function(
        coroutine=fs.list_files,
        name="ls",
        description="List files and directories in the workspace.",
        args_schema=LsInput,
    )

    read_tool = StructuredTool.from_function(
        coroutine=fs.read_file,
        name="read_file",
        description="Read the contents of a file.",
        args_schema=ReadInput,
    )

    write_tool = StructuredTool.from_function(
        coroutine=fs.write_file,
        name="write_file",
        description="Write content to a file in the workspace.",
        args_schema=WriteInput,
    )

    exec_tool = StructuredTool.from_function(
        coroutine=fs.run_command,
        name="execute_python",
        description="Execute Python code in the sandboxed environment.",
        args_schema=ExecInput,
    )

    simulate_tool = StructuredTool.from_function(
        coroutine=fs.simulate,
        name="simulate",
        description="Run a physics stability check on the current design.",
        args_schema=BenchmarkInput,
    )

    validate_tool = StructuredTool.from_function(
        coroutine=fs.validate,
        name="validate",
        description="Verify geometric validity and randomization robustness.",
        args_schema=BenchmarkInput,
    )

    submit_tool = StructuredTool.from_function(
        coroutine=fs.submit,
        name="submit_for_review",
        description="Submit the current design for human review.",
        args_schema=BenchmarkInput,
    )

    return [
        ls_tool,
        read_tool,
        write_tool,
        exec_tool,
        simulate_tool,
        validate_tool,
        submit_tool,
    ]
