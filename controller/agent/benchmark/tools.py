from typing import List, Optional
from langchain_core.tools import tool
from controller.middleware.remote_fs import RemoteFilesystemMiddleware
from worker.api.schema import EditOp


def get_benchmark_tools(fs: RemoteFilesystemMiddleware):
    @tool
    async def list_files(path: str = "/") -> List[dict]:
        """List files in the workspace."""
        return await fs.list_files(path)

    @tool
    async def read_file(path: str) -> str:
        """Read a file's content."""
        return await fs.read_file(path)

    @tool
    async def write_file(path: str, content: str, overwrite: bool = False) -> bool:
        """Write content to a file."""
        return await fs.write_file(path, content, overwrite=overwrite)

    @tool
    async def edit_file(path: str, old_string: str, new_string: str) -> bool:
        """Edit a file by replacing old_string with new_string."""
        return await fs.edit_file(
            path, [EditOp(old_string=old_string, new_string=new_string)]
        )

    @tool
    async def grep(
        pattern: str, path: Optional[str] = None, glob: Optional[str] = None
    ) -> List[dict]:
        """Search for a pattern in files."""
        return await fs.grep(pattern, path, glob)

    @tool
    async def simulate(
        script_path: str, backend: SimulatorBackendType = SimulatorBackendType.MUJOCO
    ) -> dict:
        """Run physics simulation for the benchmark.
        Use MUJOCO for rigid body only, GENESIS for fluids or FEM.
        """
        return await fs.simulate(script_path, backend=backend)

    @tool
    async def validate(script_path: str) -> dict:
        """Run geometric validation for the benchmark."""
        return await fs.validate(script_path)

    @tool
    async def submit_for_review(script_path: str) -> dict:
        """Submit the benchmark for review."""
        return await fs.submit(script_path)

    return [
        list_files,
        read_file,
        write_file,
        edit_file,
        grep,
        simulate,
        validate,
        submit_for_review,
    ]
