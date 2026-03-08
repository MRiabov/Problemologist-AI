from collections.abc import Callable

from controller.middleware.remote_fs import EditOp, RemoteFilesystemMiddleware


def create_fs_tools(fs: RemoteFilesystemMiddleware) -> list[Callable]:
    """Create plain callable filesystem tools aligned with runtime tool naming."""

    async def list_files(path: str = "/"):
        return await fs.list_files(path)

    async def read_file(path: str):
        return await fs.read_file(path)

    async def write_file(path: str, content: str, overwrite: bool = False):
        return await fs.write_file(path, content, overwrite=overwrite)

    async def edit_file(path: str, old_string: str, new_string: str):
        return await fs.edit_file(
            path, [EditOp(old_string=old_string, new_string=new_string)]
        )

    async def execute_command(command: str):
        return await fs.run_command(command=command, timeout=30)

    tools = [list_files, read_file, write_file, edit_file, execute_command]
    return tools
