import asyncio

import structlog
from deepagents.backends.protocol import (
    BackendProtocol,
    EditResult,
    FileDownloadResponse,
    FileUploadResponse,
    GrepMatch,
    WriteResult,
)
from deepagents.backends.protocol import (
    FileInfo as ProtocolFileInfo,
)
from deepagents.backends.utils import (
    check_empty_content,
    format_content_with_line_numbers,
)

from .worker import WorkerClient

logger = structlog.get_logger(__name__)


class RemoteFilesystemBackend(BackendProtocol):
    """Controller-side backend that proxies operations to the Worker API.

    Implements BackendProtocol by calling WorkerClient methods.
    This allows the Agent to 'live' in the Controller but manipulate
    files on the Worker.
    """

    def __init__(self, client: WorkerClient):
        self.client = client

    # --- Async BackendProtocol Implementation ---

    async def als_info(self, path: str) -> list[ProtocolFileInfo]:
        files = await self.client.list_files(path)
        return [
            {
                "path": f.path,
                "is_dir": f.is_dir,
                "size": f.size or 0,
            }
            for f in files
        ]

    async def aread(self, file_path: str, offset: int = 0, limit: int = 2000) -> str:
        try:
            content = await self.client.read_file(file_path)
            empty_msg = check_empty_content(content)
            if empty_msg:
                return empty_msg

            lines = content.splitlines()
            start_idx = offset
            end_idx = min(start_idx + limit, len(lines))

            if start_idx >= len(lines):
                return f"Error: Line offset {offset} exceeds file length ({len(lines)} lines)"

            selected_lines = lines[start_idx:end_idx]
            return format_content_with_line_numbers(
                selected_lines, start_line=start_idx + 1
            )
        except Exception as e:
            return f"Error reading file '{file_path}': {e!s}"

    async def awrite(self, file_path: str, content: str) -> WriteResult:
        try:
            success = await self.client.write_file(file_path, content)
            if success:
                return WriteResult(path=file_path, files_update=None)
            return WriteResult(error=f"Failed to write file '{file_path}'")
        except Exception as e:
            return WriteResult(error=str(e))

    async def aedit(
        self,
        file_path: str,
        old_string: str,
        new_string: str,
        replace_all: bool = False,
    ) -> EditResult:
        # Note: WorkerClient.edit_file takes a list of EditOp
        from worker.api.schema import EditOp

        try:
            success = await self.client.edit_file(
                file_path, [EditOp(old_string=old_string, new_string=new_string)]
            )
            if success:
                return EditResult(path=file_path, files_update=None, occurrences=1)
            return EditResult(error=f"Failed to edit file '{file_path}'")
        except Exception as e:
            return EditResult(error=str(e))

    async def agrep_raw(
        self, pattern: str, path: str | None = None, glob: str | None = None
    ) -> list[GrepMatch] | str:
        matches = await self.client.grep(pattern, path, glob)
        return [
            {
                "path": m.path,
                "line": m.line,
                "text": m.text,
            }
            for m in matches
        ]

    async def aglob_info(self, pattern: str, path: str = "/") -> list[ProtocolFileInfo]:
        return await self.als_info(path)

    async def aupload_files(
        self, files: list[tuple[str, bytes]]
    ) -> list[FileUploadResponse]:
        responses = []
        for path, content in files:
            try:
                success = await self.client.upload_file(path, content)
                responses.append(
                    FileUploadResponse(path=path, error=None if success else "failed")
                )
            except Exception as e:
                responses.append(FileUploadResponse(path=path, error=str(e)))
        return responses

    async def adownload_files(self, paths: list[str]) -> list[FileDownloadResponse]:
        responses = []
        for path in paths:
            try:
                content = await self.client.read_file_binary(path)
                responses.append(
                    FileDownloadResponse(
                        path=path, content=content, error=None
                    )
                )
            except Exception as e:
                responses.append(FileDownloadResponse(path=path, error=str(e)))
        return responses

    # --- Sync Fallbacks ---

    def _run_sync(self, coro):
        try:
            loop = asyncio.get_event_loop()
            if loop.is_running():
                return asyncio.run_coroutine_threadsafe(coro, loop).result()
            return asyncio.run(coro)
        except RuntimeError:
            return asyncio.run(coro)

    def ls_info(self, path: str) -> list[ProtocolFileInfo]:
        return self._run_sync(self.als_info(path))

    def read(self, file_path: str, offset: int = 0, limit: int = 2000) -> str:
        return self._run_sync(self.aread(file_path, offset, limit))

    def write(self, file_path: str, content: str) -> WriteResult:
        return self._run_sync(self.awrite(file_path, content))

    def edit(
        self,
        file_path: str,
        old_string: str,
        new_string: str,
        replace_all: bool = False,
    ) -> EditResult:
        return self._run_sync(
            self.aedit(file_path, old_string, new_string, replace_all)
        )

    def grep_raw(
        self, pattern: str, path: str | None = None, glob: str | None = None
    ) -> list[GrepMatch] | str:
        return self._run_sync(self.agrep_raw(pattern, path, glob))

    def glob_info(self, pattern: str, path: str = "/") -> list[ProtocolFileInfo]:
        return self._run_sync(self.aglob_info(pattern, path))

    def upload_files(self, files: list[tuple[str, bytes]]) -> list[FileUploadResponse]:
        return self._run_sync(self.aupload_files(files))

    def download_files(self, paths: list[str]) -> list[FileDownloadResponse]:
        return self._run_sync(self.adownload_files(paths))
