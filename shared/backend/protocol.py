import asyncio
from typing import Any, Protocol, runtime_checkable
from pydantic import BaseModel, Field


class FileInfo(BaseModel):
    path: str
    is_dir: bool
    size: int
    modified_at: str | None = None


class WriteResult(BaseModel):
    path: str | None = None
    error: str | None = None
    files_update: Any | None = None


class EditResult(BaseModel):
    path: str | None = None
    error: str | None = None
    files_update: Any | None = None
    occurrences: int = 0


class FileUploadResponse(BaseModel):
    path: str
    error: str | None = None


class FileDownloadResponse(BaseModel):
    path: str
    content: bytes | None = None
    error: str | None = None


class ExecuteResponse(BaseModel):
    output: str
    exit_code: int | None = None
    truncated: bool = False


class GrepMatch(BaseModel):
    path: str
    line: int
    text: str


@runtime_checkable
class BackendProtocol(Protocol):
    def ls_info(self, path: str) -> list[FileInfo]: ...

    async def als_info(self, path: str) -> list[FileInfo]: ...

    def read(self, file_path: str, offset: int = 0, limit: int = 2000) -> str: ...

    async def aread(
        self, file_path: str, offset: int = 0, limit: int = 2000
    ) -> str: ...

    def write(
        self, file_path: str, content: str, overwrite: bool = False
    ) -> WriteResult: ...

    async def awrite(
        self, file_path: str, content: str, overwrite: bool = False
    ) -> WriteResult: ...

    def edit(
        self,
        file_path: str,
        old_string: str,
        new_string: str,
        replace_all: bool = False,
    ) -> EditResult: ...

    async def aedit(
        self,
        file_path: str,
        old_string: str,
        new_string: str,
        replace_all: bool = False,
    ) -> EditResult: ...

    def grep_raw(
        self, pattern: str, path: str | None = None, glob: str | None = None
    ) -> list[GrepMatch] | str: ...

    async def agrep_raw(
        self, pattern: str, path: str | None = None, glob: str | None = None
    ) -> list[GrepMatch] | str: ...

    def glob_info(self, pattern: str, path: str = "/") -> list[FileInfo]: ...

    async def aglob_info(self, pattern: str, path: str = "/") -> list[FileInfo]: ...

    def upload_files(
        self, files: list[tuple[str, bytes]]
    ) -> list[FileUploadResponse]: ...

    async def aupload_files(
        self, files: list[tuple[str, bytes]]
    ) -> list[FileUploadResponse]: ...

    def download_files(self, paths: list[str]) -> list[FileDownloadResponse]: ...

    async def adownload_files(self, paths: list[str]) -> list[FileDownloadResponse]: ...


class SandboxBackendProtocol(BackendProtocol, Protocol):
    @property
    def id(self) -> str: ...

    def execute(self, command: str) -> ExecuteResponse: ...

    async def aexecute(self, command: str) -> ExecuteResponse: ...
