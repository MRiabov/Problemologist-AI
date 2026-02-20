import asyncio
from pathlib import Path
from typing import Any

import structlog
from fastapi import (
    APIRouter,
    Depends,
    File,
    Form,
    Header,
    HTTPException,
    Response,
    UploadFile,
)

from shared.enums import ResponseStatus
from shared.workers.persistence import collect_and_cleanup_events
from shared.workers.schema import (
    DeleteFileRequest,
    EditFileRequest,
    ExecuteRequest,
    ExecuteResponse,
    ExistsResponse,
    GitCommitRequest,
    GitCommitResponse,
    GitMergeRequest,
    GitResolveRequest,
    GitStatusResponse,
    GrepMatch,
    GrepRequest,
    InspectTopologyRequest,
    InspectTopologyResponse,
    LintRequest,
    LintResponse,
    ListFilesRequest,
    ReadFileRequest,
    ReadFileResponse,
    StatusResponse,
    WriteFileRequest,
)
from shared.workers.filesystem.backend import FileInfo
from shared.workers.filesystem.router import (
    WritePermissionError,
    create_filesystem_router,
)
from worker_light.runtime.executor import RuntimeConfig, run_python_code_async
from worker_light.utils.assets import get_media_type, validate_asset_source
from worker_light.utils.git import (
    abort_merge,
    commit_all,
    complete_merge,
    get_repo_status,
    init_workspace_repo,
    resolve_conflict_ours,
    resolve_conflict_theirs,
)

logger = structlog.get_logger(__name__)
light_router = APIRouter()


async def get_router(x_session_id: str = Header(...)):
    """Dependency to create a filesystem router for the current session."""
    try:
        from worker_light.config import settings

        return create_filesystem_router(
            session_id=x_session_id, base_dir=settings.sessions_dir
        )
    except Exception as e:
        logger.error("router_creation_failed", error=str(e))
        raise HTTPException(status_code=500, detail="Failed to initialize filesystem")


@light_router.post("/topology/inspect", response_model=InspectTopologyResponse)
async def api_inspect_topology(
    request: InspectTopologyRequest,
    fs_router=Depends(get_router),
):
    """Inspect topological features of the component."""
    try:
        from worker_light.tools.topology import inspect_topology

        # Resolve script path
        try:
            local_p = fs_router.local_backend._resolve(request.script_path)
        except Exception:
            local_p = Path(request.script_path)

        props = inspect_topology(target_id=request.target_id, script_path=str(local_p))
        return InspectTopologyResponse(success=True, properties=props)
    except Exception as e:
        logger.error("api_inspect_topology_failed", error=str(e))
        return InspectTopologyResponse(success=False, message=str(e))


def _collect_events(fs_router) -> list[dict[str, Any]]:
    """Read and delete events.jsonl from the workspace."""
    return collect_and_cleanup_events(fs_router.local_backend.root)


@light_router.post("/fs/ls", response_model=list[FileInfo])
async def list_files(request: ListFilesRequest, fs_router=Depends(get_router)):
    """List contents of a directory."""
    try:
        return fs_router.ls(request.path)
    except FileNotFoundError:
        raise HTTPException(status_code=404, detail="Directory not found")
    except Exception as e:
        logger.error("api_ls_failed", path=request.path, error=str(e))
        raise HTTPException(status_code=500, detail=str(e))


@light_router.post("/fs/exists", response_model=ExistsResponse)
async def file_exists(request: ReadFileRequest, fs_router=Depends(get_router)):
    """Check if a file or directory exists."""
    try:
        exists = fs_router.exists(request.path)
        return ExistsResponse(exists=exists)
    except Exception as e:
        logger.error("api_exists_failed", path=request.path, error=str(e))
        raise HTTPException(status_code=500, detail=str(e))


@light_router.post("/fs/read", response_model=ReadFileResponse)
async def read_file(request: ReadFileRequest, fs_router=Depends(get_router)):
    """Read file contents."""
    try:
        content = fs_router.read(request.path)
        return ReadFileResponse(content=content.decode("utf-8"))
    except FileNotFoundError:
        raise HTTPException(status_code=404, detail="File not found")
    except Exception as e:
        logger.error("api_read_failed", path=request.path, error=str(e))
        raise HTTPException(status_code=500, detail=str(e))


@light_router.post("/fs/write", response_model=StatusResponse)
async def write_file(request: WriteFileRequest, fs_router=Depends(get_router)):
    """Write content to a file."""
    try:
        fs_router.write(request.path, request.content, overwrite=request.overwrite)
        return StatusResponse(status=ResponseStatus.SUCCESS)
    except WritePermissionError as e:
        raise HTTPException(status_code=403, detail=str(e))
    except Exception as e:
        logger.error("api_write_failed", path=request.path, error=str(e))
        raise HTTPException(status_code=500, detail=str(e))


@light_router.post("/fs/edit", response_model=StatusResponse)
async def edit_file(request: EditFileRequest, fs_router=Depends(get_router)):
    """Edit a file with one or more operations."""
    try:
        # Check if file exists first
        if not fs_router.exists(request.path):
            raise HTTPException(status_code=404, detail="File not found")

        # Apply each edit operation
        for edit in request.edits:
            success = fs_router.edit(request.path, edit.old_string, edit.new_string)
            if not success:
                raise HTTPException(
                    status_code=400,
                    detail=f"Content not found for replacement: {edit.old_string[:50]}...",
                )

        return StatusResponse(status=ResponseStatus.SUCCESS)
    except WritePermissionError as e:
        raise HTTPException(status_code=403, detail=str(e))
    except HTTPException:
        raise
    except Exception as e:
        logger.error("api_edit_failed", path=request.path, error=str(e))
        raise HTTPException(status_code=500, detail=str(e))


@light_router.post("/fs/upload_file", response_model=StatusResponse)
async def upload_file(
    path: str = Form(...),
    file: UploadFile = File(...),
    fs_router=Depends(get_router),
):
    """Upload a file with binary content."""
    try:
        content = await file.read()
        responses = fs_router.upload_files([(path, content)])

        if responses and responses[0].error:
            raise HTTPException(status_code=403, detail=responses[0].error)

        return StatusResponse(status=ResponseStatus.SUCCESS)
    except HTTPException:
        raise
    except Exception as e:
        logger.error("api_upload_file_failed", path=path, error=str(e))
        raise HTTPException(status_code=500, detail=str(e))


@light_router.post("/fs/read_blob")
async def read_blob(request: ReadFileRequest, fs_router=Depends(get_router)):
    """Read file contents as binary blob."""
    try:
        content = fs_router.read(request.path)
        return Response(content=content, media_type="application/octet-stream")
    except FileNotFoundError:
        raise HTTPException(status_code=404, detail="File not found")
    except Exception as e:
        logger.error("api_read_blob_failed", path=request.path, error=str(e))
        raise HTTPException(status_code=500, detail=str(e))


@light_router.post("/fs/grep", response_model=list[GrepMatch])
async def api_grep(request: GrepRequest, fs_router=Depends(get_router)):
    """Search for a pattern in files."""
    try:
        matches = fs_router.grep_raw(
            pattern=request.pattern, path=request.path, glob=request.glob
        )
        if isinstance(matches, str):
            raise HTTPException(status_code=400, detail=matches)
        return matches
    except HTTPException:
        raise
    except Exception as e:
        logger.error("api_grep_failed", pattern=request.pattern, error=str(e))
        raise HTTPException(status_code=500, detail=str(e))


@light_router.post("/fs/bundle")
async def bundle_session(fs_router=Depends(get_router)):
    """Bundle the session workspace into a gzipped tarball."""
    import io
    import tarfile

    root = fs_router.local_backend.root

    buf = io.BytesIO()
    with tarfile.open(fileobj=buf, mode="w:gz") as tar:
        # Exclude large artifacts and internal git state
        exclude = {"renders", ".git", "__pycache__", "assets"}

        for path in root.rglob("*"):
            rel_p = path.relative_to(root)
            if any(part in exclude for part in rel_p.parts):
                continue
            if path.is_file():
                tar.add(path, arcname=str(rel_p))

    return Response(
        content=buf.getvalue(),
        media_type="application/x-gzip",
        headers={"Content-Disposition": "attachment; filename=session.tar.gz"},
    )


@light_router.post("/fs/delete", response_model=StatusResponse)
async def delete_file(request: DeleteFileRequest, fs_router=Depends(get_router)):
    """Delete a file or directory."""
    try:
        fs_router.delete(request.path)
        return StatusResponse(status=ResponseStatus.SUCCESS)
    except FileNotFoundError:
        raise HTTPException(status_code=404, detail="Path not found")
    except WritePermissionError as e:
        raise HTTPException(status_code=403, detail=str(e)) from e
    except Exception as e:
        logger.error("api_delete_failed", path=request.path, error=str(e))
        raise HTTPException(status_code=500, detail=str(e)) from e


@light_router.post("/git/init", response_model=StatusResponse)
async def git_init(fs_router=Depends(get_router)):
    """Initialize a git repository in the workspace."""
    try:
        init_workspace_repo(fs_router.local_backend.root)
        return StatusResponse(status=ResponseStatus.SUCCESS)
    except Exception as e:
        logger.error("api_git_init_failed", error=str(e))
        raise HTTPException(status_code=500, detail=str(e))


@light_router.post("/git/commit", response_model=GitCommitResponse)
async def git_commit(request: GitCommitRequest, fs_router=Depends(get_router)):
    """Commit changes to the local repository."""
    try:
        commit_hash = commit_all(fs_router.local_backend.root, request.message)

        if commit_hash:
            return GitCommitResponse(
                success=True,
                commit_hash=commit_hash,
                message="Commit successful",
            )
        return GitCommitResponse(
            success=True,
            commit_hash=None,
            message="No changes to commit",
        )
    except Exception as e:
        logger.error("api_git_commit_failed", error=str(e))
        return GitCommitResponse(success=False, message=str(e))


@light_router.get("/git/status", response_model=GitStatusResponse)
async def git_status(fs_router=Depends(get_router)):
    """Get repository status."""
    return get_repo_status(fs_router.local_backend.root)


@light_router.post("/git/resolve", response_model=StatusResponse)
async def git_resolve(request: GitResolveRequest, fs_router=Depends(get_router)):
    """Resolve a merge conflict."""
    path = fs_router.local_backend.root
    success = False
    if request.strategy == "ours":
        success = resolve_conflict_ours(path, request.file_path)
    elif request.strategy == "theirs":
        success = resolve_conflict_theirs(path, request.file_path)

    if success:
        return StatusResponse(status=ResponseStatus.SUCCESS)
    raise HTTPException(status_code=500, detail="Failed to resolve conflict")


@light_router.post("/git/merge/abort", response_model=StatusResponse)
async def git_abort(fs_router=Depends(get_router)):
    """Abort a merge."""
    if abort_merge(fs_router.local_backend.root):
        return StatusResponse(status=ResponseStatus.SUCCESS)
    raise HTTPException(status_code=500, detail="Failed to abort merge")


@light_router.post("/git/merge/complete", response_model=GitCommitResponse)
async def git_complete(request: GitMergeRequest, fs_router=Depends(get_router)):
    """Complete a merge."""
    commit_hash = complete_merge(fs_router.local_backend.root, request.message)
    if commit_hash:
        return GitCommitResponse(
            success=True,
            commit_hash=commit_hash,
            message="Merge completed successfully",
        )
    return GitCommitResponse(
        success=False,
        message="Failed to complete merge (conflicts might remain)",
    )


@light_router.post("/runtime/execute", response_model=ExecuteResponse)
async def execute_code(
    request: ExecuteRequest,
    fs_router=Depends(get_router),
):
    """Execute Python code in session-isolated environment."""
    session_dir = fs_router.local_backend.root

    config = RuntimeConfig(
        timeout_seconds=request.timeout,
        working_directory=str(session_dir),
    )
    result = await run_python_code_async(code=request.code, config=config)
    events = _collect_events(fs_router)

    return ExecuteResponse(
        stdout=result.stdout,
        stderr=result.stderr,
        exit_code=result.exit_code,
        timed_out=result.timed_out,
        events=events,
    )


@light_router.get("/assets/{path:path}")
async def get_asset(path: str, fs_router=Depends(get_router)):
    """Serve assets from the filesystem."""
    try:
        validate_asset_source(fs_router.local_backend.root, path)

        content = fs_router.read(path)
        media_type = get_media_type(path)
        return Response(content=content, media_type=media_type)
    except (FileNotFoundError, IsADirectoryError):
        raise HTTPException(status_code=404, detail="Asset not found")
    except HTTPException:
        raise
    except Exception as e:
        logger.error("api_asset_failed", path=path, error=str(e))
        raise HTTPException(status_code=500, detail=str(e))


@light_router.post("/lint", response_model=LintResponse)
async def api_lint(
    request: LintRequest,
    fs_router=Depends(get_router),
):
    """Lint Python code using ruff."""
    import asyncio
    import json
    import tempfile

    try:
        # Get content either from path or direct content
        if request.path:
            content = fs_router.read(request.path).decode("utf-8")
        elif request.content:
            content = request.content
        else:
            return LintResponse(
                success=False,
                errors=[{"message": "Either path or content must be provided"}],
            )

        # Write to temp file for ruff
        with tempfile.NamedTemporaryFile(mode="w", suffix=".py", delete=False) as tmp:
            tmp.write(content)
            tmp_path = tmp.name

        try:
            # Run ruff check with JSON output asynchronously
            process = await asyncio.create_subprocess_exec(
                "ruff",
                "check",
                "--output-format",
                "json",
                tmp_path,
                stdout=asyncio.subprocess.PIPE,
                stderr=asyncio.subprocess.PIPE,
            )

            try:
                stdout, _ = await asyncio.wait_for(process.communicate(), timeout=30)
            except TimeoutError:
                process.kill()
                await process.wait()
                raise

            errors = []
            warnings = []

            if stdout:
                lint_results = json.loads(stdout.decode())
                for item in lint_results:
                    issue = {
                        "code": item.get("code", ""),
                        "message": item.get("message", ""),
                        "line": item.get("location", {}).get("row", 0),
                        "column": item.get("location", {}).get("column", 0),
                    }
                    if item.get("code", "").startswith(("E", "F")):
                        errors.append(issue)
                    else:
                        warnings.append(issue)

            return LintResponse(
                success=len(errors) == 0,
                errors=errors,
                warnings=warnings,
            )
        finally:
            Path(tmp_path).unlink()

    except FileNotFoundError:
        return LintResponse(
            success=False,
            errors=[{"message": f"File not found: {request.path}"}],
        )
    except Exception as e:
        logger.error("api_lint_failed", error=str(e))
        return LintResponse(
            success=False,
            errors=[{"message": str(e)}],
        )
