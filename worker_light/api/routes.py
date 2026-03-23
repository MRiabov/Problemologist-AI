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
from shared.workers.filesystem.router import (
    WritePermissionError,
    create_filesystem_router,
)
from shared.workers.persistence import collect_and_cleanup_events
from shared.workers.schema import (
    DeleteFileRequest,
    EditFileRequest,
    ExecuteRequest,
    ExecuteResponse,
    ExistsResponse,
    FsFileEntry,
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
from worker_light.runtime.executor import RuntimeConfig, run_command_async
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


def _bypass_enabled(requested: bool, system_header: str | None) -> bool:
    return bool(requested and system_header == "1")


def _sanitize_workspace_alias(text: str, session_dir: Path) -> str:
    """Hide host session roots from tool-visible command output."""
    if not text:
        return text
    session_root = str(session_dir.resolve())
    return text.replace(session_root, "/workspace")


def _is_host_session_absolute_path(path: str, session_dir: Path) -> bool:
    """Detect leaked host paths that point at the current session root."""
    normalized = str(path).strip()
    if not normalized.startswith("/"):
        return False
    session_root = str(session_dir.resolve())
    return normalized == session_root or normalized.startswith(f"{session_root}/")


async def get_router(x_session_id: str = Header(...)):
    """Dependency to create a filesystem router for the current session."""
    try:
        from worker_light.config import settings

        return create_filesystem_router(
            session_id=x_session_id, base_dir=settings.sessions_dir
        )
    except Exception as e:
        logger.warning("router_creation_failed", error=str(e))
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
        logger.warning("api_inspect_topology_failed", error=str(e))
        return InspectTopologyResponse(success=False, message=str(e))


def _collect_events(fs_router) -> list[dict[str, Any]]:
    """Read and delete events.jsonl from the workspace."""
    return collect_and_cleanup_events(fs_router.local_backend.root)


@light_router.post("/fs/ls", response_model=list[FsFileEntry])
async def list_files(
    request: ListFilesRequest,
    fs_router=Depends(get_router),
    x_system_fs_bypass: str | None = Header(default=None, alias="X-System-FS-Bypass"),
):
    """List contents of a directory."""
    try:
        if _bypass_enabled(request.bypass_agent_permissions, x_system_fs_bypass):
            return fs_router.local_backend.ls(request.path)
        return fs_router.ls(request.path)
    except FileNotFoundError:
        raise HTTPException(status_code=404, detail="Directory not found")
    except PermissionError as e:
        raise HTTPException(status_code=403, detail=str(e))
    except Exception as e:
        logger.warning("api_ls_failed", path=request.path, error=str(e))
        raise HTTPException(status_code=500, detail=str(e))


@light_router.post("/fs/exists", response_model=ExistsResponse)
async def file_exists(
    request: ReadFileRequest,
    fs_router=Depends(get_router),
    x_system_fs_bypass: str | None = Header(default=None, alias="X-System-FS-Bypass"),
):
    """Check if a file or directory exists."""
    try:
        if _bypass_enabled(request.bypass_agent_permissions, x_system_fs_bypass):
            return ExistsResponse(exists=fs_router.local_backend.exists(request.path))
        exists = fs_router.exists(request.path)
        return ExistsResponse(exists=exists)
    except PermissionError as e:
        raise HTTPException(status_code=403, detail=str(e))
    except Exception as e:
        logger.warning("api_exists_failed", path=request.path, error=str(e))
        raise HTTPException(status_code=500, detail=str(e))


@light_router.post("/fs/read", response_model=ReadFileResponse)
async def read_file(
    request: ReadFileRequest,
    fs_router=Depends(get_router),
    x_system_fs_bypass: str | None = Header(default=None, alias="X-System-FS-Bypass"),
):
    """Read file contents."""
    try:
        if _bypass_enabled(request.bypass_agent_permissions, x_system_fs_bypass):
            local_path = fs_router.local_backend._resolve(request.path)
            if not local_path.exists():
                raise FileNotFoundError
            return ReadFileResponse(content=local_path.read_text(encoding="utf-8"))
        content = fs_router.read(request.path)
        return ReadFileResponse(content=content.decode("utf-8"))
    except FileNotFoundError:
        raise HTTPException(status_code=404, detail="File not found")
    except PermissionError as e:
        raise HTTPException(status_code=403, detail=str(e))
    except Exception as e:
        logger.warning("api_read_failed", path=request.path, error=str(e))
        raise HTTPException(status_code=500, detail=str(e))


@light_router.post("/fs/write", response_model=StatusResponse)
async def write_file(
    request: WriteFileRequest,
    fs_router=Depends(get_router),
    x_system_fs_bypass: str | None = Header(default=None, alias="X-System-FS-Bypass"),
):
    """Write content to a file."""
    try:
        if _bypass_enabled(request.bypass_agent_permissions, x_system_fs_bypass):
            result = fs_router.local_backend.write(
                request.path, request.content, overwrite=request.overwrite
            )
            if result.error:
                raise HTTPException(status_code=500, detail=result.error)
            return StatusResponse(status=ResponseStatus.SUCCESS)
        fs_router.write(request.path, request.content, overwrite=request.overwrite)
        return StatusResponse(status=ResponseStatus.SUCCESS)
    except (WritePermissionError, PermissionError) as e:
        raise HTTPException(status_code=403, detail=str(e))
    except Exception as e:
        logger.warning("api_write_failed", path=request.path, error=str(e))
        raise HTTPException(status_code=500, detail=str(e))


@light_router.post("/fs/edit", response_model=StatusResponse)
async def edit_file(
    request: EditFileRequest,
    fs_router=Depends(get_router),
    x_system_fs_bypass: str | None = Header(default=None, alias="X-System-FS-Bypass"),
):
    """Edit a file with one or more operations."""
    try:
        session_dir = fs_router.local_backend.root
        if _is_host_session_absolute_path(request.path, session_dir):
            raise HTTPException(
                status_code=400,
                detail=(
                    "Host session absolute paths are not allowed. "
                    "Use workspace-relative paths like 'script.py' or the "
                    "'/workspace/script.py' alias."
                ),
            )

        if _bypass_enabled(request.bypass_agent_permissions, x_system_fs_bypass):
            if not fs_router.local_backend.exists(request.path):
                raise HTTPException(status_code=404, detail="File not found")
            for edit in request.edits:
                result = fs_router.local_backend.edit(
                    request.path, edit.old_string, edit.new_string
                )
                if result.error or (result.occurrences or 0) <= 0:
                    raise HTTPException(
                        status_code=400,
                        detail=f"Content not found for replacement: {edit.old_string[:50]}...",
                    )
            return StatusResponse(status=ResponseStatus.SUCCESS)

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
    except (WritePermissionError, PermissionError) as e:
        raise HTTPException(status_code=403, detail=str(e))
    except HTTPException:
        raise
    except Exception as e:
        logger.warning("api_edit_failed", path=request.path, error=str(e))
        raise HTTPException(status_code=500, detail=str(e))


@light_router.post("/fs/upload_file", response_model=StatusResponse)
async def upload_file(
    path: str = Form(...),
    file: UploadFile = File(...),
    bypass_agent_permissions: bool = Form(False),
    fs_router=Depends(get_router),
    x_system_fs_bypass: str | None = Header(default=None, alias="X-System-FS-Bypass"),
):
    """Upload a file with binary content."""
    try:
        content = await file.read()
        if _bypass_enabled(bypass_agent_permissions, x_system_fs_bypass):
            responses = fs_router.local_backend.upload_files([(path, content)])
        else:
            responses = fs_router.upload_files([(path, content)])

        if responses and responses[0].error:
            raise HTTPException(status_code=403, detail=responses[0].error)

        return StatusResponse(status=ResponseStatus.SUCCESS)
    except HTTPException:
        raise
    except Exception as e:
        logger.warning("api_upload_file_failed", path=path, error=str(e))
        raise HTTPException(status_code=500, detail=str(e))


@light_router.post("/fs/read_blob")
async def read_blob(
    request: ReadFileRequest,
    fs_router=Depends(get_router),
    x_system_fs_bypass: str | None = Header(default=None, alias="X-System-FS-Bypass"),
):
    """Read file contents as binary blob."""
    try:
        if _bypass_enabled(request.bypass_agent_permissions, x_system_fs_bypass):
            local_path = fs_router.local_backend._resolve(request.path)
            if not local_path.exists():
                raise FileNotFoundError
            return Response(
                content=local_path.read_bytes(),
                media_type="application/octet-stream",
            )
        content = fs_router.read(request.path)
        return Response(content=content, media_type="application/octet-stream")
    except FileNotFoundError:
        raise HTTPException(status_code=404, detail="File not found")
    except PermissionError as e:
        raise HTTPException(status_code=403, detail=str(e))
    except Exception as e:
        logger.warning("api_read_blob_failed", path=request.path, error=str(e))
        raise HTTPException(status_code=500, detail=str(e))


@light_router.post("/fs/grep", response_model=list[GrepMatch])
async def api_grep(
    request: GrepRequest,
    fs_router=Depends(get_router),
    x_system_fs_bypass: str | None = Header(default=None, alias="X-System-FS-Bypass"),
):
    """Search for a pattern in files."""
    try:
        if _bypass_enabled(request.bypass_agent_permissions, x_system_fs_bypass):
            matches = fs_router.local_backend.grep_raw(
                pattern=request.pattern, path=request.path, glob=request.glob
            )
        else:
            matches = fs_router.grep_raw(
                pattern=request.pattern, path=request.path, glob=request.glob
            )
        if isinstance(matches, str):
            raise HTTPException(status_code=400, detail=matches)
        return matches
    except PermissionError as e:
        raise HTTPException(status_code=403, detail=str(e))
    except HTTPException:
        raise
    except Exception as e:
        logger.warning("api_grep_failed", pattern=request.pattern, error=str(e))
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
async def delete_file(
    request: DeleteFileRequest,
    fs_router=Depends(get_router),
    x_system_fs_bypass: str | None = Header(default=None, alias="X-System-FS-Bypass"),
):
    """Delete a file or directory."""
    try:
        if _bypass_enabled(request.bypass_agent_permissions, x_system_fs_bypass):
            fs_router.local_backend.delete(request.path)
            return StatusResponse(status=ResponseStatus.SUCCESS)
        fs_router.delete(request.path)
        return StatusResponse(status=ResponseStatus.SUCCESS)
    except FileNotFoundError:
        raise HTTPException(status_code=404, detail="Path not found")
    except (WritePermissionError, PermissionError) as e:
        raise HTTPException(status_code=403, detail=str(e)) from e
    except Exception as e:
        logger.warning("api_delete_failed", path=request.path, error=str(e))
        raise HTTPException(status_code=500, detail=str(e)) from e


@light_router.post("/git/init", response_model=StatusResponse)
async def git_init(fs_router=Depends(get_router)):
    """Initialize a git repository in the workspace."""
    try:
        init_workspace_repo(fs_router.local_backend.root)
        return StatusResponse(status=ResponseStatus.SUCCESS)
    except Exception as e:
        logger.warning("api_git_init_failed", error=str(e))
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
        logger.warning("api_git_commit_failed", error=str(e))
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
async def git_complete(
    request: GitMergeRequest,
    x_session_id: str = Header(...),
    fs_router=Depends(get_router),
):
    """Complete a merge."""
    commit_hash = complete_merge(
        fs_router.local_backend.root, request.message, session_id=x_session_id
    )
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
    x_session_id: str = Header(...),
    fs_router=Depends(get_router),
):
    """Execute a shell command in the session-isolated environment."""
    session_dir = fs_router.local_backend.root

    config = RuntimeConfig(
        timeout_seconds=request.timeout,
        working_directory=str(session_dir),
    )
    from worker_light.config import settings

    # Executed scripts should validate directly against the session workspace
    # rather than re-entering the controller script-tools proxy. The proxy path
    # is for control-plane orchestration, while runtime execution needs the
    # lightweight heavy-worker fallback to observe the exact workspace files
    # written in this session.
    result = await run_command_async(
        command=request.code,
        env={
            "SESSION_ID": x_session_id,
            "EPISODE_ID": request.episode_id or x_session_id,
            "WORKER_HEAVY_URL": settings.worker_heavy_url,
            "CONTROLLER_URL": "",
        },
        config=config,
        session_id=x_session_id,
    )
    events = _collect_events(fs_router)

    return ExecuteResponse(
        stdout=_sanitize_workspace_alias(result.stdout, session_dir),
        stderr=_sanitize_workspace_alias(result.stderr, session_dir),
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
    except PermissionError as e:
        raise HTTPException(status_code=403, detail=str(e))
    except HTTPException:
        raise
    except Exception as e:
        logger.warning("api_asset_failed", path=path, error=str(e))
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
        logger.warning("api_lint_failed", error=str(e))
        return LintResponse(
            success=False,
            errors=[{"message": str(e)}],
        )
