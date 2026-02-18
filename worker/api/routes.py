import asyncio
import multiprocessing
from concurrent.futures import ProcessPoolExecutor
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
from worker.utils.assets import get_media_type, validate_asset_source
from worker.utils.loader import load_component_from_script
from worker.utils.persistence import (
    collect_and_cleanup_events,
    record_validation_result,
)
from worker.utils.validation import validate_fem_manufacturability
from worker.workbenches.models import WorkbenchResult

from ..filesystem.backend import FileInfo
from ..filesystem.router import WritePermissionError, create_filesystem_router
from ..runtime.executor import RuntimeConfig, run_python_code_async
from ..utils import submit_for_review, validate
from ..utils.git import (
    abort_merge,
    commit_all,
    complete_merge,
    get_repo_status,
    init_workspace_repo,
    resolve_conflict_ours,
    resolve_conflict_theirs,
)
from ..utils.preview import preview_design
from .schema import (
    AnalyzeRequest,
    BenchmarkToolRequest,
    BenchmarkToolResponse,
    DeleteFileRequest,
    EditFileRequest,
    ExecuteRequest,
    ExecuteResponse,
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
    PreviewDesignRequest,
    PreviewDesignResponse,
    ReadFileRequest,
    ReadFileResponse,
    StatusResponse,
    WriteFileRequest,
)

logger = structlog.get_logger(__name__)
router = APIRouter()


async def get_router(x_session_id: str = Header(...)):
    """Dependency to create a filesystem router for the current session."""
    try:
        return create_filesystem_router(session_id=x_session_id)
    except Exception as e:
        logger.error("router_creation_failed", error=str(e))
        raise HTTPException(status_code=500, detail="Failed to initialize filesystem")


@router.post("/topology/inspect", response_model=InspectTopologyResponse)
async def api_inspect_topology(
    request: InspectTopologyRequest,
    fs_router=Depends(get_router),
):
    """Inspect topological features of the component."""
    try:
        from worker.tools.topology import inspect_topology

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


@router.post("/fs/ls", response_model=list[FileInfo])
async def list_files(request: ListFilesRequest, fs_router=Depends(get_router)):
    """List contents of a directory."""
    try:
        return fs_router.ls(request.path)
    except FileNotFoundError:
        raise HTTPException(status_code=404, detail="Directory not found")
    except Exception as e:
        logger.error("api_ls_failed", path=request.path, error=str(e))
        raise HTTPException(status_code=500, detail=str(e))


@router.post("/fs/read", response_model=ReadFileResponse)
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


@router.post("/fs/write", response_model=StatusResponse)
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


@router.post("/fs/edit", response_model=StatusResponse)
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


@router.post("/fs/upload_file", response_model=StatusResponse)
async def upload_file(
    path: str = Form(...),
    file: UploadFile = File(...),
    fs_router=Depends(get_router),
):
    """Upload a file with binary content."""
    try:
        content = await file.read()
        # fs_router.upload_files expects list of (path, content)
        responses = fs_router.upload_files([(path, content)])

        # Check if any error occurred
        if responses and responses[0].error:
            raise HTTPException(status_code=403, detail=responses[0].error)

        return StatusResponse(status=ResponseStatus.SUCCESS)
    except HTTPException:
        raise
    except Exception as e:
        logger.error("api_upload_file_failed", path=path, error=str(e))
        raise HTTPException(status_code=500, detail=str(e))


@router.post("/fs/read_blob")
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


@router.post("/fs/grep", response_model=list[GrepMatch])
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


@router.post("/fs/delete", response_model=StatusResponse)
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


@router.post("/git/init", response_model=StatusResponse)
async def git_init(fs_router=Depends(get_router)):
    """Initialize a git repository in the workspace."""
    try:
        init_workspace_repo(fs_router.local_backend.root)
        return StatusResponse(status=ResponseStatus.SUCCESS)
    except Exception as e:
        logger.error("api_git_init_failed", error=str(e))
        raise HTTPException(status_code=500, detail=str(e))


@router.post("/git/commit", response_model=GitCommitResponse)
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


@router.get("/git/status", response_model=GitStatusResponse)
async def git_status(fs_router=Depends(get_router)):
    """Get repository status."""
    return get_repo_status(fs_router.local_backend.root)


@router.post("/git/resolve", response_model=StatusResponse)
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


@router.post("/git/merge/abort", response_model=StatusResponse)
async def git_abort(fs_router=Depends(get_router)):
    """Abort a merge."""
    if abort_merge(fs_router.local_backend.root):
        return StatusResponse(status=ResponseStatus.SUCCESS)
    raise HTTPException(status_code=500, detail="Failed to abort merge")


@router.post("/git/merge/complete", response_model=GitCommitResponse)
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


@router.post("/runtime/execute", response_model=ExecuteResponse)
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


# Global lock to ensure only one simulation/render runs at a time cross-session
HEAVY_OPERATION_LOCK = asyncio.Lock()
SIMULATION_QUEUE_DEPTH = 0


def _init_genesis_worker():
    """Pre-warm Genesis in the worker process."""
    try:
        import genesis as gs
        import torch

        # Basic init
        has_gpu = torch.cuda.is_available()
        gs.init(backend=gs.gpu if has_gpu else gs.cpu, logging_level="warning")

        # Build a tiny scene to trigger kernel compilation
        scene = gs.Scene(show_viewer=False)
        scene.add_entity(gs.morphs.Plane())
        scene.build()

        logger.info(
            "genesis_worker_prewarmed", pid=multiprocessing.current_process().pid
        )
    except Exception as e:
        logger.warning("genesis_worker_prewarm_failed", error=str(e))


# Use 'spawn' context for true isolation, as Genesis/Torch/Vulkan hate fork
SIMULATION_EXECUTOR = ProcessPoolExecutor(
    max_workers=1,
    max_tasks_per_child=None,
    mp_context=multiprocessing.get_context("spawn"),
    initializer=_init_genesis_worker,
)


@router.post("/benchmark/simulate", response_model=BenchmarkToolResponse)
async def api_simulate(
    request: BenchmarkToolRequest,
    x_session_id: str = Header(...),
    fs_router=Depends(get_router),
):
    """Physics-backed stability check in isolated session."""
    global SIMULATION_QUEUE_DEPTH
    SIMULATION_QUEUE_DEPTH += 1
    wait_pos = SIMULATION_QUEUE_DEPTH
    try:
        if wait_pos > 4:
            logger.info(
                "simulation_queued",
                session_id=x_session_id,
                queue_depth=wait_pos,
                message=f"the simulation queue has {wait_pos - 4} members in it for your host, please wait",
            )

        from shared.simulation.schemas import SimulatorBackendType
        from worker.utils.validation import simulate_subprocess

        # Determine backend type
        backend_type = request.backend
        if isinstance(backend_type, str):
            backend_type = SimulatorBackendType(backend_type)

        loop = asyncio.get_running_loop()
        # Run in isolated subprocess to bypass Genesis global state issues
        result = await loop.run_in_executor(
            SIMULATION_EXECUTOR,
            simulate_subprocess,
            fs_router.local_backend._resolve(request.script_path),
            fs_router.local_backend.root,
            request.script_content,
            fs_router.local_backend.root,
            request.smoke_test_mode,
            backend_type,
            x_session_id,
        )

        # Reconstruct model if needed
        render_paths = result.render_paths
        mjcf_content = result.mjcf_content
        stress_summaries = [s.model_dump() for s in result.stress_summaries]
        fluid_metrics = [m.model_dump() for m in result.fluid_metrics]
        summary = result.summary
        if wait_pos > 1:
            summary = (
                f"{summary} (Queued: wait position {wait_pos})"
                if summary
                else f"Queued: wait position {wait_pos}"
            )
        success = result.success
        confidence = result.confidence

        events = _collect_events(fs_router)
        return BenchmarkToolResponse(
            success=success,
            message=summary,
            confidence=confidence,
            artifacts={
                "render_paths": render_paths,
                "mjcf_content": mjcf_content,
                "stress_summaries": stress_summaries,
                "fluid_metrics": fluid_metrics,
            },
            events=events,
        )

    except Exception as e:
        logger.error("api_benchmark_simulate_failed", error=str(e))
        return BenchmarkToolResponse(success=False, message=str(e))
    finally:
        SIMULATION_QUEUE_DEPTH -= 1


@router.post("/benchmark/validate", response_model=BenchmarkToolResponse)
async def api_validate(
    request: BenchmarkToolRequest,
    x_session_id: str = Header(...),
    fs_router=Depends(get_router),
):
    """Geometric validity check in isolated session."""
    global SIMULATION_QUEUE_DEPTH
    SIMULATION_QUEUE_DEPTH += 1
    wait_pos = SIMULATION_QUEUE_DEPTH
    try:
        if wait_pos > 1:
            logger.info(
                "validation_queued",
                session_id=x_session_id,
                queue_depth=wait_pos,
                message=f"the simulation queue has {wait_pos - 1} members in it for your host, please wait",
            )
        async with HEAVY_OPERATION_LOCK:
            component = load_component_from_script(
                script_path=fs_router.local_backend._resolve(request.script_path),
                session_root=fs_router.local_backend.root,
                script_content=request.script_content,
            )
            # Geometric validation
            is_valid, message = await asyncio.to_thread(
                validate,
                component,
                output_dir=fs_router.local_backend.root,
                session_id=x_session_id,
                smoke_test_mode=request.smoke_test_mode,
            )

            # INT-102: Fetch objectives to check if FEM material validation is required
            fem_valid, fem_msg = await asyncio.to_thread(
                validate_fem_manufacturability,
                component,
                fs_router.local_backend.root,
            )
            if is_valid and not fem_valid:
                is_valid = False
                message = (message + "; " + fem_msg) if message else fem_msg

        # INT-018: Record validation results to satisfy the handover gate
        record_validation_result(fs_router.local_backend.root, is_valid, message)

        if wait_pos > 1:
            message = (
                f"{message} (Queued: wait position {wait_pos})"
                if message
                else f"Queued: wait position {wait_pos}"
            )

        events = _collect_events(fs_router)
        return BenchmarkToolResponse(
            success=is_valid,
            message=message or "Validation successful",
            events=events,
        )

    except Exception as e:
        logger.error("api_benchmark_validate_failed", error=str(e))
        return BenchmarkToolResponse(success=False, message=str(e))
    finally:
        SIMULATION_QUEUE_DEPTH -= 1


@router.post("/benchmark/analyze", response_model=WorkbenchResult)
async def api_analyze(
    request: AnalyzeRequest,
    x_session_id: str = Header(...),
    fs_router=Depends(get_router),
):
    """Component analysis (topology, material, weight) in isolated session."""
    global SIMULATION_QUEUE_DEPTH
    SIMULATION_QUEUE_DEPTH += 1
    wait_pos = SIMULATION_QUEUE_DEPTH
    try:
        if wait_pos > 1:
            logger.info(
                "analysis_queued",
                session_id=x_session_id,
                queue_depth=wait_pos,
                message=f"the simulation queue has {wait_pos - 1} members in it for your host, please wait",
            )
        async with HEAVY_OPERATION_LOCK:
            # Component analysis logic
            from worker.utils.topology import analyze_component

            component = load_component_from_script(
                script_path=fs_router.local_backend._resolve(request.script_path),
                session_root=fs_router.local_backend.root,
                script_content=request.script_content,
            )
            result = await asyncio.to_thread(
                analyze_component,
                component,
                output_dir=fs_router.local_backend.root,
            )

            # [NEW] Wrap message with queue info
            msg = result.message
            if wait_pos > 1:
                msg = (
                    f"{msg} (Queued: wait position {wait_pos})"
                    if msg
                    else f"Queued: wait position {wait_pos}"
                )
            result.message = msg

            return result
    except Exception as e:
        logger.error("api_benchmark_analyze_failed", error=str(e))
        return WorkbenchResult(success=False, message=str(e))
    finally:
        SIMULATION_QUEUE_DEPTH -= 1


@router.post("/benchmark/submit", response_model=BenchmarkToolResponse)
async def api_submit(
    request: BenchmarkToolRequest,
    fs_router=Depends(get_router),
):
    """Handover to reviewer in isolated session."""
    try:
        component = load_component_from_script(
            script_path=fs_router.local_backend._resolve(request.script_path),
            session_root=fs_router.local_backend.root,
            script_content=request.script_content,
        )
        success = submit_for_review(component, cwd=fs_router.local_backend.root)
        events = _collect_events(fs_router)
        return BenchmarkToolResponse(
            success=success,
            message="Handover complete" if success else "Handover failed",
            events=events,
        )

    except Exception as e:
        logger.error("api_benchmark_submit_failed", error=str(e))
        return BenchmarkToolResponse(success=False, message=str(e))


@router.get("/assets/{path:path}")
async def get_asset(path: str, fs_router=Depends(get_router)):
    """Serve assets from the filesystem."""
    try:
        # Check source code if requesting a model
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


@router.post("/benchmark/preview", response_model=PreviewDesignResponse)
async def api_preview(
    request: PreviewDesignRequest,
    x_session_id: str = Header(...),
    fs_router=Depends(get_router),
):
    """Render a preview of the CAD design from specified camera angles."""
    global SIMULATION_QUEUE_DEPTH
    SIMULATION_QUEUE_DEPTH += 1
    wait_pos = SIMULATION_QUEUE_DEPTH
    try:
        if wait_pos > 1:
            logger.info(
                "preview_queued",
                session_id=x_session_id,
                queue_depth=wait_pos,
                message=f"the simulation queue has {wait_pos - 1} members in it for your host, please wait",
            )

        component = load_component_from_script(
            script_path=fs_router.local_backend._resolve(request.script_path),
            session_root=fs_router.local_backend.root,
        )

        async with HEAVY_OPERATION_LOCK:
            image_path = await asyncio.to_thread(
                preview_design,
                component,
                pitch=request.pitch,
                yaw=request.yaw,
                output_dir=fs_router.local_backend.root / "renders",
            )
        events = _collect_events(fs_router)

        message = "Preview generated successfully"
        if wait_pos > 1:
            message = f"{message} (Queued: wait position {wait_pos})"

        return PreviewDesignResponse(
            success=True,
            message=message,
            image_path=str(image_path.relative_to(fs_router.local_backend.root)),
            events=events,
        )

    except Exception as e:
        logger.error("api_benchmark_preview_failed", error=str(e))
        return PreviewDesignResponse(success=False, message=str(e))
    finally:
        SIMULATION_QUEUE_DEPTH -= 1


@router.post("/lint", response_model=LintResponse)
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
                    # Errors are typically E* codes, warnings are W* codes
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


@router.post("/benchmark/build", response_model=BenchmarkToolResponse)
async def api_build(
    request: PreviewDesignRequest,
    x_session_id: str = Header(...),
    fs_router=Depends(get_router),
):
    """Rebuild simulation assets (GLB) from source without running full simulation."""
    global SIMULATION_QUEUE_DEPTH
    SIMULATION_QUEUE_DEPTH += 1
    wait_pos = SIMULATION_QUEUE_DEPTH
    try:
        if wait_pos > 1:
            logger.info(
                "build_queued",
                session_id=x_session_id,
                queue_depth=wait_pos,
                message=f"the simulation queue has {wait_pos - 1} members in it for your host, please wait",
            )
        async with HEAVY_OPERATION_LOCK:
            # Load component
            component = load_component_from_script(
                script_path=fs_router.local_backend._resolve(request.script_path),
                session_root=fs_router.local_backend.root,
                script_content=request.script_content,
            )

            # Get builder
            from worker.simulation.factory import get_simulation_builder

            builder = get_simulation_builder(fs_router.local_backend.root)

            # Build assets (GLB/OBJ/Scene)
            # We don't need objectives/moving_parts for basic visualization rebuild
            scene_path = await asyncio.to_thread(
                builder.build_from_assembly,
                component,
                smoke_test_mode=request.smoke_test_mode,
            )

        events = _collect_events(fs_router)
        message = f"Assets rebuilt. Scene saved to {scene_path.name}"
        if wait_pos > 1:
            message = f"{message} (Queued: wait position {wait_pos})"

        return BenchmarkToolResponse(
            success=True,
            message=message,
            artifacts={
                # Return paths relative to session root
                "scene_path": str(scene_path.relative_to(fs_router.local_backend.root))
            },
            events=events,
        )

    except Exception as e:
        logger.error("api_benchmark_build_failed", error=str(e))
        return BenchmarkToolResponse(success=False, message=str(e))
    finally:
        SIMULATION_QUEUE_DEPTH -= 1
