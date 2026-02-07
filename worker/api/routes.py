import ast
import importlib.util
import os
import sys
from pathlib import Path

import structlog
from fastapi import APIRouter, Depends, Header, HTTPException, Response

from shared.enums import ResponseStatus

from ..filesystem.backend import FileInfo
from ..filesystem.router import WritePermissionError, create_filesystem_router
from ..runtime.executor import RuntimeConfig, run_python_code_async
from ..utils import simulate, submit_for_review, validate
from .schema import (
    BenchmarkToolRequest,
    BenchmarkToolResponse,
    EditFileRequest,
    ExecuteRequest,
    ExecuteResponse,
    ListFilesRequest,
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


def _get_session_dir(session_id: str) -> Path:
    """Get or create a local temporary directory for the session."""
    base_dir = Path("/tmp/problemologist/sessions")
    session_dir = base_dir / session_id
    session_dir.mkdir(parents=True, exist_ok=True)
    return session_dir


async def _sync_to_local(fs_router, session_dir: Path):
    """Synchronize files from S3/FS_ROUTER to the local session directory."""
    try:
        files = fs_router.ls("/")
        for file_info in files:
            if not file_info.is_dir:
                try:
                    content = fs_router.read(file_info.path)

                    rel_path = file_info.path.lstrip("/")
                    local_file_path = session_dir / rel_path
                    local_file_path.parent.mkdir(parents=True, exist_ok=True)

                    mode = "wb" if isinstance(content, bytes) else "w"
                    with open(local_file_path, mode) as f:
                        f.write(content)
                except Exception as e:
                    logger.warning(
                        "session_sync_file_failed", path=file_info.path, error=str(e)
                    )
    except Exception as e:
        logger.error("session_sync_failed", error=str(e))


def _load_component(script_path: str, script_content: str | None = None):
    """Utility to load the component from the agent's script."""
    # 1. If content is provided, use it directly (stateless/memory execution)
    if script_content:
        local_scope = {}
        try:
            exec(script_content, local_scope)
            build_func = local_scope.get("build")
            if not build_func:
                for val in local_scope.values():
                    if callable(val) and getattr(val, "__name__", "") == "build":
                        build_func = val
                        break

            if build_func:
                return build_func()
            raise AttributeError("build() function not found in script content.")
        except Exception as e:
            raise RuntimeError(f"Failed to execute script content: {e}")

    # 2. Fallback to file path loading
    path = Path(script_path)

    # We expect the caller to provide an absolute path (mapped to session dir)
    # But if not, we fallback to CWD for backward compatibility
    if not path.is_absolute():
        path = Path.cwd() / script_path

    if not path.exists():
        raise FileNotFoundError(
            f"Script not found at {path}. "
            "Ensure the agent has written the file before calling simulate/validate."
        )

    # Add script's directory to sys.path
    script_dir = str(path.parent)
    if script_dir not in sys.path:
        sys.path.insert(0, script_dir)

    spec = importlib.util.spec_from_file_location("dynamic_build", str(path))
    if spec is None or spec.loader is None:
        raise ImportError(f"Could not load spec from {path}")

    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)

    if hasattr(module, "build"):
        return module.build()
    raise AttributeError("build() function not found in script.")


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
        fs_router.write(request.path, request.content)
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


@router.post("/runtime/execute", response_model=ExecuteResponse)
async def execute_code(
    request: ExecuteRequest,
    fs_router=Depends(get_router),
    x_session_id: str = Header(...),
):
    """Execute Python code in session-isolated environment."""
    session_dir = _get_session_dir(x_session_id)
    await _sync_to_local(fs_router, session_dir)

    config = RuntimeConfig(
        timeout_seconds=request.timeout,
        working_directory=str(session_dir),
    )
    result = await run_python_code_async(code=request.code, config=config)

    return ExecuteResponse(
        stdout=result.stdout,
        stderr=result.stderr,
        exit_code=result.exit_code,
        timed_out=result.timed_out,
    )


import asyncio

# Global semaphore to limit concurrent simulations
# Spec: "only one can simulate"
SIMULATION_SEMAPHORE = asyncio.Semaphore(1)


@router.post("/benchmark/simulate", response_model=BenchmarkToolResponse)
async def api_simulate(
    request: BenchmarkToolRequest,
    fs_router=Depends(get_router),
    x_session_id: str = Header(...),
):
    """Physics-backed stability check in isolated session."""
    try:
        session_dir = _get_session_dir(x_session_id)
        await _sync_to_local(fs_router, session_dir)

        script_path = Path(request.script_path)
        if not script_path.is_absolute():
            script_path = session_dir / script_path

        component = _load_component(str(script_path), request.script_content)

        # Enforce single concurrent simulation
        async with SIMULATION_SEMAPHORE:
            # We run simulate (sync) in a threadpool, but the semaphore prevents conflicting
            # concurrent runs. Ideally we'd wrap simulate in run_in_executor but FastAPI does it.
            # However, holding an async lock while running blocking code in the *simultaneous*
            # handling is tricky if not awaiting.
            # Since `simulate` is blocking, we should ideally run it in a thread and await it.
            # But the semaphore is async.
            result = await asyncio.to_thread(
                simulate, component, output_dir=session_dir
            )

        return BenchmarkToolResponse(
            success=result.success,
            message=result.summary,
            artifacts={
                "render_paths": result.render_paths,
                "mjcf_content": result.mjcf_content,
            },
        )
    except Exception as e:
        logger.error("api_benchmark_simulate_failed", error=str(e))
        return BenchmarkToolResponse(success=False, message=str(e))


@router.post("/benchmark/validate", response_model=BenchmarkToolResponse)
async def api_validate(
    request: BenchmarkToolRequest,
    fs_router=Depends(get_router),
    x_session_id: str = Header(...),
):
    """Geometric validity check in isolated session."""
    try:
        session_dir = _get_session_dir(x_session_id)
        await _sync_to_local(fs_router, session_dir)

        script_path = Path(request.script_path)
        if not script_path.is_absolute():
            script_path = session_dir / script_path

        component = _load_component(str(script_path), request.script_content)
        is_valid = validate(component)
        return BenchmarkToolResponse(
            success=is_valid,
            message="Validation successful" if is_valid else "Validation failed",
        )
    except Exception as e:
        logger.error("api_benchmark_validate_failed", error=str(e))
        return BenchmarkToolResponse(success=False, message=str(e))


@router.post("/benchmark/submit", response_model=BenchmarkToolResponse)
async def api_submit(
    request: BenchmarkToolRequest,
    fs_router=Depends(get_router),
    x_session_id: str = Header(...),
):
    """Handover to reviewer in isolated session."""
    try:
        session_dir = _get_session_dir(x_session_id)
        await _sync_to_local(fs_router, session_dir)

        script_path = Path(request.script_path)
        if not script_path.is_absolute():
            script_path = session_dir / script_path

        component = _load_component(str(script_path), request.script_content)
        success = submit_for_review(component)
        return BenchmarkToolResponse(
            success=success,
            message="Handover complete" if success else "Handover failed",
        )
    except Exception as e:
        logger.error("api_benchmark_submit_failed", error=str(e))
        return BenchmarkToolResponse(success=False, message=str(e))


@router.get("/assets/{path:path}")
async def get_asset(path: str, fs_router=Depends(get_router)):
    """Serve assets from the filesystem."""
    try:
        # Check source code if requesting a model
        if path.endswith(".glb") or path.endswith(".stl"):
            # Try to find the source python file
            # Heuristic: file with same name in root, or 'main.py'
            candidate_paths = [
                Path(path).with_suffix(".py").name,
                "main.py",
                "component.py",
                "solution.py",
            ]

            for py_path in candidate_paths:
                if fs_router.exists(py_path):
                    try:
                        source_code = fs_router.read(py_path)
                        ast.parse(source_code)
                        break  # Found valid source, or at least one exists
                    except SyntaxError:
                        logger.warning(
                            "asset_serving_refused_syntax_error",
                            asset=path,
                            source=py_path,
                        )
                        raise HTTPException(
                            status_code=422,
                            detail=f"Source code {py_path} has syntax errors.",
                        )
                    except Exception as e:
                        logger.warning("asset_source_check_failed", error=str(e))
                        # Don't block if we stick to heuristic, but maybe we should?
                        # User asked for "red (linting) errors". SyntaxError is definitely red.
                        pass

        content = fs_router.read(path)
        media_type = "application/octet-stream"
        if path.endswith(".glb"):
            media_type = "model/gltf-binary"
        elif path.endswith(".py"):
            media_type = "text/x-python"
        elif path.endswith(".stl"):
            media_type = "model/stl"

        return Response(content=content, media_type=media_type)
    except FileNotFoundError:
        raise HTTPException(status_code=404, detail="Asset not found")
    except HTTPException:
        raise
    except Exception as e:
        logger.error("api_asset_failed", path=path, error=str(e))
        raise HTTPException(status_code=500, detail=str(e))
