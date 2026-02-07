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


def _load_component(script_path: str):
    """Utility to load the component from the agent's script."""
    path = Path(script_path)
    if not path.is_absolute():
        # Heuristic: if relative, try relative to root first, then maybe check if it's currently being written
        pass

    if not path.exists():
        cwd = Path.cwd()
        raise FileNotFoundError(
            f"Script not found at {path.absolute()}. Current working directory: {cwd}. "
            "Ensure the agent has written the file before calling simulate/validate."
        )

    # Add current directory to sys.path to allow local imports in the script
    workspace_root = str(Path.cwd())
    if workspace_root not in sys.path:
        sys.path.insert(0, workspace_root)

    spec = importlib.util.spec_from_file_location("dynamic_build", script_path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)

    if hasattr(module, "build"):
        # We assume build() takes no args or uses default seed
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
async def execute_code(request: ExecuteRequest):
    """Execute Python code."""
    config = RuntimeConfig(timeout_seconds=request.timeout)
    result = await run_python_code_async(code=request.code, config=config)

    return ExecuteResponse(
        stdout=result.stdout,
        stderr=result.stderr,
        exit_code=result.exit_code,
        timed_out=result.timed_out,
    )


@router.post("/benchmark/simulate", response_model=BenchmarkToolResponse)
async def api_simulate(request: BenchmarkToolRequest):
    """Physics-backed stability check."""
    try:
        component = _load_component(request.script_path)
        result = simulate(component)
        return BenchmarkToolResponse(
            success=result.success,
            message=result.summary,
            artifacts={"render_paths": result.render_paths},
        )
    except Exception as e:
        logger.error("api_benchmark_simulate_failed", error=str(e))
        return BenchmarkToolResponse(success=False, message=str(e))


@router.post("/benchmark/validate", response_model=BenchmarkToolResponse)
async def api_validate(request: BenchmarkToolRequest):
    """Geometric validity check."""
    try:
        component = _load_component(request.script_path)
        is_valid = validate(component)
        return BenchmarkToolResponse(
            success=is_valid,
            message="Validation successful" if is_valid else "Validation failed",
        )
    except Exception as e:
        logger.error("api_benchmark_validate_failed", error=str(e))
        return BenchmarkToolResponse(success=False, message=str(e))


@router.post("/benchmark/submit", response_model=BenchmarkToolResponse)
async def api_submit(request: BenchmarkToolRequest):
    """Handover to reviewer."""
    try:
        component = _load_component(request.script_path)
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
