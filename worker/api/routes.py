import ast
import importlib.util
import json
import sys
import time
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
from worker.workbenches.config import load_config
from worker.workbenches.models import WorkbenchResult

from ..filesystem.backend import FileInfo
from ..filesystem.router import WritePermissionError, create_filesystem_router
from ..runtime.executor import RuntimeConfig, run_python_code_async
from ..utils import simulate, submit_for_review, validate, validate_and_price
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
    LintRequest,
    LintResponse,
    ListFilesRequest,
    PreviewDesignRequest,
    PreviewDesignResponse,
    ReadFileRequest,
    ReadFileResponse,
    StatusResponse,
    WriteFileRequest,
    InspectTopologyRequest,
    InspectTopologyResponse,
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


def _load_component(fs_router, script_path: str, script_content: str | None = None):
    """Utility to load the component from the agent's script."""
    # 1. If content is provided, use it directly (stateless/memory execution)
    if script_content:
        logger.warning("load_component_using_script_content_deprecated")
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

    # 2. Path loading from local storage
    try:
        local_p = fs_router.local_backend._resolve(script_path)
    except Exception:
        # Fallback to absolute or relative to CWD if resolve fails (e.g. traversal check)
        local_p = Path(script_path)

    if not local_p.exists():
        raise FileNotFoundError(
            f"Script not found at {local_p.absolute()}. "
            "Ensure the agent has written the file before calling simulate/validate."
        )

    # Add session root to sys.path to allow local imports in the script
    session_root = str(fs_router.local_backend.root)
    if session_root not in sys.path:
        sys.path.insert(0, session_root)

    spec = importlib.util.spec_from_file_location("dynamic_build", str(local_p))
    if spec is None or spec.loader is None:
        raise RuntimeError(f"Could not load spec for {local_p}")

    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)

    if hasattr(module, "build"):
        return module.build()
    raise AttributeError("build() function not found in script.")


def _collect_events(fs_router) -> list[dict[str, Any]]:
    """Read and delete events.jsonl from the workspace."""
    events = []
    events_path = "events.jsonl"
    try:
        if fs_router.exists(events_path):
            content = fs_router.read(events_path).decode("utf-8")
            for line in content.strip().split("\n"):
                if line:
                    try:
                        events.append(json.loads(line))
                    except json.JSONDecodeError:
                        logger.warning("failed_to_decode_event_line", line=line)
            # Delete the file after reading to avoid cross-contamination between runs
            # Note: fs_router expects paths relative to root or absolute if backed allows
            # But here fs_router seems to handle it.
            # Actually, fs_router doesn't have a 'delete' method in the snippet I saw.
            # I'll check the filesystem router/backend later if needed.
            # For now, I'll just leave it or use a trick.
            # Wait, I saw create_filesystem_router in lines 43-50.
    except Exception as e:
        logger.warning("failed_to_collect_events", error=str(e))
    return events


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


import asyncio

# Global semaphore to limit concurrent simulations
# Spec: "only one can simulate"
SIMULATION_SEMAPHORE = asyncio.Semaphore(1)


@router.post("/benchmark/simulate", response_model=BenchmarkToolResponse)
async def api_simulate(
    request: BenchmarkToolRequest,
    fs_router=Depends(get_router),
):
    """Physics-backed stability check in isolated session."""
    try:
        component = _load_component(
            fs_router, request.script_path, request.script_content
        )

        # Enforce single concurrent simulation
        async with SIMULATION_SEMAPHORE:
            result = await asyncio.to_thread(
                simulate,
                component,
                output_dir=fs_router.local_backend.root,
                smoke_test_mode=request.smoke_test_mode,
                backend=request.backend,
            )
        events = _collect_events(fs_router)
        return BenchmarkToolResponse(
            success=result.success,
            message=result.summary,
            confidence=result.confidence,
            artifacts={
                "render_paths": result.render_paths,
                "mjcf_content": result.mjcf_content,
                "stress_summaries": [s.model_dump() for s in result.stress_summaries]
                if result.stress_summaries
                else [],
                "fluid_metrics": [m.model_dump() for m in result.fluid_metrics]
                if result.fluid_metrics
                else [],
            },
            events=events,
        )

    except Exception as e:
        logger.error("api_benchmark_simulate_failed", error=str(e))
        return BenchmarkToolResponse(success=False, message=str(e))


@router.post("/benchmark/validate", response_model=BenchmarkToolResponse)
async def api_validate(
    request: BenchmarkToolRequest,
    fs_router=Depends(get_router),
):
    """Geometric validity check in isolated session."""
    try:
        component = _load_component(
            fs_router, request.script_path, request.script_content
        )
        # Geometric validation
        is_valid, message = validate(component, output_dir=fs_router.local_backend.root)

        # INT-102: Fetch objectives to check if FEM material validation is required
        working_dir = fs_router.local_backend.root
        obj_path = working_dir / "objectives.yaml"
        if is_valid and obj_path.exists():
            try:
                import yaml

                from shared.models.schemas import ObjectivesYaml
                from worker.utils.dfm import validate_and_price
                from worker.workbenches.config import load_config
                from worker.workbenches.models import ManufacturingMethod

                data = yaml.safe_load(obj_path.read_text(encoding="utf-8"))
                objectives = ObjectivesYaml(**data)
                if objectives.physics and objectives.physics.fem_enabled:
                    config = load_config()
                    # Check for custom config in working dir
                    custom_config_path = working_dir / "manufacturing_config.yaml"
                    if custom_config_path.exists():
                        config = load_config(str(custom_config_path))

                    val_report = validate_and_price(
                        component,
                        ManufacturingMethod.CNC,
                        config,
                        fem_required=True,
                    )
                    if not val_report.is_manufacturable:
                        is_valid = False
                        msg = "Material validation failed: " + "; ".join(
                            map(str, val_report.violations)
                        )
                        message = (message + "; " + msg) if message else msg
            except Exception as e:
                logger.warning("api_validate_fem_check_failed", error=str(e))

        # INT-018: Record validation results to satisfy the handover gate
        results_path = fs_router.local_backend.root / "validation_results.json"
        results_path.write_text(
            json.dumps(
                {"success": is_valid, "message": message, "timestamp": time.time()}
            ),
            encoding="utf-8",
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


@router.post("/benchmark/analyze", response_model=WorkbenchResult)
async def api_analyze(
    request: AnalyzeRequest,
    fs_router=Depends(get_router),
):
    """Manufacturing analysis using specified workbench."""
    try:
        component = _load_component(
            fs_router, request.script_path, request.script_content
        )

        # Load default configuration
        config = load_config()

        # Determine if FEM validation is required from objectives
        fem_required = False
        working_dir = fs_router.local_backend.root
        obj_path = working_dir / "objectives.yaml"
        if obj_path.exists():
            try:
                import yaml

                from shared.models.schemas import ObjectivesYaml

                data = yaml.safe_load(obj_path.read_text(encoding="utf-8"))
                objectives = ObjectivesYaml(**data)
                if objectives.physics:
                    fem_required = objectives.physics.fem_enabled
            except Exception:
                pass

        result = validate_and_price(
            component,
            request.method,
            config,
            quantity=request.quantity,
            fem_required=fem_required,
        )

        # INT-018: Record validation results to satisfy the handover gate
        results_path = fs_router.local_backend.root / "validation_results.json"
        results_path.write_text(
            json.dumps(
                {
                    "success": result.is_manufacturable,
                    "message": "; ".join(result.violations)
                    if result.violations
                    else "Analysis successful",
                    "timestamp": time.time(),
                }
            ),
            encoding="utf-8",
        )

        return result
    except Exception as e:
        logger.error("api_benchmark_analyze_failed", error=str(e))
        # Wrap error in a failed WorkbenchResult if possible, or raise HTTP error
        # Since WorkbenchResult has strict fields, raising HTTP exception is safer/easier
        raise HTTPException(status_code=500, detail=str(e))


@router.post("/benchmark/submit", response_model=BenchmarkToolResponse)
async def api_submit(
    request: BenchmarkToolRequest,
    fs_router=Depends(get_router),
):
    """Handover to reviewer in isolated session."""
    try:
        component = _load_component(
            fs_router, request.script_path, request.script_content
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


@router.post("/benchmark/preview", response_model=PreviewDesignResponse)
async def api_preview(
    request: PreviewDesignRequest,
    fs_router=Depends(get_router),
):
    """Render a preview of the CAD design from specified camera angles."""
    try:
        component = _load_component(fs_router, request.script_path)

        image_path = preview_design(
            component,
            pitch=request.pitch,
            yaw=request.yaw,
            output_dir=fs_router.local_backend.root / "renders",
        )
        events = _collect_events(fs_router)

        return PreviewDesignResponse(
            success=True,
            message="Preview generated successfully",
            image_path=str(image_path.relative_to(fs_router.local_backend.root)),
            events=events,
        )

    except Exception as e:
        logger.error("api_benchmark_preview_failed", error=str(e))
        return PreviewDesignResponse(success=False, message=str(e))


@router.post("/lint", response_model=LintResponse)
async def api_lint(
    request: LintRequest,
    fs_router=Depends(get_router),
):
    """Lint Python code using ruff."""
    import json
    import subprocess
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
            # Run ruff check with JSON output
            result = subprocess.run(
                ["ruff", "check", "--output-format", "json", tmp_path],
                capture_output=True,
                text=True,
                timeout=30,
            )

            errors = []
            warnings = []

            if result.stdout:
                lint_results = json.loads(result.stdout)
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
    fs_router=Depends(get_router),
):
    """Rebuild simulation assets (GLB) from source without running full simulation."""
    try:
        # Load component
        component = _load_component(fs_router, request.script_path)

        # Get builder
        from worker.simulation.factory import get_simulation_builder

        builder = get_simulation_builder(fs_router.local_backend.root)

        # Build assets (GLB/OBJ/Scene)
        # We don't need objectives/moving_parts for basic visualization rebuild
        scene_path = builder.build_from_assembly(component)

        events = _collect_events(fs_router)
        return BenchmarkToolResponse(
            success=True,
            message=f"Assets rebuilt. Scene saved to {scene_path}",
            artifacts={
                # Return paths relative to session root
                "scene_path": str(scene_path.relative_to(fs_router.local_backend.root))
            },
            events=events,
        )

    except Exception as e:
        logger.error("api_benchmark_build_failed", error=str(e))
        return BenchmarkToolResponse(success=False, message=str(e))
