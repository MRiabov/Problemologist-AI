import asyncio
import io
import tarfile
import tempfile
from pathlib import Path
from typing import Any

import structlog
from temporalio import activity

from shared.simulation.schemas import SimulatorBackendType
from shared.workers.loader import load_component_from_script
from worker_heavy.utils.validation import simulate_subprocess, validate
from worker_heavy.utils.preview import preview_design

logger = structlog.get_logger(__name__)


def _extract_bundle(bundle_bytes: bytes, target_dir: Path):
    """Extract gzipped tarball to target directory using system tar."""
    import subprocess

    with tempfile.NamedTemporaryFile(suffix=".tar.gz", delete=False) as tf:
        tf.write(bundle_bytes)
        tf_path = tf.name
    try:
        subprocess.run(
            ["tar", "-zxf", tf_path, "-C", str(target_dir), "--no-same-owner"],
            check=True,
            capture_output=True,
            text=True,
        )
    except subprocess.CalledProcessError as spe:
        logger.error("tar_subprocess_failed", stderr=spe.stderr)
        raise RuntimeError(f"tar extraction failed: {spe.stderr}")
    finally:
        if Path(tf_path).exists():
            Path(tf_path).unlink()


@activity.defn(name="worker_run_simulation")
async def run_simulation_activity(params: dict[str, Any]) -> dict[str, Any]:
    """Execute physics simulation from a session bundle."""
    bundle_bytes = params["bundle_bytes"]
    script_path = params["script_path"]
    backend = params["backend"]
    smoke_test_mode = params.get("smoke_test_mode", False)
    session_id = params["session_id"]

    with tempfile.TemporaryDirectory() as tmpdir:
        root = Path(tmpdir)
        _extract_bundle(bundle_bytes, root)

        # backend might be a string from temporal, convert to enum
        backend_type = SimulatorBackendType(backend)

        result = await asyncio.to_thread(
            simulate_subprocess,
            script_path=str(root / script_path),
            session_root=str(root),
            output_dir=root,
            smoke_test_mode=smoke_test_mode,
            backend=backend_type,
            session_id=session_id,
        )
        return result.model_dump()


@activity.defn(name="worker_validate_design")
async def validate_design_activity(params: dict[str, Any]) -> dict[str, Any]:
    """Execute geometric validation from a session bundle."""
    bundle_bytes = params["bundle_bytes"]
    script_path = params["script_path"]
    session_id = params["session_id"]

    with tempfile.TemporaryDirectory() as tmpdir:
        root = Path(tmpdir)
        _extract_bundle(bundle_bytes, root)

        component = load_component_from_script(
            script_path=root / script_path,
            session_root=root,
        )

        is_valid, message = await asyncio.to_thread(
            validate,
            component,
            output_dir=root,
            session_id=session_id,
        )

        return {"success": is_valid, "message": message}


@activity.defn(name="worker_preview_design")
async def preview_design_activity(params: dict[str, Any]) -> dict[str, Any]:
    """Render design preview from a session bundle."""
    bundle_bytes = params["bundle_bytes"]
    script_path = params["script_path"]
    pitch = params.get("pitch", -45.0)
    yaw = params.get("yaw", 45.0)

    with tempfile.TemporaryDirectory() as tmpdir:
        root = Path(tmpdir)
        _extract_bundle(bundle_bytes, root)

        component = load_component_from_script(
            script_path=root / script_path,
            session_root=root,
        )

        renders_dir = root / "renders"
        renders_dir.mkdir(exist_ok=True)

        image_path = await asyncio.to_thread(
            preview_design,
            component,
            pitch=pitch,
            yaw=yaw,
            output_dir=renders_dir,
        )

        # Since it's a temp dir, we might want to return the image bytes or upload to S3
        # For now, let's assume we return the bytes or the caller handles it.
        # Spec says "Stateless Simulation Payloads".
        # Usually we want the heavy worker to upload results to S3.

        return {
            "success": True,
            "image_bytes": image_path.read_bytes() if image_path.exists() else None,
            "filename": image_path.name if image_path.exists() else None,
        }
